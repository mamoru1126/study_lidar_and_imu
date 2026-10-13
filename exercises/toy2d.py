"""2D のおもちゃ世界 (演習 3, 4, 7 で共通利用)

本物の 3D LiDAR の代わりに、2D の LiDAR (360 本のレーザーを水平に打つ) を
シミュレーションします。考え方は 3D と同じで、式の次元が 3→2 になるだけです。

  - make_environment(): 壁・柱を線分の集合として作る
  - raycast():          ある姿勢から 2D LiDAR で壁までの距離を測る
  - estimate_covariances(): 各点の周りの「面 (2D では線)」の形を共分散で表す (GICP の前処理)
  - GaussianVoxelMap:   点群をボクセルに分け、ボクセルごとに平均・共分散を持つ (VGICP の地図)
  - vgicp_terms():      VGICP の誤差とヤコビアンを計算する (演習 4, 7 の心臓部)

姿勢は (x, y, θ) の 3 次元ベクトルで表します。
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from common import drot2, rot2


# ---------------------------------------------------------------------------
# 環境
# ---------------------------------------------------------------------------
def _box(cx, cy, w, h):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def make_environment() -> np.ndarray:
    """壁と柱を線分 (M, 2, 2) で返す。

    外壁 40m x 24m の建物の中に、中央ブロックと柱・斜めの壁を置いた環境。
    ロボットは中央ブロックの周りをぐるっと一周する。
    """
    segs = []
    segs += _box(0.0, 0.0, 40.0, 24.0)  # 外壁
    segs += _box(0.0, 0.0, 22.0, 8.0)  # 中央ブロック
    # 柱 (長い廊下で前後方向の手がかりになる)
    for x in [-13.0, -6.5, -1.0, 4.5, 9.5, 14.0]:
        segs += _box(x, -10.6, 0.8, 0.8)
        segs += _box(x + 2.0, 10.6, 0.8, 0.8)
    for x in [-8.0, 2.0, 7.5]:
        segs += _box(x, -5.0, 1.0, 0.6)
        segs += _box(x - 3.0, 5.0, 1.0, 0.6)
    # 四隅の斜めの壁
    segs += [((-20.0, -8.0), (-16.0, -12.0)), ((16.0, 12.0), (20.0, 8.0))]
    segs += [((-20.0, 8.0), (-17.5, 12.0)), ((17.0, -12.0), (20.0, -9.0))]
    # 中央ブロック左右の出っ張り
    segs += _box(-11.6, 1.5, 1.2, 2.0)
    segs += _box(11.6, -1.5, 1.2, 2.0)
    # 壁から突き出た短い仕切り (棚やドア枠のイメージ)。
    # 廊下は壁が平行に続くので前後方向 (x) が決まりにくい。こうした凹凸が手がかりになる
    for x in [-14.5, -9.5, -4.0, 1.5, 6.0, 11.0, 15.2]:
        segs += [((x, -12.0), (x, -11.0))]
    for x in [-13.5, -8.5, -3.0, 2.5, 7.0, 12.0, 15.0]:
        segs += [((x, 12.0), (x, 11.0))]
    for x in [-5.5, 0.5, 5.5]:
        segs += [((x, -4.0), (x, -4.5)), ((x - 1.0, 4.0), (x - 1.0, 4.5))]
    return np.array(segs, dtype=float)


def sample_map_points(segs: np.ndarray, spacing=0.08, noise=0.01, rng=None) -> np.ndarray:
    """線分の上に点を並べて「事前に作った地図点群」を作る"""
    rng = rng or np.random.default_rng(0)
    pts = []
    for a, b in segs:
        n = max(2, int(np.linalg.norm(b - a) / spacing))
        t = np.linspace(0, 1, n)[:, None]
        pts.append(a + t * (b - a))
    pts = np.vstack(pts)
    return pts + rng.normal(scale=noise, size=pts.shape)


def raycast(pose, segs: np.ndarray, n_beams=360, max_range=15.0, noise=0.02, rng=None) -> np.ndarray:
    """姿勢 pose=(x, y, θ) から 2D LiDAR で観測した点群をセンサ座標系で返す"""
    rng = rng or np.random.default_rng()
    x, y, th = pose
    ang = np.linspace(-np.pi, np.pi, n_beams, endpoint=False) + th
    d = np.stack([np.cos(ang), np.sin(ang)], axis=1)  # (B,2)
    a = segs[:, 0, :]  # (M,2)
    e = segs[:, 1, :] - a  # (M,2)
    o = np.array([x, y])
    # o + r d = a + u e を r, u について解く (全ビーム × 全線分を一度に)
    denom = d[:, None, 0] * (-e[None, :, 1]) - d[:, None, 1] * (-e[None, :, 0])
    ao = a[None, :, :] - o[None, None, :]
    with np.errstate(divide="ignore", invalid="ignore"):
        r = (ao[..., 0] * (-e[None, :, 1]) - ao[..., 1] * (-e[None, :, 0])) / denom
        u = (d[:, None, 0] * ao[..., 1] - d[:, None, 1] * ao[..., 0]) / denom
    valid = (np.abs(denom) > 1e-12) & (r > 0.05) & (u >= 0) & (u <= 1)
    r = np.where(valid, r, np.inf).min(axis=1)
    hit = r < max_range
    r = r[hit] + rng.normal(scale=noise, size=hit.sum())
    local_ang = ang[hit] - th
    return np.stack([r * np.cos(local_ang), r * np.sin(local_ang)], axis=1)


def voxel_downsample(pts: np.ndarray, res: float) -> np.ndarray:
    """ボクセルグリッドで間引く (各ボクセルの重心を残す)"""
    keys = np.floor(pts / res).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.ravel()
    n = inv.max() + 1
    out = np.zeros((n, 2))
    np.add.at(out, inv, pts)
    cnt = np.bincount(inv, minlength=n)[:, None]
    return out / cnt


# ---------------------------------------------------------------------------
# GICP の前処理: 点ごとの共分散
# ---------------------------------------------------------------------------
def estimate_covariances(pts: np.ndarray, k=8, eps=1e-3) -> np.ndarray:
    """各点の近傍 k 点から共分散を求め、GICP 流に正則化する。

    壁の上の点なら「壁に沿う方向には大きく、壁に垂直な方向には小さい」共分散になる。
    GICP では固有値を (1, ε) に置き換えて、形だけを残す (面/線のモデル化)。
    """
    k = min(k, len(pts))
    tree = cKDTree(pts)
    _, idx = tree.query(pts, k=k)
    nb = pts[idx]  # (N,k,2)
    mu = nb.mean(axis=1, keepdims=True)
    d = nb - mu
    C = np.einsum("nki,nkj->nij", d, d) / k
    _, vecs = np.linalg.eigh(C)  # 固有値は昇順 → 0 列目が法線方向
    D = np.diag([eps, 1.0])
    return vecs @ D @ np.transpose(vecs, (0, 2, 1))


# ---------------------------------------------------------------------------
# VGICP 用のガウス分布ボクセル地図
# ---------------------------------------------------------------------------
class GaussianVoxelMap:
    """点群をボクセルに分け、各ボクセルに (平均, 共分散) を 1 つずつ持たせた地図。

    VGICP は「点 → 近傍点」を kd-tree で探す代わりに、点が入るボクセルを
    ハッシュ (ここではソート済みキーの二分探索) で一発で引く。これが高速化・GPU 化の鍵。
    """

    OFFSET = 1 << 20

    def __init__(self, pts: np.ndarray, covs: np.ndarray, resolution: float, min_points=1):
        self.res = resolution
        keys = self._keys(pts)
        uniq, inv, cnt = np.unique(keys, return_inverse=True, return_counts=True)
        inv = inv.ravel()
        mean = np.zeros((len(uniq), 2))
        cov = np.zeros((len(uniq), 2, 2))
        np.add.at(mean, inv, pts)
        np.add.at(cov, inv, covs)
        mean /= cnt[:, None]
        cov /= cnt[:, None, None]  # VGICP: 点の共分散の平均をボクセルの共分散とする
        keep = cnt >= min_points
        self.keys = uniq[keep]
        self.mean = mean[keep]
        self.cov = cov[keep]

    def _keys(self, pts):
        ij = np.floor(pts / self.res).astype(np.int64) + self.OFFSET
        return ij[:, 0] * (2 * self.OFFSET) + ij[:, 1]

    def lookup(self, pts: np.ndarray) -> np.ndarray:
        """各点が入るボクセルの番号を返す (無ければ -1)"""
        if len(self.keys) == 0:
            return np.full(len(pts), -1)
        k = self._keys(pts)
        pos = np.searchsorted(self.keys, k)
        pos = np.clip(pos, 0, len(self.keys) - 1)
        return np.where(self.keys[pos] == k, pos, -1)

    def __len__(self):
        return len(self.keys)


def make_voxelmaps(pts, covs, resolutions=(0.5, 1.0)):
    """マルチ解像度のボクセル地図 (GLIM の voxelmap_levels に相当)"""
    return [GaussianVoxelMap(pts, covs, r) for r in resolutions]


# ---------------------------------------------------------------------------
# VGICP の誤差とヤコビアン
# ---------------------------------------------------------------------------
def _chol2(A):
    """2x2 の対称正定値行列 (N,2,2) のコレスキー分解 L (A = L L^T)"""
    a, b, c = A[:, 0, 0], A[:, 0, 1], A[:, 1, 1]
    l11 = np.sqrt(a)
    l21 = b / l11
    l22 = np.sqrt(np.maximum(c - l21**2, 1e-12))
    L = np.zeros_like(A)
    L[:, 0, 0], L[:, 1, 0], L[:, 1, 1] = l11, l21, l22
    return L


def vgicp_terms(target_pose, source_pose, src_pts, src_covs, voxelmap: GaussianVoxelMap):
    """VGICP の白色化済み誤差 e とヤコビアンを返す。

    target_pose が None なら、ターゲット (地図) は世界座標に固定されているとみなす。

    点 p_k (ソース座標系) をターゲット座標系に移した位置を q_k とすると
        q_k = R_t^T (R_s p_k + t_s - t_t)
        d_k = μ_v - q_k                       (μ_v: q_k が入るボクセルの平均)
        Ω_k = (C_v + R_ts C_k R_ts^T)^{-1}     (分布同士の共分散の和の逆行列)
        コスト = Σ_k d_k^T Ω_k d_k = Σ_k |L_k^T d_k|^2   (Ω_k = L_k L_k^T)

    戻り値: e (n,2), J_t (n,2,3) または None, J_s (n,2,3), 対応点数 n
    """
    xs, ys, ths = source_pose
    Rs, ts = rot2(ths), np.array([xs, ys])
    if target_pose is None:
        Rt, tt, tht = np.eye(2), np.zeros(2), 0.0
    else:
        xt, yt, tht = target_pose
        Rt, tt = rot2(tht), np.array([xt, yt])

    w = src_pts @ Rs.T + ts  # 世界座標
    q = (w - tt) @ Rt  # ターゲット座標 (R_t^T (w - t_t))
    vid = voxelmap.lookup(q)
    m = vid >= 0
    n = int(m.sum())
    if n == 0:
        return np.zeros((0, 2)), None, np.zeros((0, 2, 3)), 0
    q, p, Ck = q[m], src_pts[m], src_covs[m]
    mu, Cv = voxelmap.mean[vid[m]], voxelmap.cov[vid[m]]
    R_ts = Rt.T @ Rs
    S = Cv + R_ts @ Ck @ R_ts.T
    Omega = np.linalg.inv(S)
    L = _chol2(Omega)
    d = mu - q
    e = np.einsum("nji,nj->ni", L, d)  # L^T d

    # ∂q/∂(source pose) = [R_t^T, R_t^T dR_s p]
    dq_s = np.zeros((n, 2, 3))
    dq_s[:, :, 0:2] = Rt.T
    dq_s[:, :, 2] = (p @ drot2(ths).T) @ Rt
    J_s = -np.einsum("nji,njk->nik", L, dq_s)
    J_t = None
    if target_pose is not None:
        # ∂q/∂(target pose) = [-R_t^T, dR_t^T (w - t_t)]
        dq_t = np.zeros((n, 2, 3))
        dq_t[:, :, 0:2] = -Rt.T
        dq_t[:, :, 2] = (w[m] - tt) @ drot2(tht)
        J_t = -np.einsum("nji,njk->nik", L, dq_t)
    return e, J_t, J_s, n


def align_vgicp(src_pts, src_covs, voxelmaps, init_pose, iters=20, verbose=False):
    """地図 (世界座標に固定) に対してスキャンを VGICP で位置合わせする"""
    x = np.array(init_pose, dtype=float)
    n = 0
    for it in range(iters):
        H = np.zeros((3, 3))
        g = np.zeros(3)
        n = 0
        for vm in voxelmaps:
            e, _, J, k = vgicp_terms(None, x, src_pts, src_covs, vm)
            n += k
            if k:
                H += np.einsum("nki,nkj->ij", J, J)
                g += np.einsum("nki,nk->i", J, e)
        if n < 10:
            break
        dx = np.linalg.solve(H + 1e-6 * np.eye(3), -g)
        x += dx
        if verbose:
            print(f"  iter {it}: pose={x}, 対応点数={n}")
        if np.linalg.norm(dx) < 1e-6:
            break
    return x, n
