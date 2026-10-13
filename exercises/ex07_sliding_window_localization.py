"""演習 7 (総まとめ): 論文の手法を 2D のおもちゃ世界で再現する

対応する資料: docs/06_fixed_lag_smoothing.md, docs/07_map_based_localization.md,
             docs/08_paper_walkthrough.md

論文の目的関数 (の 2D 版) をそのまま組み立てて、スライディングウィンドウで最適化します。

    F(X) = Σ_i Σ_{j=i-2}^{i-1} f^P(x_i, x_j)    ... 連続フレーム間の VGICP (スキャン間マッチング)
         + Σ_i f^P(x_i, m)                       ... 地図点群 m との VGICP (地図マッチング)
         + Σ_i f^I(x_{i-1}, x_i)                 ... IMU 事前積分因子
         + C(X)                                  ... 周辺化で生まれた事前分布

状態 x_i = [x, y, θ, vx, vy, b_ax, b_ay, b_g]  (姿勢・速度・IMU バイアス)

地図は x < 2 m の範囲にしか無い (論文の実験と同じく、地図の大部分を消してある)。
ロボットは地図のある左側から出発し、地図の無い右側を通って戻ってくる。

比較する 3 つの手法
  A. 地図マッチングのみ   : 等速モデルで予測 → 地図と VGICP。地図に強く依存する従来法の代表
  B. LiDAR-IMU のみ       : 提案手法から地図因子を抜いたもの (= 地図を使わないオドメトリ)
  C. 提案手法             : スキャン間 + 地図 + IMU を同じウィンドウで同時に最適化

実行: python exercises/ex07_sliding_window_localization.py   (1〜3 分かかります)
出力: docs/figures/ex07_*.png と html/data/sliding_window_result.js (HTML 版のデモ用)
"""

from __future__ import annotations

import json
import time

import numpy as np

from common import (
    C_AQUA,
    C_BLUE,
    C_INK,
    C_MUTED,
    C_ORANGE,
    C_SHADE,
    DATA_DIR,
    drot2,
    fig_path,
    rot2,
    setup_matplotlib,
    wrap_angle,
)
from toy2d import (
    GaussianVoxelMap,
    align_vgicp,
    estimate_covariances,
    make_environment,
    make_voxelmaps,
    raycast,
    sample_map_points,
    vgicp_terms,
    voxel_downsample,
)

# ===========================================================================
# 設定
# ===========================================================================
LIDAR_HZ = 5  # LiDAR のフレームレート
IMU_HZ = 100  # IMU のサンプリング周波数
SMOOTHER_LAG = 3.0  # ウィンドウの長さ [s] (論文の例は 5 秒。ここは計算を軽くするため 3 秒)
FULL_CONNECTION = 2  # 新しいフレームを直前の何フレームとマッチングするか (論文: j = i-2, i-1)
VOXEL_RES = (0.5, 1.0)  # マルチ解像度ボクセル (GLIM の voxelmap_levels = 2 に相当)
MAP_X_MAX = 2.0  # 地図点群は x < MAP_X_MAX の範囲にしか無い
GN_ITERS = 4  # 1 フレームあたりのガウス・ニュートン反復回数

# IMU の誤差 (真値)
BIAS_A_TRUE = np.array([0.08, -0.05])  # 加速度バイアス [m/s^2]
BIAS_G_TRUE = 0.01  # ジャイロバイアス [rad/s] (≒ 0.57 deg/s)
SIGMA_A = 0.03  # 加速度ノイズ [m/s^2] (1 サンプル)
SIGMA_G = 0.003  # ジャイロノイズ [rad/s] (1 サンプル)
BIAS_RW_A = 1e-3  # バイアスのランダムウォーク [m/s^2/√s]
BIAS_RW_G = 1e-4  # [rad/s/√s]

rng = np.random.default_rng(7)


# ===========================================================================
# 真の軌跡: 中央ブロックの周りを 1.25 周する (超楕円の上を速度を変えながら走る)
# ===========================================================================
A_AX, B_AX, N_SE = 15.5, 8.0, 4.0
PHI0, PHI1 = np.pi, np.pi + 2.5 * np.pi


def curve(phi):
    r = (np.abs(np.cos(phi) / A_AX) ** N_SE + np.abs(np.sin(phi) / B_AX) ** N_SE) ** (-1.0 / N_SE)
    return np.stack([r * np.cos(phi), r * np.sin(phi)], axis=-1)


phi_grid = np.linspace(PHI0, PHI1, 400001)
pts_grid = curve(phi_grid)
s_grid = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts_grid, axis=0), axis=1))])


def speed(t):
    return 1.6 + 0.4 * np.sin(2 * np.pi * t / 9.0)


def dist(t):
    return 1.6 * t + 0.4 * 9.0 / (2 * np.pi) * (1 - np.cos(2 * np.pi * t / 9.0))


T_END = 0.0
while dist(T_END) < s_grid[-1] - 0.5:
    T_END += 0.01


def motion(t):
    """時刻 t の (位置, 向き θ, 世界座標の速度, 世界座標の加速度, 角速度 ω)"""
    t = np.atleast_1d(t)
    phi = np.interp(dist(t), s_grid, phi_grid)
    h = 1e-4
    p = curve(phi)
    d1 = (curve(phi + h) - curve(phi - h)) / (2 * h)
    d2 = (curve(phi + h) - 2 * p + curve(phi - h)) / h**2
    n1 = np.linalg.norm(d1, axis=1)
    tang = d1 / n1[:, None]
    normal = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    kappa = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / n1**3  # 曲率
    v = speed(t)
    dv = 0.4 * (2 * np.pi / 9.0) * np.cos(2 * np.pi * t / 9.0)
    theta = np.arctan2(tang[:, 1], tang[:, 0])
    vel = v[:, None] * tang
    acc = dv[:, None] * tang + (v**2 * kappa)[:, None] * normal
    omega = v * kappa
    return p, theta, vel, acc, omega


# ===========================================================================
# センサのシミュレーション
# ===========================================================================
segs = make_environment()
frame_t = np.arange(0.0, T_END, 1.0 / LIDAR_HZ)
NF = len(frame_t)
gt_p, gt_th, gt_v, _, _ = motion(frame_t)
gt = np.column_stack([gt_p, gt_th])

imu_t = np.arange(0.0, frame_t[-1] + 1e-9, 1.0 / IMU_HZ)  # 区間 [t_k, t_k + dt] の計測
mid = imu_t + 0.5 / IMU_HZ
_, th_m, _, acc_m, om_m = motion(mid)
imu_gyro = om_m + BIAS_G_TRUE + rng.normal(scale=SIGMA_G, size=len(mid))
imu_acc = np.einsum("nji,nj->ni", np.stack([rot2(a) for a in th_m]), acc_m)  # R^T a (ボディ座標)
imu_acc += BIAS_A_TRUE + rng.normal(scale=SIGMA_A, size=imu_acc.shape)

print(f"走行時間 {frame_t[-1]:.1f} 秒, 走行距離 {dist(frame_t[-1]):.1f} m, LiDAR {NF} フレーム")
print("LiDAR スキャンを生成中...")
scans, scan_covs, scan_vms = [], [], []
for k in range(NF):
    pts = voxel_downsample(raycast(gt[k], segs, rng=rng), 0.25)
    covs = estimate_covariances(pts)
    scans.append(pts)
    scan_covs.append(covs)
    scan_vms.append(make_voxelmaps(pts, covs, VOXEL_RES))

# 事前地図: 環境全体の点群から、x >= MAP_X_MAX の部分を削除する
map_all = sample_map_points(segs, rng=rng)
map_pts = map_all[map_all[:, 0] < MAP_X_MAX]
map_vms = make_voxelmaps(map_pts, estimate_covariances(map_pts, k=10), VOXEL_RES)
in_map_region = gt[:, 0] < MAP_X_MAX
print(f"地図点数 {len(map_pts)} (全体 {len(map_all)} 点のうち x < {MAP_X_MAX} m のみ)")


# ===========================================================================
# IMU 事前積分 (2D 版)
# ===========================================================================
S90 = np.array([[0.0, -1.0], [1.0, 0.0]])  # dR(θ)/dθ = R(θ) S90


class Preintegration:
    """区間 [t_i, t_j] の IMU 計測を事前積分する。

    Δθ = Σ (ω - b_g) dt
    Δv = Σ R(Δθ_k) (a - b_a) dt
    Δp = Σ [Δv_k dt + ½ R(Δθ_k) (a - b_a) dt²]
    バイアスに関するヤコビアンと、ノイズの共分散も同時に計算する。
    """

    def __init__(self, gyro, acc, ba, bg):
        dt = 1.0 / IMU_HZ
        self.ba0, self.bg0 = ba.copy(), float(bg)
        self.dT = len(gyro) * dt
        dth, dv, dp = 0.0, np.zeros(2), np.zeros(2)
        J_th_bg = 0.0
        J_v_ba, J_v_bg = np.zeros((2, 2)), np.zeros(2)
        J_p_ba, J_p_bg = np.zeros((2, 2)), np.zeros(2)
        cov = np.zeros((5, 5))  # 順番: [θ, v(2), p(2)]
        Q = np.diag([SIGMA_G**2, SIGMA_A**2, SIGMA_A**2])
        for w, a in zip(gyro, acc):
            ah = a - ba
            R = rot2(dth)
            RSa = R @ S90 @ ah
            # 共分散の伝播
            A = np.eye(5)
            A[1:3, 0] = RSa * dt
            A[3:5, 0] = 0.5 * RSa * dt**2
            A[3:5, 1:3] = np.eye(2) * dt
            B = np.zeros((5, 3))
            B[0, 0] = dt
            B[1:3, 1:3] = R * dt
            B[3:5, 1:3] = 0.5 * R * dt**2
            cov = A @ cov @ A.T + B @ Q @ B.T
            # ヤコビアン (古い値を使うので p → v → θ の順)
            J_p_ba = J_p_ba + J_v_ba * dt - 0.5 * R * dt**2
            J_p_bg = J_p_bg + J_v_bg * dt + 0.5 * RSa * J_th_bg * dt**2
            J_v_ba = J_v_ba - R * dt
            J_v_bg = J_v_bg + RSa * J_th_bg * dt
            J_th_bg = J_th_bg - dt
            # 積分
            dp = dp + dv * dt + 0.5 * R @ ah * dt**2
            dv = dv + R @ ah * dt
            dth = dth + (w - bg) * dt
        self.dth, self.dv, self.dp = dth, dv, dp
        self.J = (J_th_bg, J_v_ba, J_v_bg, J_p_ba, J_p_bg)
        self.sqrt_info = np.linalg.cholesky(np.linalg.inv(cov + 1e-12 * np.eye(5))).T

    def corrected(self, ba, bg):
        """バイアスが変わったときの 1 次補正 (積分のやり直しが不要)"""
        J_th_bg, J_v_ba, J_v_bg, J_p_ba, J_p_bg = self.J
        dba, dbg = ba - self.ba0, bg - self.bg0
        return (
            self.dth + J_th_bg * dbg,
            self.dv + J_v_ba @ dba + J_v_bg * dbg,
            self.dp + J_p_ba @ dba + J_p_bg * dbg,
        )

    def predict(self, xi):
        dth, dv, dp = self.corrected(xi[5:7], xi[7])
        Ri = rot2(xi[2])
        xj = xi.copy()
        xj[0:2] = xi[0:2] + xi[3:5] * self.dT + Ri @ dp
        xj[2] = wrap_angle(xi[2] + dth)
        xj[3:5] = xi[3:5] + Ri @ dv
        return xj

    def residual(self, xi, xj):
        dth, dv, dp = self.corrected(xi[5:7], xi[7])
        Ri = rot2(xi[2])
        r = np.zeros(5)
        r[0] = wrap_angle(xj[2] - xi[2] - dth)
        r[1:3] = Ri.T @ (xj[3:5] - xi[3:5]) - dv
        r[3:5] = Ri.T @ (xj[0:2] - xi[0:2] - xi[3:5] * self.dT) - dp
        return self.sqrt_info @ r


def imu_between(i, j):
    k0, k1 = int(round(frame_t[i] * IMU_HZ)), int(round(frame_t[j] * IMU_HZ))
    return imu_gyro[k0:k1], imu_acc[k0:k1]


# ===========================================================================
# 因子 (factor)。どれも linearize(states) -> (keys, H, g, cost) を返す
#   cost = ½ Σ |r|²,  H = J^T J,  g = J^T r   (状態 1 個あたり 8 次元)
# ===========================================================================
def _from_residual(keys, rfun, states, eps=1e-6):
    """残差関数を数値微分して H, g を作る (小さな因子用)"""
    x = np.concatenate([states[k] for k in keys])
    r0 = rfun(x)
    J = np.zeros((len(r0), len(x)))
    for c in range(len(x)):
        d = np.zeros(len(x))
        d[c] = eps
        J[:, c] = (rfun(x + d) - rfun(x - d)) / (2 * eps)
    return keys, J.T @ J, J.T @ r0, 0.5 * r0 @ r0


class PriorFactor:
    def __init__(self, key, z, sigmas):
        self.keys, self.z, self.w = [key], z.copy(), 1.0 / np.asarray(sigmas)

    def linearize(self, states):
        def r(x):
            d = x - self.z
            d[2] = wrap_angle(d[2])
            return d * self.w

        return _from_residual(self.keys, r, states)


class ImuFactor:
    def __init__(self, i, j, preint):
        self.keys, self.pre = [i, j], preint

    def linearize(self, states):
        return _from_residual(self.keys, lambda x: self.pre.residual(x[:8], x[8:]), states)


class BiasFactor:
    def __init__(self, i, j, dT):
        self.keys = [i, j]
        self.w = 1.0 / (np.array([BIAS_RW_A, BIAS_RW_A, BIAS_RW_G]) * np.sqrt(dT))

    def linearize(self, states):
        return _from_residual(self.keys, lambda x: (x[13:16] - x[5:8]) * self.w, states)


class ScanFactor:
    """VGICP 因子: target が None なら地図 (世界座標に固定) とのマッチング"""

    def __init__(self, target, source, voxelmap):
        self.target, self.source, self.vm = target, source, voxelmap
        self.keys = [source] if target is None else [target, source]

    def linearize(self, states):
        tp = None if self.target is None else states[self.target][:3]
        e, Jt, Js, n = vgicp_terms(tp, states[self.source][:3], scans[self.source], scan_covs[self.source], self.vm)
        dim = 8 * len(self.keys)
        H, g = np.zeros((dim, dim)), np.zeros(dim)
        if n == 0:
            return self.keys, H, g, 0.0
        Js8 = np.zeros((n, 2, 8))
        Js8[:, :, :3] = Js
        if Jt is None:
            Jall = Js8
        else:
            Jt8 = np.zeros((n, 2, 8))
            Jt8[:, :, :3] = Jt
            Jall = np.concatenate([Jt8, Js8], axis=2)
        H = np.einsum("nki,nkj->ij", Jall, Jall)
        g = np.einsum("nki,nk->i", Jall, e)
        return self.keys, H, g, 0.5 * float(np.sum(e * e))


class LinearPrior:
    """周辺化で生まれる事前分布 C(X): r = S (x ⊟ x0) + r0"""

    def __init__(self, keys, x0, S, r0):
        self.keys, self.x0, self.S, self.r0 = keys, x0, S, r0

    def linearize(self, states):
        d = np.concatenate([states[k] for k in self.keys]) - self.x0
        d[2::8] = wrap_angle(d[2::8])
        r = self.S @ d + self.r0
        return self.keys, self.S.T @ self.S, self.S.T @ r, 0.5 * r @ r


def assemble(states, keys, factors):
    index = {k: n for n, k in enumerate(keys)}
    H = np.zeros((8 * len(keys), 8 * len(keys)))
    g = np.zeros(8 * len(keys))
    cost = 0.0
    for f in factors:
        fk, Hf, gf, c = f.linearize(states)
        cost += c
        idx = np.concatenate([np.arange(8 * index[k], 8 * index[k] + 8) for k in fk])
        H[np.ix_(idx, idx)] += Hf
        g[idx] += gf
    return H, g, cost


# ===========================================================================
# 手法 B, C: スライディングウィンドウ (固定ラグ平滑化) による推定
# ===========================================================================
class WindowEstimator:
    def __init__(self, use_map: bool):
        self.use_map = use_map
        self.states: dict[int, np.ndarray] = {}
        self.factors: list = []
        self.window_start = []

    def initialize(self):
        x0 = np.zeros(8)
        x0[0:3], x0[3:5] = gt[0], gt_v[0]  # 初期位置・速度は既知とする (自己位置推定の前提)
        self.states[0] = x0
        self.factors.append(PriorFactor(0, x0, [0.01, 0.01, 0.005, 0.05, 0.05, 0.2, 0.2, 0.05]))
        self._add_map_factor(0)
        self.optimize()

    def _add_map_factor(self, j):
        if self.use_map:
            for vm in map_vms:
                self.factors.append(ScanFactor(None, j, vm))

    def add_frame(self, j):
        i = j - 1
        gyro, acc = imu_between(i, j)
        pre = Preintegration(gyro, acc, self.states[i][5:7], self.states[i][7])
        self.states[j] = pre.predict(self.states[i])  # IMU による予測 = 最適化の初期値
        self.factors.append(ImuFactor(i, j, pre))
        self.factors.append(BiasFactor(i, j, pre.dT))
        for target in range(j - FULL_CONNECTION, j):  # 直前のフレームとの VGICP
            if target in self.states:
                for vm in scan_vms[target]:
                    self.factors.append(ScanFactor(target, j, vm))
        self._add_map_factor(j)
        self.optimize()
        while frame_t[j] - frame_t[min(self.states)] > SMOOTHER_LAG:
            self.marginalize_oldest()
        self.window_start.append(min(self.states))
        return self.states[j].copy()

    def optimize(self):
        keys = sorted(self.states)
        for _ in range(GN_ITERS):
            H, g, _ = assemble(self.states, keys, self.factors)
            dx = np.linalg.solve(H + 1e-6 * np.eye(len(g)), -g)
            for n, k in enumerate(keys):
                self.states[k] = self.states[k] + dx[8 * n : 8 * n + 8]
                self.states[k][2] = wrap_angle(self.states[k][2])
            if np.abs(dx).max() < 1e-6:
                break

    def marginalize_oldest(self):
        m = min(self.states)
        touching = [f for f in self.factors if m in f.keys]
        self.factors = [f for f in self.factors if m not in f.keys]
        nb = sorted({k for f in touching for k in f.keys} - {m})
        keys = [m] + nb
        H, g, _ = assemble(self.states, keys, touching)
        # シューア補行列で m を消去: H' = H_rr - H_rm H_mm^{-1} H_mr
        Hmm, Hmr, Hrr = H[:8, :8], H[:8, 8:], H[8:, 8:]
        Hmm_inv = np.linalg.inv(Hmm + 1e-9 * np.eye(8))
        Hp = Hrr - Hmr.T @ Hmm_inv @ Hmr
        gp = g[8:] - Hmr.T @ Hmm_inv @ g[:8]
        Hp = 0.5 * (Hp + Hp.T)
        w, V = np.linalg.eigh(Hp)
        keep = w > 1e-8 * max(w.max(), 1.0)
        S = np.sqrt(w[keep])[:, None] * V[:, keep].T
        r0 = (V[:, keep].T @ gp) / np.sqrt(w[keep])
        x0 = np.concatenate([self.states[k] for k in nb])
        self.factors.append(LinearPrior(nb, x0, S, r0))
        del self.states[m]


def run_window(use_map, label):
    est = WindowEstimator(use_map)
    t0 = time.time()
    est.initialize()
    out = [est.states[0].copy()]
    for j in range(1, NF):
        out.append(est.add_frame(j))
        if j % 50 == 0:
            e = np.linalg.norm(out[-1][:2] - gt[j, :2])
            print(f"  [{label}] フレーム {j:4d}/{NF}  位置誤差 {e:6.3f} m  ({time.time()-t0:5.1f} s)")
    out = np.array(out)
    print(f"  推定したバイアス (最終): b_a = {out[-1][5:7]}, b_g = {out[-1][7]:.4f}"
          f"   (真値: {BIAS_A_TRUE}, {BIAS_G_TRUE})")
    return out, [0] + est.window_start


# ===========================================================================
# 手法 A: 地図マッチングのみ
# ===========================================================================
def run_map_only():
    out = [gt[0].copy(), gt[1].copy()]  # 最初の 2 フレームは既知とする
    for j in range(2, NF):
        a, b = out[-2], out[-1]
        # 等速モデルで予測: 直前の相対移動をもう一度繰り返す
        rel = np.array([*(rot2(a[2]).T @ (b[:2] - a[:2])), wrap_angle(b[2] - a[2])])
        pred = np.array([*(b[:2] + rot2(b[2]) @ rel[:2]), wrap_angle(b[2] + rel[2])])
        x, n = align_vgicp(scans[j], scan_covs[j], map_vms, pred, iters=10)
        out.append(x if n >= 30 else pred)  # 対応点が少ないときは予測をそのまま使う
        out[-1][2] = wrap_angle(out[-1][2])
    return np.array(out)


# ===========================================================================
# 実行
# ===========================================================================
print("\n[A] 地図マッチングのみ")
est_map_only = run_map_only()
print("\n[B] LiDAR-IMU のみ (地図なし)")
est_lio, _ = run_window(False, "LiDAR-IMU")
print("\n[C] 提案手法 (スキャン間 + 地図 + IMU のウィンドウ最適化)")
est_prop, window_start = run_window(True, "提案手法")

methods = {
    "A. 地図マッチングのみ": (est_map_only[:, :3], C_ORANGE),
    "B. LiDAR-IMU のみ": (est_lio[:, :3], C_AQUA),
    "C. 提案手法": (est_prop[:, :3], C_BLUE),
}
errors = {k: np.linalg.norm(v[0][:, :2] - gt[:, :2], axis=1) for k, v in methods.items()}


def rmse(e, sel):
    return float(np.sqrt(np.mean(e[sel] ** 2)))


# 地図の無い区間を抜けたあと (再び地図に入ってから) を区別する
first_exit = int(np.argmax(~in_map_region))
reenter = first_exit + int(np.argmax(in_map_region[first_exit:]))
sel_before = np.arange(NF) < first_exit
sel_nomap = ~in_map_region
sel_after = np.arange(NF) >= reenter
print("\n位置誤差 RMSE [m]")
print(f"  {'手法':16s} {'地図あり(前半)':>14s} {'地図なし区間':>12s} {'地図に戻った後':>14s}")
for k in methods:
    e = errors[k]
    print(f"  {k:16s} {rmse(e, sel_before):14.3f} {rmse(e, sel_nomap):12.3f} {rmse(e, sel_after):14.3f}")

# ===========================================================================
# 図
# ===========================================================================
plt = setup_matplotlib()
fig = plt.figure(figsize=(12, 9.5))
gs = fig.add_gridspec(2, 1, height_ratios=[1.35, 1])
ax = fig.add_subplot(gs[0])
ax.axvspan(MAP_X_MAX, 21, color=C_SHADE, alpha=0.6, lw=0)
ax.text(MAP_X_MAX + 0.4, 12.6, "地図点群が無い領域", color=C_INK, fontsize=10)
for a, b in segs:
    ax.plot([a[0], b[0]], [a[1], b[1]], color="#c9ced8", lw=0.8, zorder=0)
ax.scatter(map_pts[::3, 0], map_pts[::3, 1], s=1.5, color=C_MUTED, label="地図点群", zorder=1)
ax.plot(gt[:, 0], gt[:, 1], "--", color=C_INK, lw=1.6, label="真値", zorder=5)
for k, (v, c) in methods.items():
    ax.plot(v[:, 0], v[:, 1], color=c, lw=1.8, label=k, zorder=4)
ax.scatter(*gt[0, :2], marker="o", s=60, color=C_INK, zorder=6)
ax.annotate("スタート", gt[0, :2], textcoords="offset points", xytext=(8, -14), color=C_INK)
ax.set_xlim(-21, 21)
ax.set_ylim(-13, 13.5)
ax.set_aspect("equal")
ax.set_title("推定した軌跡 (2D 版の再現実験)", color=C_INK)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.06), ncol=5, fontsize=9, markerscale=4)

ax = fig.add_subplot(gs[1])
nomap = ~in_map_region
edges = np.flatnonzero(np.diff(np.concatenate([[0], nomap.astype(int), [0]])))
for s0, s1 in zip(edges[::2], edges[1::2]):
    ax.axvspan(frame_t[s0], frame_t[s1 - 1], color=C_SHADE, alpha=0.6, lw=0)
for k, (v, c) in methods.items():
    ax.plot(frame_t, errors[k], color=c, label=k)
ax.set_yscale("log")
ax.set_ylim(5e-3, 50)
ax.set_xlabel("時間 [s]  (色付きの帯 = 地図が無い区間)")
ax.set_ylabel("位置誤差 [m] (対数)")
ax.set_title("位置誤差の時間変化", color=C_INK)
ax.legend(ncol=3, fontsize=9)
fig.tight_layout()
out = fig_path("ex07_sliding_window_localization.png")
fig.savefig(out)
print("\n図を保存しました:", out)

# ===========================================================================
# HTML 版デモ用のデータ
# ===========================================================================
def r3(a):
    return np.round(np.asarray(a, dtype=float), 3).tolist()


sub = []
for k in range(NF):
    pts = scans[k]
    idx = np.linspace(0, len(pts) - 1, min(60, len(pts))).astype(int)
    sub.append(r3(pts[idx]))
data = {
    "segments": r3(segs),
    "map_points": r3(map_pts[::4]),
    "map_x_max": MAP_X_MAX,
    "t": r3(frame_t),
    "truth": r3(gt),
    "methods": {
        "map_only": r3(est_map_only[:, :3]),
        "lio_only": r3(est_lio[:, :3]),
        "proposed": r3(est_prop[:, :3]),
    },
    "window_start": [int(w) for w in window_start],
    "scans": sub,
    "rmse": {
        name: {"before": rmse(errors[k], sel_before), "nomap": rmse(errors[k], sel_nomap), "after": rmse(errors[k], sel_after)}
        for name, k in zip(["map_only", "lio_only", "proposed"], methods)
    },
    "params": {"lidar_hz": LIDAR_HZ, "imu_hz": IMU_HZ, "lag": SMOOTHER_LAG, "full_connection": FULL_CONNECTION},
}
DATA_DIR.mkdir(parents=True, exist_ok=True)
js = DATA_DIR / "sliding_window_result.js"
js.write_text("// ex07_sliding_window_localization.py が生成したデータ\nwindow.SW_DATA = " + json.dumps(data, separators=(",", ":")) + ";\n")
print("HTML デモ用データを保存しました:", js, f"({js.stat().st_size/1024:.0f} KB)")

print(
    """
やってみよう
  - FULL_CONNECTION = 1 にすると (直前フレームだけとマッチング) 地図なし区間の誤差は?
  - SMOOTHER_LAG を 1 秒 / 5 秒にすると?
  - MAP_X_MAX を -5 や 10 にして、地図の欠け方を変える
  - BIAS_G_TRUE を 0.05 に増やしても提案手法はバイアスを推定して追従できる?
"""
)
