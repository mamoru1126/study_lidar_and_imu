"""演習 3: スキャンマッチング (ICP → 点対面 ICP → GICP)

対応する資料: docs/03_point_cloud_registration.md

2 つの姿勢から観測した 2D LiDAR スキャンを位置合わせします。
  - 点対点 ICP   : 最近傍点との距離 |q - T p|^2 を最小化 (SVD で閉形式に解ける)
  - 点対線 ICP   : 最近傍点の「線 (3D なら面)」までの距離 (n^T (q - T p))^2 を最小化
  - GICP         : 点を正規分布とみなし、分布同士のマハラノビス距離を最小化
                   d^T (C_q + R C_p R^T)^{-1} d   (論文が使う VGICP の基礎)

この演習で確かめること
  1. GICP / 点対線 ICP は点対点 ICP より少ない反復で収束する
  2. 初期値のずれが大きいと、どの手法も失敗しやすくなる (収束域がある)

実行: python exercises/ex03_icp_2d.py
"""

import numpy as np
from scipy.spatial import cKDTree

from common import C_AQUA, C_BLUE, C_INK, C_MUTED, C_ORANGE, fig_path, rot2, drot2, setup_matplotlib, wrap_angle
from toy2d import estimate_covariances, make_environment, raycast, voxel_downsample

rng = np.random.default_rng(1)
segs = make_environment()

# 2 つの姿勢 (世界座標)。ターゲット = A、ソース = B
pose_A = np.array([-14.0, -8.0, 0.20])
pose_B = np.array([-13.2, -7.6, 0.35])
tgt = voxel_downsample(raycast(pose_A, segs, rng=rng), 0.15)
src = voxel_downsample(raycast(pose_B, segs, rng=rng), 0.15)
tgt_covs = estimate_covariances(tgt)
src_covs = estimate_covariances(src)
tgt_normals = np.linalg.eigh(tgt_covs)[1][:, :, 0]  # 最小固有値の固有ベクトル = 法線
tree = cKDTree(tgt)

# 正解の相対姿勢 (ソース座標 → ターゲット座標)
R_A = rot2(pose_A[2])
gt = np.array([*(R_A.T @ (pose_B[:2] - pose_A[:2])), pose_B[2] - pose_A[2]])
print("正解の相対姿勢 (x, y, θ) =", gt)

MAX_DIST = 1.5  # これより遠い最近傍は対応とみなさない


def pose_error(x):
    return np.linalg.norm(x[:2] - gt[:2]), abs(wrap_angle(x[2] - gt[2]))


def icp_point_to_point(x0, iters=60):
    x = x0.copy()
    hist = [pose_error(x)]
    for _ in range(iters):
        p = src @ rot2(x[2]).T + x[:2]
        dist, idx = tree.query(p)
        m = dist < MAX_DIST
        if m.sum() < 3:  # 対応点がほとんど無い → 失敗
            break
        P, Q = p[m], tgt[idx[m]]
        # 重心を合わせて SVD で最適な回転を求める (Kabsch / Umeyama 法)
        mp, mq = P.mean(0), Q.mean(0)
        U, _, Vt = np.linalg.svd((P - mp).T @ (Q - mq))
        dR = Vt.T @ U.T
        if np.linalg.det(dR) < 0:
            Vt[-1] *= -1
            dR = Vt.T @ U.T
        dt = mq - dR @ mp
        # 更新: 新しい変換 = dR * (現在の変換)
        R_new = dR @ rot2(x[2])
        x = np.array([*(dR @ x[:2] + dt), np.arctan2(R_new[1, 0], R_new[0, 0])])
        hist.append(pose_error(x))
    return x, hist


def gauss_newton_icp(x0, kind, iters=40):
    """kind = 'plane' (点対線) または 'gicp'"""
    x = x0.copy()
    hist = [pose_error(x)]
    for _ in range(iters):
        R, dR = rot2(x[2]), drot2(x[2])
        p = src @ R.T + x[:2]
        dist, idx = tree.query(p)
        m = dist < MAX_DIST
        if m.sum() < 3:
            break
        q, pm = tgt[idx[m]], src[m]
        d = q - p[m]  # 誤差ベクトル
        dp = np.zeros((len(pm), 2, 3))  # ∂(T p)/∂(x, y, θ)
        dp[:, :, 0:2] = np.eye(2)
        dp[:, :, 2] = pm @ dR.T
        if kind == "plane":
            n = tgt_normals[idx[m]]
            e = np.einsum("ni,ni->n", n, d)[:, None]
            J = -np.einsum("ni,nij->nj", n, dp)[:, None, :]
        else:  # GICP
            Om = np.linalg.inv(tgt_covs[idx[m]] + R @ src_covs[m] @ R.T)
            L = np.linalg.cholesky(Om)
            e = np.einsum("nji,nj->ni", L, d)
            J = -np.einsum("nji,njk->nik", L, dp)
        H = np.einsum("nki,nkj->ij", J, J)
        g = np.einsum("nki,nk->i", J, e)
        x = x + np.linalg.solve(H + 1e-9 * np.eye(3), -g)
        hist.append(pose_error(x))
    return x, hist


methods = {
    "点対点 ICP": lambda x0: icp_point_to_point(x0),
    "点対線 ICP": lambda x0: gauss_newton_icp(x0, "plane"),
    "GICP": lambda x0: gauss_newton_icp(x0, "gicp"),
}
colors = {"点対点 ICP": C_ORANGE, "点対線 ICP": C_AQUA, "GICP": C_BLUE}

# ---------------------------------------------------------------------------
print("\n[1] 初期値 = 単位行列 (動いていないと仮定) からの位置合わせ")
x0 = np.zeros(3)
results = {}
for name, f in methods.items():
    x, hist = f(x0)
    results[name] = (x, hist)
    t3, r3 = hist[3]
    print(f"  {name:8s}: 3 回反復後の誤差 {t3*100:6.2f} cm / {np.degrees(r3):5.2f} deg,"
          f"  最終誤差 {hist[-1][0]*100:5.2f} cm / {np.degrees(hist[-1][1]):5.2f} deg")

# ---------------------------------------------------------------------------
print("\n[2] 初期値のずれと成功率 (各 40 回試行, 成功 = 10cm かつ 1deg 以内)")
levels = [0.5, 1.0, 2.0, 3.0, 4.0, 5.0]
success = {name: [] for name in methods}
for lv in levels:
    for name, f in methods.items():
        ok = 0
        for _ in range(40):
            ang = rng.uniform(0, 2 * np.pi)
            init = gt + np.array([lv * np.cos(ang), lv * np.sin(ang), rng.uniform(-1, 1) * 0.2 * lv])
            x, hist = f(init)
            t, r = hist[-1]
            ok += t < 0.10 and r < np.radians(1.0)
        success[name].append(ok / 40)
    print(f"  ずれ {lv:4.2f} m : " + ", ".join(f"{n} {success[n][-1]*100:3.0f}%" for n in methods))

# ---------------------------------------------------------------------------
plt = setup_matplotlib()
fig = plt.figure(figsize=(11, 8.2))
gs = fig.add_gridspec(2, 2, height_ratios=[1.1, 1])


def draw(ax, x, title):
    p = src @ rot2(x[2]).T + x[:2]
    ax.scatter(tgt[:, 0], tgt[:, 1], s=4, color=C_MUTED, label="ターゲット (フレーム A)")
    ax.scatter(p[:, 0], p[:, 1], s=4, color=C_BLUE, label="ソース (フレーム B)")
    ax.set_aspect("equal")
    ax.set_title(title, color=C_INK)
    ax.set_xlim(-7, 15)
    ax.set_ylim(-6, 11)
    ax.legend(loc="upper right", fontsize=8, markerscale=3)


draw(fig.add_subplot(gs[0, 0]), x0, "位置合わせ前 (初期値 = 動いていない)")
draw(fig.add_subplot(gs[0, 1]), results["GICP"][0], "GICP で位置合わせ後")

ax = fig.add_subplot(gs[1, 0])
for name, (_, hist) in results.items():
    ax.semilogy([h[0] * 100 + 1e-3 for h in hist], color=colors[name], label=name)
ax.set_xlabel("反復回数")
ax.set_ylabel("並進誤差 [cm] (対数)")
ax.set_title("収束の速さ", color=C_INK)
ax.legend()

ax = fig.add_subplot(gs[1, 1])
for name in methods:
    ax.plot(levels, np.array(success[name]) * 100, "o-", color=colors[name], label=name)
ax.set_xlabel("初期値のずれ [m]")
ax.set_ylabel("成功率 [%]")
ax.set_ylim(-5, 105)
ax.set_title("初期値への頑健さ (収束域)", color=C_INK)
ax.legend()
fig.tight_layout()
out = fig_path("ex03_icp_2d.png")
fig.savefig(out)
print("\n図を保存しました:", out)

print(
    """
やってみよう
  - MAX_DIST を 0.5 や 3.0 にすると、収束の速さと成功率はどう変わる?
  - estimate_covariances の eps を 1 に近づける (= 点を丸い分布とみなす) と GICP は ICP に近づく
  - pose_B を pose_A から遠ざけて (重なりを減らして) 失敗の様子を見る
"""
)
