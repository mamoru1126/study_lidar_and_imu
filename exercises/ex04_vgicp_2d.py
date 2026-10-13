"""演習 4: VGICP (ボクセル化 GICP) とマルチ解像度ボクセル地図

対応する資料: docs/03_point_cloud_registration.md (後半)

論文のスキャンマッチング因子は VGICP [Koide+ ICRA2021] を使っています。
GICP との違いは「対応の探し方」です。
  - GICP : ソースの各点に対して、ターゲットの最近傍点を kd-tree で探す
  - VGICP: ターゲットをボクセルに分けて各ボクセルを正規分布 (平均・共分散) で要約し、
           ソースの点が入るボクセルを 1 回のハッシュ参照で引く
           → 分岐が少なく、GPU で大量の点を並列処理しやすい

この演習で確かめること
  1. ボクセル地図は「壁に沿った細長い楕円」の集まりになる
  2. 対応探索はボクセル参照の方が kd-tree より速い (Python でも差が出る)
  3. ボクセルは 1 個しか見ないので収束域が狭い → 粗いボクセルを併用すると広がる

実行: python exercises/ex04_vgicp_2d.py
"""

import time

import numpy as np
from scipy.spatial import cKDTree

from common import C_AQUA, C_BLUE, C_INK, C_MUTED, C_ORANGE, fig_path, rot2, setup_matplotlib, wrap_angle
from toy2d import (
    GaussianVoxelMap,
    align_vgicp,
    estimate_covariances,
    make_environment,
    raycast,
    sample_map_points,
    voxel_downsample,
)

rng = np.random.default_rng(2)
segs = make_environment()

# 事前地図 (世界座標) と、その中で観測した 1 スキャン
map_pts = sample_map_points(segs, rng=rng)
map_covs = estimate_covariances(map_pts, k=10)
true_pose = np.array([-15.0, 3.0, -0.6])
scan = voxel_downsample(raycast(true_pose, segs, rng=rng), 0.2)
scan_covs = estimate_covariances(scan)
print(f"地図の点数 {len(map_pts)}, スキャンの点数 {len(scan)}")

# ---------------------------------------------------------------------------
print("\n[1] ボクセル地図を作る")
for res in [0.5, 1.0, 2.0]:
    vm = GaussianVoxelMap(map_pts, map_covs, res)
    print(f"  解像度 {res:3.1f} m: ボクセル数 {len(vm):5d}  (元の点数 {len(map_pts)})")

# ---------------------------------------------------------------------------
print("\n[2] 対応探索の速さ: kd-tree の最近傍探索 vs ボクセル参照")
query = rng.uniform([-20, -12], [20, 12], size=(200000, 2))
tree = cKDTree(map_pts)
vm = GaussianVoxelMap(map_pts, map_covs, 0.5)
t0 = time.perf_counter()
tree.query(query)
t1 = time.perf_counter()
vm.lookup(query)
t2 = time.perf_counter()
print(f"  20 万点の対応探索: kd-tree {1000*(t1-t0):6.1f} ms, ボクセル参照 {1000*(t2-t1):6.1f} ms")
print("  (C++/GPU ではボクセル参照はハッシュ表 1 回の参照で済み、差はさらに大きくなります)")

# ---------------------------------------------------------------------------
print("\n[3] マルチ解像度の効果: 初期値のずれ vs 成功率 (各 30 回)")
configs = {
    "0.5 m のみ": (0.5,),
    "0.5 + 1.0 m": (0.5, 1.0),
    "0.5 + 1.0 + 2.0 m": (0.5, 1.0, 2.0),
}
voxelmaps = {k: [GaussianVoxelMap(map_pts, map_covs, r) for r in v] for k, v in configs.items()}
levels = [0.25, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
success = {k: [] for k in configs}
for lv in levels:
    for name, vms in voxelmaps.items():
        ok = 0
        for _ in range(30):
            a = rng.uniform(0, 2 * np.pi)
            init = true_pose + np.array([lv * np.cos(a), lv * np.sin(a), rng.uniform(-1, 1) * 0.1 * lv])
            est, _ = align_vgicp(scan, scan_covs, vms, init, iters=30)
            ok += np.linalg.norm(est[:2] - true_pose[:2]) < 0.05 and abs(wrap_angle(est[2] - true_pose[2])) < np.radians(1)
        success[name].append(ok / 30)
    print(f"  ずれ {lv:3.1f} m: " + ", ".join(f"{k} {success[k][-1]*100:3.0f}%" for k in configs))

# ---------------------------------------------------------------------------
plt = setup_matplotlib()
fig, axes = plt.subplots(1, 2, figsize=(11.5, 5.0), gridspec_kw={"width_ratios": [1.25, 1]})

ax = axes[0]
vm = GaussianVoxelMap(map_pts, map_covs, 1.0)
t = np.linspace(0, 2 * np.pi, 24)
circle = np.stack([np.cos(t), np.sin(t)])
for mu, C in zip(vm.mean, vm.cov):
    if not (-20 < mu[0] < -6 and -4 < mu[1] < 12):
        continue
    w, V = np.linalg.eigh(C)
    ell = (V @ np.diag(0.45 * np.sqrt(np.maximum(w, 0.02))) @ circle).T + mu
    ax.fill(ell[:, 0], ell[:, 1], color=C_AQUA, alpha=0.45, lw=0, zorder=3)
    ax.plot(ell[:, 0], ell[:, 1], color=C_AQUA, lw=0.9, zorder=3)
est, _ = align_vgicp(scan, scan_covs, voxelmaps["0.5 + 1.0 m"], true_pose + np.array([0.4, -0.3, 0.05]))
p = scan @ rot2(est[2]).T + est[:2]
ax.scatter(p[:, 0], p[:, 1], s=3, color=C_BLUE, label="位置合わせしたスキャン", zorder=2)
ax.scatter(*est[:2], marker="^", s=80, color=C_INK, label="推定したロボット位置", zorder=4)
ax.fill([], [], color=C_AQUA, alpha=0.45, label="ボクセルの正規分布 (見やすさのため太らせて表示)")
for x in np.arange(-20, -5.9, 1.0):
    ax.axvline(x, color="#d5dae2", lw=0.5, zorder=0)
for y in np.arange(-4, 12.1, 1.0):
    ax.axhline(y, color="#d5dae2", lw=0.5, zorder=0)
ax.set_xlim(-20.5, -6)
ax.set_ylim(-4.5, 12.5)
ax.set_aspect("equal")
ax.grid(False)
ax.set_title("1.0 m ボクセル地図 (緑の楕円 = 各ボクセルの分布)", color=C_INK)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), fontsize=9, ncol=1)

ax = axes[1]
cols = [C_ORANGE, C_AQUA, C_BLUE]
for c, name in zip(cols, configs):
    ax.plot(levels, np.array(success[name]) * 100, "o-", color=c, label=name)
ax.set_xlabel("初期値のずれ [m]")
ax.set_ylabel("成功率 [%]")
ax.set_ylim(-5, 105)
ax.set_title("粗いボクセルを足すと収束域が広がる", color=C_INK)
ax.legend()
fig.tight_layout()
out = fig_path("ex04_vgicp_2d.png")
fig.savefig(out)
print("\n図を保存しました:", out)

print(
    """
やってみよう
  - GaussianVoxelMap の min_points を 3 にして、点の少ないボクセルを捨てるとどうなる?
  - 細かいボクセル (0.25 m) を足すと最終精度は上がる? 収束域は?
  - true_pose を長い廊下の真ん中 (例: (0, -8, 0)) にすると、前後方向の誤差が残りやすいことを確認する
"""
)
