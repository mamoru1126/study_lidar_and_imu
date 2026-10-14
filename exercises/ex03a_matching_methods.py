"""演習 3a: スキャンマッチングの各手法は「地図 (ターゲット) を何で表し、何と何を比べるか」

対応する資料: docs/03_point_cloud_registration.md

同じ 2D スキャンのペアを使って、各手法の考え方を図にします。
  1. 前処理: ダウンサンプリング、近傍点、点ごとの共分散 (面の向き)
  2. 点対点 ICP: 最も近い「点」との距離
  3. 点対面 ICP: 最も近い点の「面 (2D では線)」までの距離
  4. NDT: ターゲットをボクセルに区切り、ボクセル内の点の位置から正規分布を作る。
          ソースの「点」がその分布の中でどれくらいありそうか (スコア) を見る
  5. GICP: ターゲットの点もソースの点も「点ごとの正規分布」。分布と分布を比べる
  6. NDT と VGICP: 同じボクセルでも、分布の作り方が違う
  7. コストの形: 正解からずらしたとき、各手法のコストがどう変わるか

実行: python exercises/ex03a_matching_methods.py
"""

import numpy as np
from scipy.spatial import cKDTree

from common import C_AQUA, C_BLUE, C_INK, C_MUTED, C_ORANGE, fig_path, rot2, setup_matplotlib
from toy2d import GaussianVoxelMap, estimate_covariances, make_environment, raycast, voxel_downsample

np.set_printoptions(precision=3, suppress=True)
rng = np.random.default_rng(3)
plt = setup_matplotlib()
from matplotlib.patches import Ellipse, Rectangle  # noqa: E402

segs = make_environment()
pose_A = np.array([-14.0, -8.0, 0.20])  # ターゲット (地図側) のスキャン位置
pose_B = np.array([-13.2, -7.6, 0.35])  # ソース (動かす側) のスキャン位置
raw_tgt = raycast(pose_A, segs, n_beams=720, rng=rng)
tgt = voxel_downsample(raw_tgt, 0.15)
src = voxel_downsample(raycast(pose_B, segs, rng=rng), 0.15)
tgt_cov = estimate_covariances(tgt)
src_cov = estimate_covariances(src)
tgt_nrm = np.linalg.eigh(tgt_cov)[1][:, :, 0]
tgt_nrm[np.einsum("ni,ni->n", tgt_nrm, -tgt) < 0] *= -1  # 法線はセンサ (原点) の方を向くようにそろえる
tree = cKDTree(tgt)

R_A = rot2(pose_A[2])
gt = np.array([*(R_A.T @ (pose_B[:2] - pose_A[:2])), pose_B[2] - pose_A[2]])  # 正解の相対姿勢
guess = gt + np.array([0.30, -0.25, 0.04])  # 少しずれた初期値 (図用)


def apply(x, pts):
    return pts @ rot2(x[2]).T + x[:2]


src_g = apply(guess, src)
src_cov_g = rot2(guess[2]) @ src_cov @ rot2(guess[2]).T
ZOOM = (-6.0, 1.8, -4.8, 0.4)  # NDT / VGICP の図で拡大する範囲 (壁の角と柱が入る)
TIGHT = (-3.4, 0.6, -4.6, -2.5)  # ICP / GICP の図で拡大する範囲 (角の付近)


def in_zoom(p, m=0.0, Z=None):
    Z = Z or ZOOM
    return (p[:, 0] > Z[0] - m) & (p[:, 0] < Z[1] + m) & (p[:, 1] > Z[2] - m) & (p[:, 1] < Z[3] + m)


def visible(C, ratio=0.04):
    """表示用: とても細い楕円は線に見えるので、短い軸を少し太らせる (計算には使わない)"""
    w, V = np.linalg.eigh(C)
    w = np.maximum(w, ratio * w.max())
    return V @ np.diag(w) @ V.T


def ellipse(ax, mu, C, scale, **kw):
    w, V = np.linalg.eigh(C)
    w = np.maximum(w, 1e-12)
    ang = np.degrees(np.arctan2(V[1, 1], V[0, 1]))
    ax.add_patch(Ellipse(mu, 2 * scale * np.sqrt(w[1]), 2 * scale * np.sqrt(w[0]), angle=ang, **kw))


def zoom_ax(ax, title, Z=None):
    Z = Z or ZOOM
    ax.set_xlim(Z[0], Z[1])
    ax.set_ylim(Z[2], Z[3])
    ax.set_aspect("equal")
    ax.set_title(title, color=C_INK)
    ax.set_xlabel("x [m]")


def legend_below(ax, ncol=1):
    ax.legend(fontsize=9, loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=ncol)


def section(title):
    print("\n" + "=" * 70 + f"\n{title}\n" + "=" * 70)


# ===========================================================================
section("1. 前処理")
print(f"生のスキャン {len(raw_tgt)} 点 → 0.15 m ボクセルでダウンサンプリング → {len(tgt)} 点")
fig, axes = plt.subplots(1, 3, figsize=(15, 5.6))
ax = axes[0]
ax.scatter(raw_tgt[:, 0], raw_tgt[:, 1], s=2, color=C_MUTED)
ax.add_patch(Rectangle((TIGHT[0], TIGHT[2]), TIGHT[1] - TIGHT[0], TIGHT[3] - TIGHT[2], fill=False, ec=C_ORANGE, lw=1.5))
ax.scatter([0], [0], marker="^", s=80, color=C_INK, zorder=5)
ax.set_aspect("equal")
ax.set_title(f"(a) 1 回分のスキャン ({len(raw_tgt)} 点)", color=C_INK)
ax.set_xlabel("x [m] (センサ座標)")
ax.text(TIGHT[0], TIGHT[3] + 0.4, "拡大する範囲", color=C_ORANGE, fontsize=9)
ax = axes[1]
m = in_zoom(raw_tgt, Z=TIGHT)
ax.scatter(raw_tgt[m, 0], raw_tgt[m, 1], s=5, color=C_MUTED, label=f"生の点 ({m.sum()} 点)")
m = in_zoom(tgt, Z=TIGHT)
ax.scatter(tgt[m, 0], tgt[m, 1], s=46, facecolor="none", edgecolor=C_BLUE, lw=1.3, label=f"ダウンサンプリング後 ({m.sum()} 点)")
zoom_ax(ax, "(b) ダウンサンプリング: 密度をそろえて点を減らす", TIGHT)
legend_below(ax)
ax = axes[2]
m = np.flatnonzero(in_zoom(tgt, Z=TIGHT))
ax.scatter(tgt[m, 0], tgt[m, 1], s=8, color=C_MUTED, label="点", zorder=3)
k_show = m[np.argmin(np.linalg.norm(tgt[m] - np.array([-1.9, -3.7]), axis=1))]
_, nb = tree.query(tgt[k_show], k=8)
ax.scatter(tgt[nb, 0], tgt[nb, 1], s=60, facecolor="none", edgecolor=C_ORANGE, lw=1.6, label="ある点の近傍 8 点", zorder=4)
for i in m:
    ellipse(ax, tgt[i], visible(tgt_cov[i]), 0.07, fc=C_BLUE, alpha=0.25, ec=C_BLUE, lw=0.8)
    ax.annotate("", tgt[i] + 0.22 * tgt_nrm[i], tgt[i], arrowprops=dict(arrowstyle="->", color=C_AQUA, lw=0.9))
ax.plot([], [], color=C_BLUE, label="点ごとの共分散 (面に沿って細長い。太さは誇張)")
ax.plot([], [], color=C_AQUA, label="法線 (共分散の一番短い軸。センサ側に向ける)")
zoom_ax(ax, "(c) 近傍から求めた点ごとの共分散と法線", TIGHT)
legend_below(ax)
fig.tight_layout()
fig.savefig(fig_path("ex03a_1_preprocess.png"))
plt.close(fig)

# ===========================================================================
section("2. 点対点 ICP と 3. 点対面 ICP")
d, idx = tree.query(src_g)
mz = np.flatnonzero(in_zoom(src_g, Z=TIGHT) & (d < 1.5))
fig, axes = plt.subplots(1, 2, figsize=(13, 5.0))
for ax, kind in zip(axes, ["p2p", "p2l"]):
    mt = in_zoom(tgt, 0.5, TIGHT)
    ax.scatter(tgt[mt, 0], tgt[mt, 1], s=22, color=C_MUTED, label="ターゲット (地図側) の点", zorder=2)
    ax.scatter(src_g[mz, 0], src_g[mz, 1], s=22, color=C_BLUE, label="ソース (動かす側) の点", zorder=3)
    for i in mz:
        p, q, n = src_g[i], tgt[idx[i]], tgt_nrm[idx[i]]
        if kind == "p2p":
            ax.annotate("", q, p, arrowprops=dict(arrowstyle="->", color=C_ORANGE, lw=1.0))
        else:
            foot = p + n * np.dot(n, q - p)  # q を通る線 (面) への垂線の足
            ax.plot([q[0] - 0.2 * n[1], q[0] + 0.2 * n[1]], [q[1] + 0.2 * n[0], q[1] - 0.2 * n[0]], color=C_AQUA, lw=1.2)
            ax.annotate("", foot, p, arrowprops=dict(arrowstyle="->", color=C_ORANGE, lw=1.0))
    if kind == "p2p":
        ax.plot([], [], color=C_ORANGE, label="誤差 = 最も近い点への矢印")
        zoom_ax(ax, "点対点 ICP: 最も近い「点」へ引っ張る", TIGHT)
    else:
        ax.plot([], [], color=C_AQUA, label="最も近い点を通る線 (面)")
        ax.plot([], [], color=C_ORANGE, label="誤差 = 線への垂直な距離")
        zoom_ax(ax, "点対面 ICP: 最も近い点の「面」へ引っ張る", TIGHT)
    legend_below(ax)
    ax.set_ylabel("y [m]")
fig.tight_layout()
fig.savefig(fig_path("ex03a_2_icp.png"))
plt.close(fig)
p2p_err = np.linalg.norm(tgt[idx[mz]] - src_g[mz], axis=1)
p2l_err = np.abs(np.einsum("ni,ni->n", tgt_nrm[idx[mz]], tgt[idx[mz]] - src_g[mz]))
print(f"拡大範囲の対応 {len(mz)} 組: 点対点の誤差の平均 {p2p_err.mean():.3f} m, 点対面の誤差の平均 {p2l_err.mean():.3f} m")
print("→ 点対面は「壁に沿った方向のずれ」を数えないので、誤差が小さく、向きもそろう")

# ===========================================================================
section("4. NDT: ボクセル内の点の位置から分布を作る")
VOX = 1.0
MIN_PTS = 3  # 2D なので 3 点。PCL の 3D 実装では既定 6 点


def ndt_voxels(pts, res, min_pts):
    keys = np.floor(pts / res).astype(int)
    out = {}
    for k in np.unique(keys, axis=0):
        sel = pts[(keys == k).all(axis=1)]
        if len(sel) < min_pts:
            out[tuple(k)] = (len(sel), None, None)
            continue
        mu = sel.mean(axis=0)
        C = np.cov(sel.T, bias=True)
        w, V = np.linalg.eigh(C)
        w = np.maximum(w, 0.01 * w.max())  # 平べったすぎる分布の補正 (PCL と同じ考え方)
        out[tuple(k)] = (len(sel), mu, V @ np.diag(w) @ V.T)
    return out


ndt = ndt_voxels(tgt, VOX, MIN_PTS)
n_ok = sum(v[1] is not None for v in ndt.values())
print(f"ボクセル {VOX} m: 点の入ったボクセル {len(ndt)} 個のうち、分布を作れたもの {n_ok} 個 (残りは点が {MIN_PTS} 未満)")


def ndt_score(p):
    k = tuple(np.floor(p / VOX).astype(int))
    v = ndt.get(k)
    if v is None or v[1] is None:
        return None
    dd = p - v[1]
    return float(np.exp(-0.5 * dd @ np.linalg.solve(v[2], dd)))


fig, axes = plt.subplots(1, 2, figsize=(12, 5.9))
ax = axes[0]
mt = in_zoom(tgt, 0.5)
ax.scatter(tgt[mt, 0], tgt[mt, 1], s=10, color=C_MUTED, label="ターゲットの点", zorder=2)
for k, (n, mu, C) in ndt.items():
    x0, y0 = k[0] * VOX, k[1] * VOX
    if x0 > ZOOM[1] or x0 + VOX < ZOOM[0] or y0 > ZOOM[3] or y0 + VOX < ZOOM[2]:
        continue
    if mu is None:
        ax.add_patch(Rectangle((x0, y0), VOX, VOX, fc=C_ORANGE, alpha=0.12, ec="none"))
    else:
        ellipse(ax, mu, C, 2.0, fc=C_BLUE, alpha=0.18, ec=C_BLUE, lw=1.2)
ax.add_patch(Rectangle((0, 0), 0, 0, fc=C_BLUE, alpha=0.3, label="ボクセルの正規分布 (2σ)"))
ax.add_patch(Rectangle((0, 0), 0, 0, fc=C_ORANGE, alpha=0.3, label=f"点が {MIN_PTS} 未満で分布を作れないボクセル"))
for x in np.arange(np.floor(ZOOM[0]), ZOOM[1] + 1e-9, VOX):
    ax.axvline(x, color="#cfd5de", lw=0.8, zorder=1)
for y in np.arange(np.floor(ZOOM[2]), ZOOM[3] + 1e-9, VOX):
    ax.axhline(y, color="#cfd5de", lw=0.8, zorder=1)
ax.grid(False)
zoom_ax(ax, f"NDT の地図: {VOX:.0f} m ボクセルごとに 1 つの分布")
legend_below(ax)

ax = axes[1]
for k, (n, mu, C) in ndt.items():
    if mu is not None:
        ellipse(ax, mu, C, 2.0, fc=C_MUTED, alpha=0.15, ec=C_MUTED, lw=0.8)
sc_vals, sc_pts, none_pts = [], [], []
for p in src_g[in_zoom(src_g)]:
    s = ndt_score(p)
    if s is None:
        none_pts.append(p)
    else:
        sc_vals.append(s)
        sc_pts.append(p)
sc_pts = np.array(sc_pts)
h = ax.scatter(sc_pts[:, 0], sc_pts[:, 1], c=sc_vals, cmap="viridis", vmin=0, vmax=1, s=22, zorder=4)
if none_pts:
    none_pts = np.array(none_pts)
    ax.scatter(none_pts[:, 0], none_pts[:, 1], marker="x", s=22, color=C_ORANGE, zorder=4, label="分布のないボクセルに入った点 (スコア 0)")
cb = fig.colorbar(h, ax=ax, fraction=0.04, pad=0.02)
cb.set_label("スコア (1 に近いほど分布の中心に近い)")
for x in np.arange(np.floor(ZOOM[0]), ZOOM[1] + 1e-9, VOX):
    ax.axvline(x, color="#cfd5de", lw=0.8, zorder=1)
for y in np.arange(np.floor(ZOOM[2]), ZOOM[3] + 1e-9, VOX):
    ax.axhline(y, color="#cfd5de", lw=0.8, zorder=1)
ax.grid(False)
zoom_ax(ax, "ソースの「点」を、入ったボクセルの分布で採点")
legend_below(ax)
fig.tight_layout()
fig.savefig(fig_path("ex03a_3_ndt.png"))
plt.close(fig)

# ===========================================================================
section("5. GICP: 点ごとの分布どうしを比べる")
fig, axes = plt.subplots(1, 2, figsize=(13, 5.4), gridspec_kw={"width_ratios": [1.5, 1]})
ax = axes[0]
mt = np.flatnonzero(in_zoom(tgt, 0.3, TIGHT))
for i in mt:
    ellipse(ax, tgt[i], visible(tgt_cov[i]), 0.07, fc=C_MUTED, alpha=0.35, ec=C_MUTED, lw=0.8)
ms = np.flatnonzero(in_zoom(src_g, Z=TIGHT))
for i in ms:
    ellipse(ax, src_g[i], visible(src_cov_g[i]), 0.07, fc=C_BLUE, alpha=0.3, ec=C_BLUE, lw=0.8)
show = ms[:: max(1, len(ms) // 6)][:6]
for i in show:
    ax.annotate("", tgt[idx[i]], src_g[i], arrowprops=dict(arrowstyle="->", color=C_ORANGE, lw=1.4))
ax.add_patch(Rectangle((0, 0), 0, 0, fc=C_MUTED, alpha=0.5, label="ターゲットの点の分布"))
ax.add_patch(Rectangle((0, 0), 0, 0, fc=C_BLUE, alpha=0.5, label="ソースの点の分布"))
ax.plot([], [], color=C_ORANGE, label="対応 (kd-tree で探した最も近い点)")
zoom_ax(ax, "GICP: 点も分布、相手も分布 (楕円の太さは誇張)", TIGHT)
ax.set_ylabel("y [m]")
legend_below(ax)

ax = axes[1]
i = show[np.argmin(np.linalg.norm(tgt[idx[show]] - src_g[show], axis=1))]  # ずれの小さい 1 組
j = idx[i]
c = 0.5 * (tgt[j] + src_g[i])
half = 0.5 * np.linalg.norm(tgt[j] - src_g[i]) + 0.12 * np.sqrt(np.linalg.eigvalsh(tgt_cov[j] + src_cov_g[i]).max()) + 0.03
ellipse(ax, tgt[j], visible(tgt_cov[j], 0.02), 0.12, fc=C_MUTED, alpha=0.35, ec=C_MUTED, lw=1.2)
ellipse(ax, src_g[i], visible(src_cov_g[i], 0.02), 0.12, fc=C_BLUE, alpha=0.3, ec=C_BLUE, lw=1.2)
ellipse(ax, tgt[j], visible(tgt_cov[j] + src_cov_g[i], 0.02), 0.12, fill=False, ec=C_ORANGE, lw=1.8, ls="--")
ax.annotate("", tgt[j], src_g[i], arrowprops=dict(arrowstyle="->", color=C_INK, lw=1.6))
ax.text(*(0.5 * (tgt[j] + src_g[i]) + np.array([0.02, 0.0])), "d", fontsize=13, color=C_INK)
ax.scatter(*tgt[j], s=30, color=C_MUTED, zorder=5)
ax.scatter(*src_g[i], s=30, color=C_BLUE, zorder=5)
ax.plot([], [], color=C_MUTED, lw=6, alpha=0.5, label="ターゲットの点 q の分布 C_q")
ax.plot([], [], color=C_BLUE, lw=6, alpha=0.4, label="ソースの点 p の分布 R C_p Rᵀ".replace("ᵀ", "^T"))
ax.plot([], [], color=C_ORANGE, ls="--", label="2 つを足した分布 (誤差の重み)")
ax.set_xlim(c[0] - half, c[0] + half)
ax.set_ylim(c[1] - 0.75 * half, c[1] + 0.75 * half)
ax.set_aspect("equal")
ax.set_title("1 組だけ拡大: d を足した分布で測る", color=C_INK)
ax.set_xlabel("x [m]")
legend_below(ax)
fig.tight_layout()
fig.savefig(fig_path("ex03a_4_gicp.png"))
plt.close(fig)
dd = tgt[j] - src_g[i]
Cs = tgt_cov[j] + src_cov_g[i]
print(f"ある対応: ずれ d = {dd}, 長さ {np.linalg.norm(dd):.3f} m")
print(f"  GICP の誤差 dᵀ(C_q + R C_p Rᵀ)⁻¹d = {dd @ np.linalg.solve(Cs, dd):.2f}  (面に垂直な成分が大きく効く)")

# ===========================================================================
section("6. NDT と VGICP: 同じボクセルでも分布の作り方が違う")
vg = GaussianVoxelMap(tgt, tgt_cov, VOX)
fig, axes = plt.subplots(1, 2, figsize=(12, 5.9))
for ax, mode in zip(axes, ["ndt", "vgicp"]):
    mt = in_zoom(tgt, 0.5)
    ax.scatter(tgt[mt, 0], tgt[mt, 1], s=10, color=C_MUTED, zorder=2)
    for x in np.arange(np.floor(ZOOM[0]), ZOOM[1] + 1e-9, VOX):
        ax.axvline(x, color="#cfd5de", lw=0.8, zorder=1)
    for y in np.arange(np.floor(ZOOM[2]), ZOOM[3] + 1e-9, VOX):
        ax.axhline(y, color="#cfd5de", lw=0.8, zorder=1)
    ax.grid(False)
    if mode == "ndt":
        for k, (n, mu, C) in ndt.items():
            x0, y0 = k[0] * VOX, k[1] * VOX
            if mu is None:
                ax.add_patch(Rectangle((x0, y0), VOX, VOX, fc=C_ORANGE, alpha=0.12, ec="none"))
                continue
            w = np.linalg.eigvalsh(C)
            ellipse(ax, mu, C, 0.45 * VOX / np.sqrt(w[-1]), fc=C_BLUE, alpha=0.2, ec=C_BLUE, lw=1.2)
        zoom_ax(ax, "NDT: ボクセル内の「点の位置」から共分散を計算")
        ax.add_patch(Rectangle((0, 0), 0, 0, fc=C_ORANGE, alpha=0.3, label=f"点が {MIN_PTS} 未満 → 分布なし (地図に穴が開く)"))
        ax.add_patch(Rectangle((0, 0), 0, 0, fc=C_BLUE, alpha=0.3, label="分布ができたボクセル"))
    else:
        for mu, C in zip(vg.mean, vg.cov):
            w = np.linalg.eigvalsh(C)
            ellipse(ax, mu, C, 0.45 * VOX / np.sqrt(w[-1]), fc=C_AQUA, alpha=0.25, ec=C_AQUA, lw=1.2)
        zoom_ax(ax, "VGICP: ボクセル内の「点ごとの共分散」を平均")
        ax.add_patch(Rectangle((0, 0), 0, 0, fc=C_AQUA, alpha=0.35, label="点が 1 つでも分布ができる (穴が開かない)"))
    legend_below(ax)
fig.tight_layout()
fig.savefig(fig_path("ex03a_5_ndt_vs_vgicp.png"))
plt.close(fig)
print(f"NDT: 分布を作れたボクセル {n_ok} / {len(ndt)},  VGICP: {len(vg)} / {len(vg)} (全部)")
print("(図では形を比べるため、楕円の長い軸の長さをそろえて描いています)")

# ===========================================================================
section("7. コストの形: 正解から x 方向にずらしたとき")
offsets = np.linspace(-2.5, 2.5, 101)
DMAX = 1.5
vg_maps = [GaussianVoxelMap(tgt, tgt_cov, r) for r in (0.5, 1.0)]


def costs(x):
    p = apply(x, src)
    Rm = rot2(x[2])
    Cp = Rm @ src_cov @ Rm.T
    dist, j = tree.query(p)
    ok = dist < DMAX
    dvec = tgt[j] - p
    c_p2p = np.sum(np.where(ok, dist**2, DMAX**2))
    pl = np.einsum("ni,ni->n", tgt_nrm[j], dvec) ** 2
    c_p2l = np.sum(np.where(ok, pl, DMAX**2))
    S = tgt_cov[j] + Cp
    g = np.einsum("ni,nij,nj->n", dvec, np.linalg.inv(S), dvec)
    c_gicp = np.sum(np.where(ok, np.minimum(g, 1e3), 1e3))
    sc = [ndt_score(q) for q in p]
    c_ndt = -np.sum([s for s in sc if s is not None])
    c_vg = 0.0
    for vm in vg_maps:
        vid = vm.lookup(p)
        hit = vid >= 0
        dv = vm.mean[vid[hit]] - p[hit]
        Sv = vm.cov[vid[hit]] + Cp[hit]
        gv = np.einsum("ni,nij,nj->n", dv, np.linalg.inv(Sv), dv)
        c_vg += np.sum(np.minimum(gv, 1e3)) + 1e3 * np.sum(~hit)
    return c_p2p, c_p2l, c_ndt, c_gicp, c_vg


C = np.array([costs(gt + np.array([o, 0.0, 0.0])) for o in offsets])
names = ["点対点 ICP", "点対面 ICP", "NDT (スコアの符号を反転)", "GICP", "VGICP (0.5 m + 1 m)"]
cols = [C_ORANGE, C_AQUA, "#e87ba4", C_BLUE, "#6250d6"]
fig, ax = plt.subplots(figsize=(8.4, 4.8))
for k in range(5):
    c = C[:, k]
    ax.plot(offsets, (c - c.min()) / (c.max() - c.min()), color=cols[k], label=names[k])
ax.axvline(0, color=C_INK, lw=0.8, ls="--")
ax.set_xlabel("正解からの x 方向のずれ [m]")
ax.set_ylabel("コスト (最小 0・最大 1 にそろえた)")
ax.set_title("正解のまわりのコストの形 (x 方向だけずらした断面)", color=C_INK)
ax.legend(fontsize=9, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3)
fig.tight_layout()
fig.savefig(fig_path("ex03a_6_cost_shape.png"))
plt.close(fig)
for k in range(5):
    c = (C[:, k] - C[:, k].min()) / (C[:, k].max() - C[:, k].min())
    print(f"  {names[k]:22s}: 最小の位置 {offsets[np.argmin(C[:, k])]:+.2f} m, ±0.5 m ずらしたときの値 {c[np.argmin(np.abs(offsets - 0.5))]:.2f}")

print("\n図を docs/figures/ex03a_*.png に保存しました")
print(
    """
やってみよう
  - VOX を 0.5 や 2.0 に変えて、NDT で分布を作れないボクセルがどう増減するか見る
  - MIN_PTS を 6 (PCL の既定値) にするとどうなる?
  - estimate_covariances の k (近傍点数) を 4 や 20 にすると、点ごとの共分散はどう変わる?
"""
)
