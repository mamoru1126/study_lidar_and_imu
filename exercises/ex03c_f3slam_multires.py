"""演習 3c: F3-SLAM のスキャンマッチング (複数解像度ボクセルから「平面で、なるべく大きい」ボクセルを選ぶ)

対応する資料: docs/03_point_cloud_registration.md 9 節
論文: 劉 陽, 山本 和成, 松井 敦史, 高橋 三郎, 阿部 敏久,
      「F3-SLAM：柔軟なセンサ構成に対応した高速かつ高精度な SLAM」,
      計測自動制御学会論文集, Vol.60, No.12, pp.631-637, 2024.

論文のスキャンマッチングを 2D で再現します (3D の「平面」は 2D では「線」になります)。
  1. ピラミッド型の複数解像度ボクセル地図 (下の層は上の層の半分の大きさ)
  2. 子ボクセルの (点数 n, 平均 μ, 共分散 Σ) から親ボクセルを更新する (14) 式
  3. 固有値の比 λ0/λ1 < θ なら「平面 (2D では線)」、それ以外は「非平面」
  4. 入力点ごとに、大きいボクセルから順に調べて「平面で、なるべく大きい」ボクセルを 1 つ選ぶ
  5. 平面なら点対面 (17) 式、非平面なら点対分布 (15) 式の残差でガウス・ニュートン法

論文に値が書かれていないもの (θ、ボクセルの最少点数、点とボクセルの最大距離、
平面残差の重み) は、この演習で決めた値です。

実行: python exercises/ex03c_f3slam_multires.py
"""

import numpy as np

from common import C_AQUA, C_BLUE, C_INK, C_MUTED, C_ORANGE, fig_path, rot2, drot2, setup_matplotlib, wrap_angle
from toy2d import raycast, voxel_downsample

rng = np.random.default_rng(31)
plt = setup_matplotlib()
from matplotlib.patches import Ellipse, Rectangle  # noqa: E402

# ---------------------------------------------------------------------------
# 環境: 論文の Fig. 2(a) と同じく「直角に交わる 2 枚の壁 + 円柱 2 本」
# ---------------------------------------------------------------------------
def circle_segs(c, r, n=24):
    t = np.linspace(0, 2 * np.pi, n + 1)
    p = np.stack([c[0] + r * np.cos(t), c[1] + r * np.sin(t)], axis=1)
    return [(p[i], p[i + 1]) for i in range(n)]


segs = [((0.0, 0.0), (8.0, 0.0)), ((0.0, 0.0), (0.0, 6.0))]
segs += circle_segs((3.0, 4.0), 0.35) + circle_segs((6.0, 4.0), 0.35)
segs = np.array(segs, dtype=float)

pose_map = np.array([4.5, 2.0, 0.0])  # 地図を作ったときのセンサ位置
pose_scan = np.array([3.8, 1.6, 0.15])  # 新しいスキャンの位置 (これを推定する)
map_pts = np.vstack([
    raycast(pose_map, segs, n_beams=1440, max_range=12, noise=0.02, rng=rng) @ rot2(pose_map[2]).T + pose_map[:2],
    raycast(pose_map + np.array([0.6, 1.0, 0.0]), segs, n_beams=1440, max_range=12, noise=0.02, rng=rng)
    @ rot2(pose_map[2]).T + pose_map[:2] + np.array([0.6, 1.0]),
])
map_pts = voxel_downsample(map_pts, 0.05)  # 地図点群 (世界座標)
scan = voxel_downsample(raycast(pose_scan, segs, n_beams=720, max_range=12, noise=0.02, rng=rng), 0.1)  # 入力点群 (センサ座標)
print(f"地図点 {len(map_pts)} 点, 入力スキャン {len(scan)} 点")

# ---------------------------------------------------------------------------
# 1. 複数解像度ボクセル地図
# ---------------------------------------------------------------------------
SIZES = [2.0, 1.0, 0.5]  # 上の層 (大きい) → 下の層 (小さい)。下の層は上の層の 1/2
THETA = 0.01  # λ0/λ1 < THETA なら平面 (2D では線)。論文に値の記載なし (0.05 だと円柱の一部も線と判定されやすい)
MIN_PTS = 5  # 候補にするボクセルの最少点数。論文に値の記載なし
EIG_FLOOR = 0.01  # 共分散の小さすぎる固有値を最大固有値の 1% に持ち上げる (NDT と同じ考え方)


class Layer:
    """1 つの解像度のボクセル群。キー → [n, μ, Σ]"""

    def __init__(self, size):
        self.size = size
        self.cells = {}

    def key(self, p):
        return (int(np.floor(p[0] / self.size)), int(np.floor(p[1] / self.size)))


layers = [Layer(s) for s in SIZES]

# 最下層 (0.5 m) に点を入れて n, μ, Σ を計算
bottom = layers[-1]
tmp = {}
for p in map_pts:
    tmp.setdefault(bottom.key(p), []).append(p)
for k, ps in tmp.items():
    ps = np.array(ps)
    bottom.cells[k] = [len(ps), ps.mean(axis=0), np.cov(ps.T, bias=True) if len(ps) > 1 else np.zeros((2, 2))]


def merge(children):
    """論文 (14) 式: 子ボクセルの統計量から親ボクセルの統計量を作る"""
    n = sum(c[0] for c in children)
    mu = sum(c[0] * c[1] for c in children) / n
    S = -np.outer(mu, mu) + sum(c[0] * (c[2] + np.outer(c[1], c[1])) for c in children) / n
    return [n, mu, S]


# 下の層から順に、親ボクセルを (14) 式で更新
for li in range(len(layers) - 2, -1, -1):
    parent, child = layers[li], layers[li + 1]
    groups = {}
    for k, c in child.cells.items():
        center = (np.array(k) + 0.5) * child.size
        groups.setdefault(parent.key(center), []).append(c)
    parent.cells = {k: merge(cs) for k, cs in groups.items()}

# (14) 式が「点から直接計算した値」と一致するか確認
err = 0.0
for k, (n, mu, S) in layers[0].cells.items():
    ps = map_pts[(np.floor(map_pts / SIZES[0]).astype(int) == np.array(k)).all(axis=1)]
    err = max(err, np.abs(ps.mean(axis=0) - mu).max(), np.abs(np.cov(ps.T, bias=True) - S).max())
print(f"(14) 式で子から作った親の μ, Σ と、点から直接計算した値の差の最大 = {err:.1e}  (≒ 0)")


def classify(cell):
    """平面 (線) / 非平面の判定と、平面の法線"""
    n, mu, S = cell
    w, V = np.linalg.eigh(S)  # 昇順 λ0 ≤ λ1
    is_plane = n >= MIN_PTS and w[1] > 0 and w[0] / w[1] < THETA
    return is_plane, V[:, 0]


for L in layers:
    for k, c in L.cells.items():
        c.append(classify(c))  # c = [n, μ, Σ, (is_plane, normal)]
    n_plane = sum(c[3][0] for c in L.cells.values())
    print(f"  {L.size:.1f} m 層: ボクセル {len(L.cells):3d} 個, うち平面 (線) {n_plane:3d} 個")


def whiten(S):
    """Σ^(-1/2) (論文 (15) 式)。小さすぎる固有値は持ち上げる"""
    w, V = np.linalg.eigh(S)
    w = np.maximum(w, EIG_FLOOR * w.max() + 1e-6)
    return V @ np.diag(w**-0.5) @ V.T


# ---------------------------------------------------------------------------
# 2. ボクセルの選び方 (論文 3.4 節、Fig. 3)
# ---------------------------------------------------------------------------
def select_voxel(q, max_dist_scale=1.0):
    """入力点 q (世界座標) について、大きいボクセルから順に調べる。
    候補: 点数 ≥ MIN_PTS かつ 点とボクセル中心 μ の距離 ≤ ボクセルの大きさ × max_dist_scale
    候補が平面ならそこで決定。非平面なら 1 つ小さい層へ。
    平面が見つからなければ、最後に見つかった非平面の候補 (一番小さい候補) を使う。
    """
    last = None
    for li, L in enumerate(layers):
        c = L.cells.get(L.key(q))
        if c is None or c[0] < MIN_PTS or np.linalg.norm(q - c[1]) > L.size * max_dist_scale:
            continue
        if c[3][0]:
            return li, L.key(q), "plane"
        last = (li, L.key(q), "nonplane")
    return last


# ---------------------------------------------------------------------------
# 3. ガウス・ニュートン法による位置合わせ
# ---------------------------------------------------------------------------
SIGMA_PLANE = 0.05  # 点対面の残差を σ で割って、非平面の白色化残差と単位をそろえる (この演習で決めた値)


def residuals(x, mode):
    """mode: "f3" (F3-SLAM 流の選択) または ("ndt", 層番号) (1 つの解像度だけ使う NDT 型)"""
    R, dR, t = rot2(x[2]), drot2(x[2]), x[:2]
    rs, Js, used = [], [], []
    for p in scan:
        q = R @ p + t
        dq = np.column_stack([np.eye(2), dR @ p])  # ∂q/∂(x, y, θ)
        if mode == "f3":
            sel = select_voxel(q)
            if sel is None:
                continue
            li, k, kind = sel
        else:
            li = mode[1]
            L = layers[li]
            k = L.key(q)
            c = L.cells.get(k)
            if c is None or c[0] < MIN_PTS:
                continue
            kind = "nonplane"
        n, mu, S, (is_plane, nrm) = layers[li].cells[k]
        if kind == "plane":  # 論文 (17)(18) 式: 点対面
            rs.append(np.array([nrm @ (q - mu) / SIGMA_PLANE]))
            Js.append((nrm @ dq / SIGMA_PLANE)[None, :])
        else:  # 論文 (15)(16) 式: 点対分布 (白色化したマハラノビス距離)
            W = whiten(S)
            rs.append(W @ (q - mu))
            Js.append(W @ dq)
        used.append((li, k, kind))
    return rs, Js, used


def align(x0, mode, iters=20):
    x = x0.copy()
    for _ in range(iters):
        rs, Js, _ = residuals(x, mode)
        if len(rs) < 5:
            break
        J = np.vstack(Js)
        r = np.concatenate(rs)
        dx = np.linalg.solve(J.T @ J + 1e-6 * np.eye(3), -J.T @ r)
        x = x + dx
        if np.abs(dx).max() < 1e-6:
            break
    return x


truth = pose_scan.copy()
_, _, used = residuals(truth, "f3")
print("\n正解の姿勢で、各入力点がどのボクセルを選んだか")
for li, L in enumerate(layers):
    for kind, name in (("plane", "平面 (点対面)"), ("nonplane", "非平面 (点対分布)")):
        c = sum(1 for u in used if u[0] == li and u[2] == kind)
        print(f"  {L.size:.1f} m 層の{name:10s}: {c:3d} 点")

print("\n初期値をずらして位置合わせ (各 30 回)。成功 = 3 cm かつ 0.5° 以内")
modes = {"NDT 型 2.0 m のみ": ("ndt", 0), "NDT 型 1.0 m のみ": ("ndt", 1), "NDT 型 0.5 m のみ": ("ndt", 2), "F3-SLAM 型 (選択)": "f3"}
OFFSETS = [(0.3, 3.0), (0.6, 6.0), (1.0, 10.0)]  # (位置のずれ [m], 向きのずれの最大 [deg])
results = {name: [] for name in modes}
for off, deg in OFFSETS:
    print(f"  初期値のずれ {off} m, 最大 {deg:.0f}°")
    for name, mode in modes.items():
        errs, ok = [], 0
        rr = np.random.default_rng(5)
        for _ in range(30):
            a = rr.uniform(0, 2 * np.pi)
            x0 = truth + np.array([off * np.cos(a), off * np.sin(a), np.radians(rr.uniform(-deg, deg))])
            x = align(x0, mode)
            e_t, e_r = np.linalg.norm(x[:2] - truth[:2]), abs(wrap_angle(x[2] - truth[2]))
            errs.append(e_t)
            ok += e_t < 0.03 and e_r < np.radians(0.5)
        results[name].append((ok / 30, np.median(errs)))
        print(f"    {name:18s}: 成功率 {ok/30*100:5.1f} %, 位置誤差の中央値 {np.median(errs)*100:5.2f} cm")
print("  → 誤差 1〜2 cm の差は、この 2D の例の測定ノイズ (2 cm) と同じ程度なので、精度の優劣は言えない")
print("    違いがはっきり出るのは、初期値のずれが大きいときの成功率 (収束域の広さ)")

# ---------------------------------------------------------------------------
# 図 1: 論文 Fig. 2 の再現 (NDT のボクセルサイズのジレンマ)
# ---------------------------------------------------------------------------
def draw_cell_ellipse(ax, mu, S, size, **kw):
    w, V = np.linalg.eigh(S)
    w = np.maximum(w, EIG_FLOOR * w.max())
    ang = np.degrees(np.arctan2(V[1, 1], V[0, 1]))
    ax.add_patch(Ellipse(mu, 2 * 2 * np.sqrt(w[1]), 2 * 2 * np.sqrt(w[0]), angle=ang, **kw))


def grid(ax, size, lw=0.8, color="#cfd5de"):
    for x in np.arange(-1, 9.01, size):
        ax.axvline(x, color=color, lw=lw, zorder=1)
    for y in np.arange(-1, 7.01, size):
        ax.axhline(y, color=color, lw=lw, zorder=1)
    ax.grid(False)


def frame(ax, title):
    ax.set_xlim(-0.8, 8.4)
    ax.set_ylim(-0.8, 6.2)
    ax.set_aspect("equal")
    ax.set_title(title, color=C_INK)


fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
ax = axes[0]
ax.scatter(map_pts[:, 0], map_pts[:, 1], s=2, color=C_INK)
frame(ax, "(a) 環境: 直角の壁 2 枚と円柱 2 本")
for ax, li, title in ((axes[1], 0, "(b) NDT 2 m: 角で分布がふくらむ"), (axes[2], 1, "(c) NDT 1 m: 角は良くなるが 1 つのボクセルの点が減る")):
    L = layers[li]
    grid(ax, L.size, lw=1.2)
    ax.scatter(map_pts[:, 0], map_pts[:, 1], s=2, color=C_MUTED, zorder=2)
    for k, (n, mu, S, _) in L.cells.items():
        if n < MIN_PTS:
            ax.add_patch(Rectangle((k[0] * L.size, k[1] * L.size), L.size, L.size, fc=C_ORANGE, alpha=0.18, ec="none"))
        else:
            draw_cell_ellipse(ax, mu, S, L.size, fc=C_AQUA, alpha=0.35, ec=C_AQUA, lw=1.0, zorder=3)
    frame(ax, title)
axes[2].add_patch(Rectangle((0, 0), 0, 0, fc=C_ORANGE, alpha=0.3, label=f"点が {MIN_PTS} 未満 (分布を作れない)"))
axes[2].legend(fontsize=9, loc="upper right")
fig.tight_layout()
fig.savefig(fig_path("ex03c_1_ndt_voxel_size.png"))
plt.close(fig)

# ---------------------------------------------------------------------------
# 図 2: 論文 Fig. 3 の再現 (入力点ごとに選ばれたボクセル)
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(13, 5.0), gridspec_kw={"width_ratios": [1.35, 1]})
ax = axes[0]
grid(ax, SIZES[-1], lw=0.5, color="#e3e6eb")
ax.scatter(map_pts[:, 0], map_pts[:, 1], s=2, color=C_MUTED, zorder=2)
lv_cols = {0: C_BLUE, 1: C_AQUA, 2: C_ORANGE}
for li, k, kind in set(used):
    L = layers[li]
    n, mu, S, (is_plane, nrm) = L.cells[k]
    ax.add_patch(Rectangle((k[0] * L.size, k[1] * L.size), L.size, L.size, fill=False, ec=lv_cols[li], lw=2.0, zorder=4))
    if kind == "plane":
        tvec = np.array([-nrm[1], nrm[0]])
        a, b = mu - 0.45 * L.size * tvec, mu + 0.45 * L.size * tvec
        ax.plot([a[0], b[0]], [a[1], b[1]], color=C_INK, lw=2.2, zorder=5)
        ax.annotate("", mu + 0.25 * L.size * nrm, mu, arrowprops=dict(arrowstyle="->", color=C_INK, lw=1.0), zorder=5)
    else:
        draw_cell_ellipse(ax, mu, S, L.size, fc="#e87ba4", alpha=0.45, ec="#c2507c", lw=1.0, zorder=5)
q = scan @ rot2(truth[2]).T + truth[:2]
ax.scatter(q[:, 0], q[:, 1], s=6, color=C_INK, zorder=6, alpha=0.6)
for li in range(3):
    ax.plot([], [], color=lv_cols[li], lw=2.5, label=f"選ばれた {SIZES[li]:.1f} m ボクセル")
ax.plot([], [], color=C_INK, lw=2.2, label="平面 (線) → 点対面の残差")
ax.add_patch(Ellipse((0, 0), 0, 0, fc="#e87ba4", alpha=0.6, label="非平面 → 点対分布の残差"))
frame(ax, "入力点ごとに「平面で、なるべく大きい」ボクセルを選ぶ")
ax.legend(fontsize=8.5, loc="upper center", bbox_to_anchor=(0.5, -0.06), ncol=3)

ax = axes[1]
names = list(results)
cols = ["#b9c0cc", "#8a93a3", "#586273", C_BLUE]
wbar = 0.2
for i, name in enumerate(names):
    vals = [r[0] * 100 for r in results[name]]
    xs = np.arange(len(OFFSETS)) + (i - 1.5) * wbar
    ax.bar(xs, vals, width=wbar, color=cols[i], label=name)
ax.set_xticks(np.arange(len(OFFSETS)))
ax.set_xticklabels([f"{o} m / {d:.0f}°" for o, d in OFFSETS])
ax.set_xlabel("初期値のずれ (位置 / 向きの最大)")
ax.set_ylabel("成功率 [%]  (3 cm かつ 0.5° 以内, 30 回)")
ax.set_ylim(0, 112)
ax.set_title("ずれが大きくても合うか (収束域の広さ)", color=C_INK)
ax.legend(fontsize=8.5, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=2)
fig.tight_layout()
fig.savefig(fig_path("ex03c_2_voxel_selection.png"))
plt.close(fig)
print("\n図を docs/figures/ex03c_*.png に保存しました")
print(
    """
やってみよう
  - THETA を 0.05 や 0.2 にすると、平面と判定されるボクセルと位置合わせの結果はどう変わる?
  - SIZES を [4.0, 2.0, 1.0, 0.5] の 4 層にするとどうなる?
  - select_voxel の max_dist_scale を小さくすると、遠すぎる対応が減る代わりに何が起こる?
"""
)
