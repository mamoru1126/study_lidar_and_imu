"""演習 2a: 誤差ベクトル・正規分布・共分散行列を数値で確かめる

対応する資料: docs/02_probability_and_least_squares.md (1〜5 節)

この演習で確かめること
  1. 誤差ベクトル = 「予測した観測」と「実際の観測」の差を、成分ごとに並べたもの
  2. 正規分布の exp(-(z-μ)²/(2σ²)) の「1/2」は、σ をちょうど標準偏差にするために必要
  3. 独立な 2 つのノイズの確率を掛け算すると、自然に e^T Σ^-1 e (Σ は対角行列) が出てくる
  4. 壁のように斜めに伸びたばらつきは、対角でない共分散行列でしか表せない
  5. 同じ長さの誤差でも、向きによって「ありえなさ」(マハラノビス距離) が違う
  6. 精度の違う測定値を組み合わせる最尤推定 = 重み付き平均 (重み = 1/σ²)

実行: python exercises/ex02a_gaussian.py
"""

import numpy as np

from common import C_AQUA, C_BLUE, C_INK, C_MUTED, C_ORANGE, fig_path, rot2, setup_matplotlib

np.set_printoptions(precision=4, suppress=True)
rng = np.random.default_rng(21)


def section(title):
    print("\n" + "=" * 70 + f"\n{title}\n" + "=" * 70)


# ---------------------------------------------------------------------------
section("1. 誤差ベクトルとは")
# ロボットの推定位置 x と、位置が分かっている 2 つのランドマーク
x_est = np.array([3.0, 1.0])
landmarks = np.array([[0.0, 0.0], [6.0, 0.0]])
z = np.array([3.30, 3.05])  # 実際に測った距離 [m]
h = np.linalg.norm(x_est - landmarks, axis=1)  # 推定位置から「こう見えるはず」と予測した距離
e = h - z
print("予測した距離 h(x) =", h)
print("実際の測定値   z  =", z)
print("誤差ベクトル e = h(x) - z =", e)
print(f"→ e の 1 成分目 {e[0]:+.3f}: 予測の方が {abs(e[0]):.3f} m 短い (実際は予測より遠くに見えた)")
print(f"  e の 2 成分目 {e[1]:+.3f}: 予測の方が {abs(e[1]):.3f} m 長い (実際は予測より近くに見えた)")
print("  推定位置 x を動かすと e が変わる。e が全体として 0 に近づく x を探すのが最小二乗法")

# ---------------------------------------------------------------------------
section("2. 1 次元の正規分布と「1/2」")
sigma = 0.5
samples = rng.normal(0.0, sigma, size=200000)
print(f"σ = {sigma} の正規乱数を 20 万個作ると、標準偏差は {samples.std():.4f} (≒ σ)")

z_grid = np.linspace(-4, 4, 40001)
dz = z_grid[1] - z_grid[0]


def std_of_shape(f):
    """形 f(z) を面積 1 に正規化したときの標準偏差"""
    p = f / (f.sum() * dz)
    return np.sqrt(np.sum(z_grid**2 * p) * dz)


with_half = np.exp(-(z_grid**2) / (2 * sigma**2))
without_half = np.exp(-(z_grid**2) / (sigma**2))
print(f"exp(-z²/(2σ²)) の山の標準偏差 = {std_of_shape(with_half):.4f}  ← σ = {sigma} と一致")
print(f"exp(-z²/σ²)    の山の標準偏差 = {std_of_shape(without_half):.4f}  ← σ/√2 = {sigma/np.sqrt(2):.4f} になってしまう")
print(f"z = σ のとき山の高さは頂上の exp(-1/2) = {np.exp(-0.5):.3f} 倍、z = 2σ で {np.exp(-2):.3f} 倍、z = 3σ で {np.exp(-4.5):.4f} 倍")
inside = [np.mean(np.abs(samples) < k * sigma) for k in (1, 2, 3)]
print(f"±1σ, ±2σ, ±3σ に入る割合: {inside[0]*100:.1f}%, {inside[1]*100:.1f}%, {inside[2]*100:.2f}%")

# ---------------------------------------------------------------------------
section("3. 独立な 2 次元のノイズ: 掛け算すると行列の形が出てくる")
sx, sy = 0.3, 0.1
ex_, ey_ = 0.2, -0.05
p_product = np.exp(-(ex_**2) / (2 * sx**2)) * np.exp(-(ey_**2) / (2 * sy**2))
Sigma_diag = np.diag([sx**2, sy**2])
ev = np.array([ex_, ey_])
p_matrix = np.exp(-0.5 * ev @ np.linalg.inv(Sigma_diag) @ ev)
print(f"p(ex)·p(ey) (定数を除く)        = {p_product:.6f}")
print(f"exp(-½ e^T Σ^-1 e), Σ=diag(σx²,σy²) = {p_matrix:.6f}  ← 同じ")
print("Σ =\n", Sigma_diag)

# ---------------------------------------------------------------------------
section("4. 斜めに伸びたばらつき (壁に沿った点の位置の不確かさ)")
wall_deg = 30.0
s_along, s_across = 0.40, 0.05  # 壁に沿う方向は不確か、壁に垂直な方向は確か
R = rot2(np.radians(wall_deg))  # 壁の座標 (u: 沿う, v: 垂直) → x, y 座標
uv = np.column_stack([rng.normal(0, s_along, 5000), rng.normal(0, s_across, 5000)])
xy = uv @ R.T
Sigma_sample = np.cov(xy.T)
Sigma_model = R @ np.diag([s_along**2, s_across**2]) @ R.T
print("サンプルから計算した共分散 np.cov =\n", Sigma_sample)
print("R diag(σ沿², σ垂²) R^T で作った共分散 =\n", Sigma_model)
print("→ 非対角成分 (x と y が一緒に動く度合い) が 0 でない。対角行列では表せない")
corr = Sigma_model[0, 1] / np.sqrt(Sigma_model[0, 0] * Sigma_model[1, 1])
print(f"相関係数 = {corr:.3f} (x が大きいとき y も大きくなりやすい)")
w, V = np.linalg.eigh(Sigma_model)
ang = (np.degrees(np.arctan2(V[1, 1], V[0, 1])) + 90) % 180 - 90  # 軸の向きは ±180° 同じなので [-90, 90) に丸める
print(f"固有値分解: 標準偏差 = {np.sqrt(w[::-1])}, 長い軸の向き = {ang:.1f}° (壁の向きと一致)")

# ---------------------------------------------------------------------------
section("5. マハラノビス距離: 同じ 0.3 m のずれでも、向きでありえなさが違う")
Om = np.linalg.inv(Sigma_model)
e_along = 0.3 * R[:, 0]  # 壁に沿って 0.3 m
e_across = 0.3 * R[:, 1]  # 壁に垂直に 0.3 m
for name, ev in [("壁に沿って 0.3 m", e_along), ("壁に垂直に 0.3 m", e_across)]:
    d = np.sqrt(ev @ Om @ ev)
    print(f"  {name}: ユークリッド距離 {np.linalg.norm(ev):.2f} m, マハラノビス距離 {d:.2f} (σ 何個分),"
          f" 確率密度は頂上の {np.exp(-0.5 * d**2):.2e} 倍")

# ---------------------------------------------------------------------------
section("6. 最尤推定 = 重み付き平均")
meas = np.array([10.12, 9.95, 10.40])
sig = np.array([0.05, 0.10, 0.30])
w_ = 1 / sig**2
x_ml = np.sum(w_ * meas) / np.sum(w_)
grid = np.linspace(9.5, 10.5, 100001)
cost = 0.5 * (((grid[:, None] - meas) / sig) ** 2).sum(axis=1)
print("測定値", meas, " 標準偏差", sig)
print(f"  普通の平均             = {meas.mean():.4f}")
print(f"  重み付き平均 (重み 1/σ²) = {x_ml:.4f}")
print(f"  コスト ½Σ((x-z)/σ)² を総当たりで最小化 = {grid[np.argmin(cost)]:.4f}  ← 重み付き平均と一致")
print(f"  推定値の標準偏差 1/√(Σ 1/σ²) = {1/np.sqrt(w_.sum()):.4f}  (一番良いセンサ単独の {sig.min()} より小さい)")

# ---------------------------------------------------------------------------
plt = setup_matplotlib()
fig, axes = plt.subplots(1, 3, figsize=(14, 5.6))
t = np.linspace(0, 2 * np.pi, 200)
circle = np.vstack([np.cos(t), np.sin(t)])

ax = axes[0]
ax.hist(samples, bins=120, range=(-2, 2), density=True, color="#d7e3f4", label="20 万回の測定のヒストグラム")
zz = np.linspace(-2, 2, 400)
ax.plot(zz, np.exp(-(zz**2) / (2 * sigma**2)) / np.sqrt(2 * np.pi * sigma**2), color=C_BLUE, label="exp(−z²/(2σ²)): 正しい式")
ax.plot(zz, np.exp(-(zz**2) / sigma**2) / np.sqrt(np.pi * sigma**2), "--", color=C_ORANGE, label="exp(−z²/σ²): ½ を忘れた式")
for k, lab in ((1, "σ"), (2, "2σ")):
    ax.axvline(k * sigma, color=C_MUTED, lw=0.8)
    ax.axvline(-k * sigma, color=C_MUTED, lw=0.8)
    ax.text(k * sigma + 0.03, 1.02, lab, color=C_INK, fontsize=10)
ax.set_xlim(-2, 2)
ax.set_ylim(0, 1.2)
ax.set_xlabel("測定誤差 z [m]  (σ = 0.5 m)")
ax.set_title("1 次元: ½ がないと山が細くなりすぎる", color=C_INK)
ax.legend(fontsize=9, loc="upper center", bbox_to_anchor=(0.5, -0.16))

ax = axes[1]
ax.plot([-1.2 * R[0, 0], 1.2 * R[0, 0]], [-1.2 * R[1, 0], 1.2 * R[1, 0]], color=C_INK, lw=4, alpha=0.18, label="壁 (30° 傾いている)")
ax.scatter(xy[:1500, 0], xy[:1500, 1], s=3, color=C_MUTED, alpha=0.6, label="ノイズのサンプル")
for k in (1, 2, 3):
    ell = (V @ np.diag(k * np.sqrt(w)) @ circle).T
    ax.plot(ell[:, 0], ell[:, 1], color=C_BLUE, lw=1.6 if k == 1 else 0.9, label="1σ, 2σ, 3σ の楕円" if k == 1 else None)
ax.set_aspect("equal")
ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-0.9, 0.9)
ax.set_xlabel("x 方向の誤差 [m]")
ax.set_ylabel("y 方向の誤差 [m]")
ax.set_title("2 次元: 斜めの楕円 = 非対角の共分散", color=C_INK)
ax.legend(fontsize=9, loc="upper center", bbox_to_anchor=(0.5, -0.16))

ax = axes[2]
ax.plot(0.3 * np.cos(t), 0.3 * np.sin(t), "--", color=C_MUTED, label="ユークリッド距離 0.3 m の円")
for k in (1, 2, 4, 6):
    ell = (V @ np.diag(k * np.sqrt(w)) @ circle).T
    ax.plot(ell[:, 0], ell[:, 1], color=C_BLUE, lw=1.2 if k == 1 else 0.8, alpha=1.0 if k == 1 else 0.6,
            label="マハラノビス距離 1, 2, 4, 6 の楕円" if k == 1 else None)
ax.scatter(*e_along, s=70, color=C_AQUA, zorder=5, label="壁に沿って 0.3 m → 0.75σ")
ax.scatter(*e_across, s=70, color=C_ORANGE, zorder=5, label="壁に垂直に 0.3 m → 6σ")
ax.set_aspect("equal")
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.4, 0.4)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("同じ長さでも「σ 何個分」は違う", color=C_INK)
ax.legend(fontsize=9, loc="upper center", bbox_to_anchor=(0.5, -0.16))
fig.tight_layout()
out = fig_path("ex02a_gaussian.png")
fig.savefig(out)
print("\n図を保存しました:", out)

print(
    """
やってみよう
  - wall_deg を 0 にすると、共分散行列の非対角成分はどうなる?
  - s_along と s_across を同じにすると、楕円はどうなる? (向きの情報が消える)
  - 6. の 3 つ目の測定値を 11.0 (外れ値) にすると、普通の平均と重み付き平均はどう変わる?
"""
)
