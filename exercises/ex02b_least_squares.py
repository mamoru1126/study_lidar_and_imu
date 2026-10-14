"""演習 2b: 非線形最小二乗 (ガウス・ニュートン法 / レーベンバーグ・マーカート法)

対応する資料: docs/02_probability_and_least_squares.md (6 節以降)

問題設定
  2D 平面上のロボットが、位置の分かっている 4 つのランドマークまでの距離を測った。
  距離センサの精度はランドマークごとに違う (標準偏差 σ_k)。
  ロボットの位置 x = (x, y) を推定したい。

  最尤推定 = 次のコスト (マハラノビス距離の 2 乗和) の最小化:
      F(x) = Σ_k ( (||x - l_k|| - z_k) / σ_k )^2

この演習で確かめること
  1. 残差 r とヤコビアン J から H = J^T Ω J, b = J^T Ω r を作り、 H δ = -b を解く
  2. 非線形最小二乗は初期値しだいで「局所解」に落ちる
     (→ 論文では IMU による予測が良い初期値を与えてくれる)
  3. 精度の違うセンサは重み Ω (= 共分散の逆行列) で正しく扱う
  4. 解の不確かさ (共分散) は H^{-1} で近似できる

実行: python exercises/ex02b_least_squares.py
"""

import numpy as np

from common import C_AQUA, C_BLUE, C_INK, C_MUTED, C_ORANGE, fig_path, setup_matplotlib

np.set_printoptions(precision=4, suppress=True)
rng = np.random.default_rng(3)

landmarks = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 8.0], [0.0, 8.0]])
sigmas = np.array([0.05, 0.05, 0.6, 0.6])  # 右上と左上のランドマークは精度が悪い
x_true = np.array([6.0, 3.0])
z = np.linalg.norm(x_true - landmarks, axis=1) + rng.normal(scale=sigmas)
Omega = np.diag(1.0 / sigmas**2)  # 情報行列 (共分散の逆行列)


def residual(x):
    return np.linalg.norm(x - landmarks, axis=1) - z


def jacobian(x):
    d = x - landmarks
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def cost(x):
    r = residual(x)
    return float(r @ Omega @ r)


def gauss_newton(x0, iters=10, verbose=True):
    x = x0.copy()
    path = [x.copy()]
    for it in range(iters):
        r, J = residual(x), jacobian(x)
        H = J.T @ Omega @ J
        b = J.T @ Omega @ r
        delta = np.linalg.solve(H, -b)
        x = x + delta
        path.append(x.copy())
        if verbose:
            print(f"  GN iter {it}: x = {x}, cost = {cost(x):10.4f}, |δ| = {np.linalg.norm(delta):.2e}")
        if np.linalg.norm(delta) < 1e-8:
            break
    return x, np.array(path)


def levenberg_marquardt(x0, iters=30, lam=1e-1, verbose=True):
    x = x0.copy()
    path = [x.copy()]
    for it in range(iters):
        r, J = residual(x), jacobian(x)
        H = J.T @ Omega @ J
        b = J.T @ Omega @ r
        # (H + λ diag(H)) δ = -b : λ が大きいと勾配降下法のように慎重に進む
        delta = np.linalg.solve(H + lam * np.diag(np.diag(H)), -b)
        if cost(x + delta) < cost(x):
            x = x + delta
            lam *= 0.3  # うまくいったのでガウス・ニュートン法に近づける
            path.append(x.copy())
        else:
            lam *= 10.0  # 失敗したので慎重にする
        if verbose:
            print(f"  LM iter {it}: x = {x}, cost = {cost(x):10.4f}, λ = {lam:.1e}")
        if np.linalg.norm(delta) < 1e-8:
            break
    return x, np.array(path)


print("真の位置:", x_true)
print("\n[ガウス・ニュートン法] 良い初期値 (5, 4) から")
x_gn, path_gn = gauss_newton(np.array([5.0, 4.0]))

print("\n[レーベンバーグ・マーカート法] 遠い初期値 (1.5, 7.5) から")
x_lm, path_lm = levenberg_marquardt(np.array([1.5, 7.5]))

print("\n[ガウス・ニュートン法] 下側の初期値 (5, -3) から")
x_gn_bad, path_gn_bad = gauss_newton(np.array([5.0, -3.0]), iters=8)
print(f"  → コスト {cost(x_gn_bad):.1f} の点に収束。真の解 (コスト {cost(x_gn):.2f}) とは別の『局所解』です。")
print("    精度の良い 2 つのランドマークが y=0 の線上にあるため、その線で折り返した位置もそれらしく見えてしまう。")

# 解の共分散 ≈ H^{-1}
J = jacobian(x_gn)
H = J.T @ Omega @ J
cov = np.linalg.inv(H)
print("\n推定値の共分散 H^{-1} =\n", cov)
print("標準偏差 [m] =", np.sqrt(np.diag(cov)))
print("→ 楕円は縦長: 精度の良い下側 2 つのランドマークは主に x 方向の位置をよく決めるため")

# 重み付き (正しい Ω) と重みなし (Ω = I) を、ノイズを変えて 2000 回比較する
def solve_once(z_obs, W, x0):
    x = x0.copy()
    for _ in range(10):
        d = x - landmarks
        rng_ = np.linalg.norm(d, axis=1)
        r, J = rng_ - z_obs, d / rng_[:, None]
        x = x + np.linalg.solve(J.T @ W @ J, -J.T @ W @ r)
    return x


err_w, err_u = [], []
for _ in range(2000):
    z_sim = np.linalg.norm(x_true - landmarks, axis=1) + rng.normal(scale=sigmas)
    err_w.append(np.linalg.norm(solve_once(z_sim, Omega, x_true + 0.5) - x_true))
    err_u.append(np.linalg.norm(solve_once(z_sim, np.eye(4), x_true + 0.5) - x_true))
print("\n2000 回の試行での平均誤差 (RMS)")
print(f"  重み付き (Ω = diag(1/σ²)) : {np.sqrt(np.mean(np.square(err_w))):.3f} m")
print(f"  重みなし (Ω = I)          : {np.sqrt(np.mean(np.square(err_u))):.3f} m")
print("  → センサの精度を重みとして入れた方が、平均的に正確になる")

# ---------------------------------------------------------------------------
plt = setup_matplotlib()
fig, ax = plt.subplots(figsize=(6.6, 5.4))
gx, gy = np.meshgrid(np.linspace(-1, 11, 240), np.linspace(-4, 9, 260))
F = np.zeros_like(gx)
for i in range(gx.shape[0]):
    for j in range(gx.shape[1]):
        F[i, j] = cost(np.array([gx[i, j], gy[i, j]]))
ax.contour(gx, gy, np.log10(F + 1e-3), levels=18, colors=C_MUTED, linewidths=0.6)
ax.scatter(landmarks[:, 0], landmarks[:, 1], marker="s", s=70, color=C_INK, label="ランドマーク", zorder=5)
for lm, s in zip(landmarks, sigmas):
    ax.annotate(f"σ={s}", lm, textcoords="offset points", xytext=(6, 6), fontsize=9, color=C_INK)
ax.plot(path_lm[:, 0], path_lm[:, 1], "o-", color=C_BLUE, ms=5, label="LM 法: 初期値 (1.5, 7.5) → 真の解")
ax.plot(path_gn_bad[:, 0], path_gn_bad[:, 1], "o-", color=C_ORANGE, ms=5, label="GN 法: 初期値 (5, -3) → 局所解")
ax.scatter(*x_true, marker="*", s=200, color=C_AQUA, edgecolor="white", zorder=6, label="真の位置")

# 共分散楕円 (3σ)。そのままだと小さすぎて見えないので 10 倍に拡大して描く
vals, vecs = np.linalg.eigh(cov)
t = np.linspace(0, 2 * np.pi, 100)
ell = (vecs @ np.diag(10 * 3 * np.sqrt(vals)) @ np.vstack([np.cos(t), np.sin(t)])).T + x_gn
ax.plot(ell[:, 0], ell[:, 1], color=C_INK, lw=1.2, label="推定の 3σ 楕円 (H^-1 から, 10倍拡大)")
ax.set_xlim(-1, 11)
ax.set_ylim(-4, 9)
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("コスト関数の等高線と最適化の軌跡", color=C_INK)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2, fontsize=9)
out = fig_path("ex02_least_squares.png")
fig.savefig(out)
print("\n図を保存しました:", out)

print(
    """
やってみよう
  - sigmas を全部同じにすると共分散楕円の形はどう変わる?
  - 初期値をいろいろ変えて、どこから始めると局所解に落ちるかを調べる
  - 左上・右上のランドマークの σ を小さくすると局所解が消えることを確かめる
  - ランドマークを 2 つに減らすと H が特異 (逆行列なし) に近づくことを確認する
"""
)
