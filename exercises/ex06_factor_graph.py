"""演習 6: ファクターグラフ・スパース性・周辺化 (固定ラグ平滑化)

対応する資料: docs/05_factor_graph.md, docs/06_fixed_lag_smoothing.md

GTSAM を使わず、numpy だけで小さなファクターグラフを組み立てて解きます。
  変数 (variable) : ロボットの姿勢 x_i = (x, y, θ)
  因子 (factor)   : - 事前分布因子   x_0 は既知
                    - 相対移動因子   オドメトリで測った x_i → x_{i+1} の移動
                    - 絶対位置因子   「地図とのマッチング」の代わり。一部の区間でしか得られない

この演習で確かめること
  1. 情報行列 H = J^T J は、因子が繋がっている変数のところだけ非ゼロ (スパース)
  2. 古い変数を「周辺化」すると、その情報は隣の変数への事前分布 (密な小ブロック) として残る
  3. 固定ラグ平滑化 (ウィンドウ最適化) でも、全体を一括最適化した結果に近い推定ができる

実行: python exercises/ex06_factor_graph.py
"""

import numpy as np

from common import C_AQUA, C_BLUE, C_INK, C_MUTED, C_ORANGE, C_SHADE, drot2, fig_path, rot2, setup_matplotlib, wrap_angle

rng = np.random.default_rng(6)
N = 60
SIG_ODOM = np.array([0.05, 0.05, np.radians(1.5)])  # 相対移動の誤差 (x, y, θ)
SIG_ABS = 0.10  # 絶対位置の誤差
HAS_MAP = lambda i: i < 15 or i >= 45  # 「地図」がある区間

# ---------------------------------------------------------------------------
# 真の軌跡と観測を作る
# ---------------------------------------------------------------------------
truth = [np.array([0.0, 0.0, 0.0])]
for i in range(N - 1):
    turn = 0.12 if 20 <= i < 40 else 0.02
    x, y, th = truth[-1]
    truth.append(np.array([x + 0.5 * np.cos(th), y + 0.5 * np.sin(th), th + turn]))
truth = np.array(truth)

odom = []  # (i, j, 計測値)
for i in range(N - 1):
    xi, xj = truth[i], truth[i + 1]
    dt = rot2(xi[2]).T @ (xj[:2] - xi[:2])
    z = np.array([*dt, xj[2] - xi[2]]) + rng.normal(scale=SIG_ODOM)
    odom.append((i, i + 1, z))
absobs = [(i, truth[i, :2] + rng.normal(scale=SIG_ABS, size=2)) for i in range(N) if HAS_MAP(i)]


# ---------------------------------------------------------------------------
# 因子: それぞれ「白色化済みの残差 r」と「各変数に対するヤコビアン」を返す
# ---------------------------------------------------------------------------
def between_factor(xi, xj, z):
    Ri, dRi = rot2(xi[2]), drot2(xi[2])
    dtw = xj[:2] - xi[:2]
    r = np.array([*(Ri.T @ dtw - z[:2]), wrap_angle(xj[2] - xi[2] - z[2])]) / SIG_ODOM
    Ji = np.zeros((3, 3))
    Jj = np.zeros((3, 3))
    Ji[:2, :2], Ji[:2, 2], Ji[2, 2] = -Ri.T, dRi.T @ dtw, -1.0
    Jj[:2, :2], Jj[2, 2] = Ri.T, 1.0
    return r, Ji / SIG_ODOM[:, None], Jj / SIG_ODOM[:, None]


def abs_factor(xi, z):
    J = np.zeros((2, 3))
    J[:, :2] = np.eye(2)
    return (xi[:2] - z) / SIG_ABS, J / SIG_ABS


def prior_factor(xi, z, sig=1e-3):
    r = np.array([*(xi[:2] - z[:2]), wrap_angle(xi[2] - z[2])]) / sig
    return r, np.eye(3) / sig


class LinearPrior:
    """周辺化で生まれる線形の事前分布因子: r = S (x ⊟ x0) + r0"""

    def __init__(self, keys, x0, S, r0):
        self.keys, self.x0, self.S, self.r0 = keys, x0, S, r0

    def linearize(self, X):
        d = np.concatenate([X[k] - self.x0[n] for n, k in enumerate(self.keys)])
        d[2::3] = wrap_angle(d[2::3])
        return self.S @ d + self.r0, self.S


def build_system(X, keys, factors):
    """変数 keys について H, g を組み立てる (cost = ½ Σ |r|^2 を 2 次近似)"""
    index = {k: n for n, k in enumerate(keys)}
    H = np.zeros((3 * len(keys), 3 * len(keys)))
    g = np.zeros(3 * len(keys))
    cost = 0.0
    for f in factors:
        fkeys, r, Js = f(X)
        cost += 0.5 * r @ r
        J = np.zeros((len(r), 3 * len(keys)))
        for k, Jk in zip(fkeys, Js):
            J[:, 3 * index[k] : 3 * index[k] + 3] = Jk
        H += J.T @ J
        g += J.T @ r
    return H, g, cost


def make_factors(odom_list, abs_list, prior_x0=None, linear_priors=()):
    """因子のリストを作る。各因子は X を受け取り (関係する変数のキー, 残差, ヤコビアンのリスト) を返す関数"""
    fs = []
    if prior_x0 is not None:

        def f_prior(X, z=prior_x0):
            r, J = prior_factor(X[0], z)
            return [0], r, [J]

        fs.append(f_prior)
    for i, j, z in odom_list:

        def f_between(X, i=i, j=j, z=z):
            r, Ji, Jj = between_factor(X[i], X[j], z)
            return [i, j], r, [Ji, Jj]

        fs.append(f_between)
    for i, z in abs_list:

        def f_abs(X, i=i, z=z):
            r, J = abs_factor(X[i], z)
            return [i], r, [J]

        fs.append(f_abs)
    for lp in linear_priors:

        def f_lin(X, lp=lp):
            r, S = lp.linearize(X)
            return lp.keys, r, [S[:, 3 * n : 3 * n + 3] for n in range(len(lp.keys))]

        fs.append(f_lin)
    return fs


def gauss_newton(X, keys, factors, iters=8):
    for _ in range(iters):
        H, g, cost = build_system(X, keys, factors)
        dx = np.linalg.solve(H + 1e-9 * np.eye(len(g)), -g)
        for n, k in enumerate(keys):
            X[k] = X[k] + dx[3 * n : 3 * n + 3]
        if np.abs(dx).max() < 1e-9:
            break
    return X


# ---------------------------------------------------------------------------
# 1. オドメトリだけで積分 (デッドレコニング)
# ---------------------------------------------------------------------------
dead = [truth[0].copy()]
for i, j, z in odom:
    x = dead[-1]
    dead.append(np.array([*(x[:2] + rot2(x[2]) @ z[:2]), x[2] + z[2]]))
dead = np.array(dead)

# ---------------------------------------------------------------------------
# 2. 全変数を一括で最適化 (バッチ最適化)
# ---------------------------------------------------------------------------
X = {k: dead[k].copy() for k in range(N)}
keys_all = list(range(N))
factors_all = make_factors(odom, absobs, prior_x0=truth[0])
X = gauss_newton(X, keys_all, factors_all)
batch = np.array([X[k] for k in range(N)])
H_full, _, _ = build_system(X, keys_all, factors_all)
nz = np.abs(H_full) > 1e-9
print(f"[バッチ最適化] 変数 {N} 個 (= {3*N} 次元), 情報行列 H の非ゼロ要素の割合 {nz.mean()*100:.1f} %")

# ---------------------------------------------------------------------------
# 3. 固定ラグ平滑化: 最新 LAG 個だけを最適化し、古い変数は周辺化して捨てる
# ---------------------------------------------------------------------------
LAG = 8
USE_MARGINALIZATION = True  # False にすると、古い変数の情報をただ捨てる
Xw = {0: truth[0].copy()}
priors = []
online = [truth[0].copy()]  # 各時刻で「最新の姿勢」として出力した値
window_H_snapshot = None
for j in range(1, N):
    # 新しい変数をオドメトリで予測して追加
    x = Xw[j - 1]
    z = odom[j - 1][2]
    Xw[j] = np.array([*(x[:2] + rot2(x[2]) @ z[:2]), x[2] + z[2]])
    keys = sorted(Xw)
    odo = [o for o in odom if o[0] in Xw and o[1] in Xw]
    ab = [a for a in absobs if a[0] in Xw]
    fs = make_factors(odo, ab, prior_x0=truth[0] if 0 in Xw else None, linear_priors=priors)
    Xw = gauss_newton(Xw, keys, fs, iters=4)
    online.append(Xw[j].copy())

    if len(keys) > LAG:
        # --- 周辺化: 最古の変数 m を消す ---
        m = keys[0]
        rest = keys[1:]
        # m に繋がる因子だけを集めて線形化する
        odo_m = [o for o in odo if m in (o[0], o[1])]
        ab_m = [a for a in ab if a[0] == m]
        pr_m = [p for p in priors if m in p.keys]
        fs_m = make_factors(odo_m, ab_m, prior_x0=truth[0] if m == 0 else None, linear_priors=pr_m)
        nb = sorted({k for o in odo_m for k in o[:2]} | {k for p in pr_m for k in p.keys} | {m})
        H, g, _ = build_system(Xw, nb, fs_m)
        # シューア補行列: H' = H_rr - H_rm H_mm^{-1} H_mr
        Hmm, Hmr, Hrr = H[:3, :3], H[:3, 3:], H[3:, 3:]
        Hmm_inv = np.linalg.inv(Hmm)
        Hp = Hrr - Hmr.T @ Hmm_inv @ Hmr
        gp = g[3:] - Hmr.T @ Hmm_inv @ g[:3]
        w, V = np.linalg.eigh(Hp)
        keep = w > 1e-8
        S = np.sqrt(w[keep])[:, None] * V[:, keep].T
        r0 = (V[:, keep].T @ gp) / np.sqrt(w[keep])
        priors = [p for p in priors if m not in p.keys]
        if USE_MARGINALIZATION:
            priors.append(LinearPrior(nb[1:], [Xw[k].copy() for k in nb[1:]], S, r0))
        del Xw[m]
        if j == 30:
            fs_now = make_factors([o for o in odom if o[0] in Xw and o[1] in Xw], [a for a in absobs if a[0] in Xw], None, priors)
            window_H_snapshot, _, _ = build_system(Xw, sorted(Xw), fs_now)

online = np.array(online)


def rmse(est, sel):
    return np.sqrt(np.mean(np.sum((est[sel, :2] - truth[sel, :2]) ** 2, axis=1)))


sel_all = np.arange(N)
print("\n位置誤差 RMSE [m]")
print(f"  オドメトリのみ           : {rmse(dead, sel_all):.3f}")
print(f"  バッチ最適化 (全変数)    : {rmse(batch, sel_all):.3f}")
print(f"  固定ラグ ({LAG} 変数) の最新値: {rmse(online, sel_all):.3f}")
print("  → 固定ラグは計算量が一定のまま、バッチに近い精度を出せる")

# ---------------------------------------------------------------------------
plt = setup_matplotlib()
fig = plt.figure(figsize=(12, 4.6))
gs = fig.add_gridspec(1, 3, width_ratios=[1.6, 1, 1])
ax = fig.add_subplot(gs[0])
mp = np.array([HAS_MAP(i) for i in range(N)])
ax.scatter(truth[mp, 0], truth[mp, 1], s=90, color=C_SHADE, zorder=0, label="絶対位置が得られる区間")
ax.plot(truth[:, 0], truth[:, 1], "--", color=C_INK, lw=1.5, label="真値")
ax.plot(dead[:, 0], dead[:, 1], color=C_ORANGE, label="オドメトリのみ")
ax.plot(online[:, 0], online[:, 1], color=C_AQUA, label=f"固定ラグ ({LAG} 変数)")
ax.plot(batch[:, 0], batch[:, 1], color=C_BLUE, label="バッチ最適化")
ax.set_aspect("equal")
ax.set_title("推定した軌跡", color=C_INK)
ax.legend(fontsize=8, loc="best")

ax = fig.add_subplot(gs[1])
ax.spy(np.abs(H_full) > 1e-9, markersize=1.2, color=C_BLUE)
ax.set_title("バッチの H (180×180)\n帯状 = スパース", color=C_INK, fontsize=10)
ax.grid(False)

ax = fig.add_subplot(gs[2])
if window_H_snapshot is not None:
    ax.spy(np.abs(window_H_snapshot) > 1e-9, markersize=4, color=C_BLUE)
    ax.add_patch(plt.Rectangle((-0.5, -0.5), 6, 6, fill=False, ec=C_ORANGE, lw=2))
ax.set_title("ウィンドウの H (周辺化後)\n左上の密なブロック = 周辺化事前分布", color=C_INK, fontsize=10)
ax.grid(False)
fig.tight_layout()
out = fig_path("ex06_factor_graph.png")
fig.savefig(out)
print("\n図を保存しました:", out)

print(
    """
やってみよう
  - LAG を 3, 15, 30 と変えて、精度と計算量 (ウィンドウの大きさ) の関係を見る
  - USE_MARGINALIZATION = False にして、古い変数の情報を「ただ捨てる」とどうなるか試す
    → 地図がない区間でウィンドウの位置を決める手がかりが無くなり、推定が崩れる。
      周辺化が「過去の情報」をウィンドウに受け継いでいることが分かる
"""
)
