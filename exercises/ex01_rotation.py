"""演習 1: 回転行列・Lie 群 (SO(3) / SE(3)) に慣れる

対応する資料: docs/01_coordinates_and_rotation.md

この演習で確かめること
  1. 回転行列は R^T R = I, det(R) = 1 を満たす
  2. 指数写像 Exp と対数写像 Log は互いに逆
  3. 回転の合成は順番を入れ替えると結果が変わる
  4. 「微小回転 δ を右から掛ける」摂動で、回転のヤコビアンが簡単に書ける
  5. 相対姿勢 T_ij = T_i^{-1} T_j は論文のスキャンマッチング因子で使われる
  6. 右摂動 R Exp(δ) の δ はセンサ座標系、左摂動 Exp(δ) R の δ は世界座標系の回転

実行: python exercises/ex01_rotation.py
"""

import numpy as np

from common import (
    C_BLUE,
    C_INK,
    C_ORANGE,
    fig_path,
    hat,
    se3,
    se3_inv,
    setup_matplotlib,
    so3_exp,
    so3_log,
    transform,
)

np.set_printoptions(precision=4, suppress=True)
rng = np.random.default_rng(0)


def section(title):
    print("\n" + "=" * 70 + f"\n{title}\n" + "=" * 70)


# ---------------------------------------------------------------------------
section("1. 回転行列の性質")
w = np.array([0.3, -0.2, 0.9])  # 回転ベクトル (回転軸 × 回転角[rad])
R = so3_exp(w)
print("R =\n", R)
print("R^T R =\n", R.T @ R, "  <- 単位行列になる")
print("det(R) =", np.linalg.det(R), "  <- 1 になる")

# ---------------------------------------------------------------------------
section("2. Exp と Log は互いに逆")
for _ in range(3):
    w = rng.normal(size=3)
    w_back = so3_log(so3_exp(w))
    # |w| が π を超えると Log は別の (等価な) ベクトルを返すので、回転行列で比較する
    err = np.linalg.norm(so3_exp(w_back) - so3_exp(w))
    print(f"w = {w},  Log(Exp(w)) = {w_back},  回転行列の差 = {err:.2e}")

# ---------------------------------------------------------------------------
section("3. 回転の合成は非可換 (順番が大事)")
Rx = so3_exp(np.array([np.pi / 2, 0, 0]))  # x 軸まわりに 90 度
Rz = so3_exp(np.array([0, 0, np.pi / 2]))  # z 軸まわりに 90 度
p = np.array([1.0, 0.0, 0.0])
print("Rx @ Rz @ p =", Rx @ Rz @ p)
print("Rz @ Rx @ p =", Rz @ Rx @ p, "  <- 違う結果になる")

# ---------------------------------------------------------------------------
section("4. 摂動 (perturbation) によるヤコビアン")
# f(R) = R p を考える。R に右から微小回転 Exp(δ) を掛けたとき
#   R Exp(δ) p ≈ R (I + [δ]×) p = R p - R [p]× δ
# なので、∂f/∂δ = -R [p]×  (3x3 行列) となる。
R = so3_exp(rng.normal(size=3))
p = rng.normal(size=3)
J_analytic = -R @ hat(p)

eps = 1e-6
J_numeric = np.zeros((3, 3))
for k in range(3):
    d = np.zeros(3)
    d[k] = eps
    J_numeric[:, k] = (R @ so3_exp(d) @ p - R @ p) / eps
print("解析的ヤコビアン -R[p]x =\n", J_analytic)
print("数値微分で求めたヤコビアン =\n", J_numeric)
print("差の最大値 =", np.abs(J_analytic - J_numeric).max())

# ---------------------------------------------------------------------------
section("5. SE(3): 同次変換と相対姿勢")
# 世界座標系でのセンサ姿勢 T_i, T_j
T_i = se3(so3_exp(np.array([0, 0, 0.2])), np.array([1.0, 0.0, 0.0]))
T_j = se3(so3_exp(np.array([0, 0, 0.5])), np.array([2.0, 0.5, 0.0]))
# フレーム j で観測した点 (センサ座標系)
pts_j = np.array([[3.0, 0.0, 0.0], [3.0, 1.0, 0.0], [3.0, -1.0, 0.5]])
# 相対姿勢: フレーム j の座標をフレーム i の座標に変換する行列
T_ij = se3_inv(T_i) @ T_j
pts_in_i = transform(T_ij, pts_j)
# 世界座標を経由しても同じ結果になることを確認
pts_in_i_2 = transform(se3_inv(T_i), transform(T_j, pts_j))
print("T_ij =\n", T_ij)
print("T_ij で変換した点 =\n", pts_in_i)
print("世界座標経由で変換した点 =\n", pts_in_i_2)
print("差 =", np.abs(pts_in_i - pts_in_i_2).max())
print("→ 論文の GICP 因子は、この T_ij を介して 2 つのフレームの点群を比べています")

# ---------------------------------------------------------------------------
section("6. 図: 1 次近似 Exp(δ) ≈ I + [δ]x はどのくらい正確か")
plt = setup_matplotlib()
angles = np.logspace(-3, 0.3, 50)  # 0.001 rad 〜 約 2 rad
axis = np.array([1.0, 2.0, 2.0]) / 3.0
err1 = [np.linalg.norm(so3_exp(a * axis) - (np.eye(3) + hat(a * axis))) for a in angles]
err0 = [np.linalg.norm(so3_exp(a * axis) - np.eye(3)) for a in angles]

fig, ax = plt.subplots(figsize=(6.4, 4.0))
ax.loglog(np.degrees(angles), err0, color=C_ORANGE, label="0 次近似  Exp(δ) ≈ I")
ax.loglog(np.degrees(angles), err1, color=C_BLUE, label="1 次近似  Exp(δ) ≈ I + [δ]×")
ax.set_xlabel("回転角 |δ| [deg]")
ax.set_ylabel("近似誤差 (フロベニウスノルム)")
ax.set_title("微小回転なら 1 次近似の誤差は角度の 2 乗で小さくなる", color=C_INK)
ax.legend()
out = fig_path("ex01_exp_approx.png")
fig.savefig(out)
print("図を保存しました:", out)

# ---------------------------------------------------------------------------
section("7. 右摂動と左摂動: 同じ δ でも回る軸が違う")
# ロボットが地図上で左に 90° 向いている (yaw = 90°)。センサ座標は x = 前, y = 左, z = 上
R = so3_exp(np.array([0.0, 0.0, np.pi / 2]))
print("R (yaw = 90°) =\n", R)
print("R の各列 = センサの各軸を世界座標で表したもの")
print("  センサの前 (x) →", R[:, 0], " (世界の +y)")
print("  センサの左 (y) →", R[:, 1], " (世界の -x)")

d = np.array([0.1, 0.0, 0.0])  # 「x 軸まわりに 0.1 rad」
R_right = R @ so3_exp(d)  # 右摂動: センサの x 軸 (= 前) まわり
R_left = so3_exp(d) @ R  # 左摂動: 世界の x 軸 (= ロボットの右) まわり
print("\nδ = (0.1, 0, 0) を加えたあとの、前方向と左方向 (世界座標)")
print("  右摂動 R Exp(δ): 前 =", R_right[:, 0], " 左 =", R_right[:, 1])
print("     → 前はそのまま、左側の z が +0.1: 前後の軸まわりに傾く (ロール)")
print("  左摂動 Exp(δ) R: 前 =", R_left[:, 0], " 左 =", R_left[:, 1])
print("     → 左はそのまま、前の z が +0.1: 機首が上がる (ピッチ)")

# 2 つの関係: センサ座標の回転軸 δ_s を R で世界座標に直すと、左摂動の δ_w になる
d_w = R @ d
print("\nR Exp(δ_s) と Exp(R δ_s) R の差 =", np.abs(R @ so3_exp(d) - so3_exp(d_w) @ R).max(), " (≒ 0)")
print("  δ_s =", d, " を世界座標に直すと δ_w = R δ_s =", d_w)

# 左摂動のヤコビアンは -[R p]x になる (右摂動の -R[p]x と形が違う)
p = np.array([1.0, 2.0, 3.0])
eps = 1e-6
J_left_num = np.zeros((3, 3))
for k in range(3):
    e = np.zeros(3)
    e[k] = eps
    J_left_num[:, k] = (so3_exp(e) @ R @ p - R @ p) / eps
print("\n左摂動のヤコビアン: 解析 -[R p]x と数値微分の差 =", np.abs(-hat(R @ p) - J_left_num).max())
print("右摂動のヤコビアン -R[p]x =\n", -R @ hat(p))
print("左摂動のヤコビアン -[R p]x =\n", -hat(R @ p))
print("→ 同じ関数でも、摂動の流儀でヤコビアンの形が変わる")

print(
    """
やってみよう
  - 4. の p や R を変えても、解析ヤコビアンと数値ヤコビアンが一致することを確認する
  - 左から摂動を掛ける Exp(δ) R p のヤコビアンを自分で導出する (答え: -[R p]x。7. で数値確認済み)
  - 7. の yaw を 0° にすると、右摂動と左摂動が一致することを確かめる
"""
)
