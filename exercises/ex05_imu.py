"""演習 5: IMU の積分・ドリフト・事前積分 (preintegration)

対応する資料: docs/04_imu_and_preintegration.md

IMU (慣性計測装置) は
  - ジャイロ: 角速度 ω (センサ座標系)
  - 加速度計: 比力 a = R^T (加速度 - 重力)  (センサ座標系、重力の分も含む!)
を 100〜1000 Hz で出力します。ただし必ず「バイアス」と「ノイズ」が乗ります。

この演習で確かめること
  1. IMU だけで積分すると、位置誤差は時間とともに急激に増える (ドリフト)
     - 加速度バイアス → 誤差はおよそ t^2 で増える
     - ジャイロバイアス → 姿勢がずれて重力が漏れ込み、誤差はおよそ t^3 で増える
  2. 事前積分 (ΔR, Δv, Δp) は始点の状態 (姿勢・速度) に依存しない
     → 最適化中に状態が変わっても積分をやり直さなくてよい
  3. バイアスが少し変わったときは、ヤコビアンを使った 1 次補正で積分のやり直しを省ける

実行: python exercises/ex05_imu.py
"""

import numpy as np

from common import (
    C_AQUA,
    C_BLUE,
    C_INK,
    C_MUTED,
    C_ORANGE,
    fig_path,
    hat,
    setup_matplotlib,
    so3_exp,
    so3_log,
    so3_right_jacobian,
)

np.set_printoptions(precision=5, suppress=True)
rng = np.random.default_rng(5)
G = np.array([0.0, 0.0, -9.81])  # 重力加速度 (世界座標, z 上向き)

# ---------------------------------------------------------------------------
# 真の軌跡を作る (ゆるやかに上下しながら楕円を描いて走る)
# ---------------------------------------------------------------------------
IMU_HZ = 200
DT = 1.0 / IMU_HZ
T_END = 30.0
t = np.arange(0.0, T_END + DT, DT)
w0 = 2 * np.pi / 20.0


def pos(tt):
    return np.stack([10 * np.cos(w0 * tt), 6 * np.sin(w0 * tt), 0.5 * np.sin(2 * w0 * tt)], axis=-1)


def vel(tt):
    return np.stack([-10 * w0 * np.sin(w0 * tt), 6 * w0 * np.cos(w0 * tt), w0 * np.cos(2 * w0 * tt)], axis=-1)


def acc(tt):
    return np.stack(
        [-10 * w0**2 * np.cos(w0 * tt), -6 * w0**2 * np.sin(w0 * tt), -2 * w0**2 * np.sin(2 * w0 * tt)], axis=-1
    )


def rot(tt):
    v = vel(tt)
    yaw = np.arctan2(v[1], v[0])
    pitch = 0.08 * np.sin(0.7 * tt)
    roll = 0.12 * np.sin(1.1 * tt)
    return so3_exp(np.array([0, 0, yaw])) @ so3_exp(np.array([0, pitch, 0])) @ so3_exp(np.array([roll, 0, 0]))


R_true = np.array([rot(tt) for tt in t])
p_true, v_true = pos(t), vel(t)

# 理想的な IMU の値。
#   ジャイロ: 区間 [t_k, t_k+1] の回転 R_k^T R_k+1 をちょうど再現する角速度
#   加速度計: 区間中央の加速度を、区間始点の姿勢 R_k で測ったとみなした比力
#   (下の積分方式と合わせておくと、ノイズ・バイアスが無いときの積分誤差がほぼ 0 になる)
omega_ideal = np.array([so3_log(R_true[k].T @ R_true[k + 1]) / DT for k in range(len(t) - 1)])
tm = t[:-1] + DT / 2
acc_ideal = np.array([R_true[k].T @ (acc(tm[k]) - G) for k in range(len(t) - 1)])

SIGMA_G = 0.002  # ジャイロのノイズ [rad/s] (1 サンプルあたり)
SIGMA_A = 0.02  # 加速度計のノイズ [m/s^2]


def make_imu(bias_a=np.zeros(3), bias_g=np.zeros(3), noise=True):
    n = len(t) - 1
    ng = rng.normal(scale=SIGMA_G, size=(n, 3)) if noise else 0.0
    na = rng.normal(scale=SIGMA_A, size=(n, 3)) if noise else 0.0
    return omega_ideal + bias_g + ng, acc_ideal + bias_a + na


def dead_reckoning(omega_m, acc_m, bias_a=np.zeros(3), bias_g=np.zeros(3)):
    """真の初期状態から IMU だけで積分する"""
    R, v, p = R_true[0].copy(), v_true[0].copy(), p_true[0].copy()
    P = [p.copy()]
    for k in range(len(omega_m)):
        a_w = G + R @ (acc_m[k] - bias_a)
        p = p + v * DT + 0.5 * a_w * DT**2
        v = v + a_w * DT
        R = R @ so3_exp((omega_m[k] - bias_g) * DT)
        P.append(p.copy())
    return np.array(P)


# ---------------------------------------------------------------------------
print("[1] IMU だけで 30 秒積分したときの位置誤差")
cases = {
    "理想的な IMU (ノイズ・バイアスなし)": dict(noise=False),
    "ノイズのみ": dict(),
    "加速度バイアス 0.05 m/s^2": dict(bias_a=np.array([0.05, 0.0, 0.0])),
    "ジャイロバイアス 0.2 deg/s": dict(bias_g=np.radians([0.2, 0.0, 0.0])),
}
errors = {}
for name, kw in cases.items():
    om, am = make_imu(**kw)
    P = dead_reckoning(om, am)
    errors[name] = np.linalg.norm(P - p_true, axis=1)
    e = errors[name]
    print(f"  {name:30s}: 1 秒後 {e[IMU_HZ]:8.3f} m, 10 秒後 {e[10*IMU_HZ]:8.2f} m, 30 秒後 {e[-1]:8.1f} m")
print("  → 加速度バイアス b: 誤差 ≈ ½ b t²  (1 秒で 0.025 m, 10 秒で 2.5 m の見積もり。")
print("     ロボットの向きが変わるとバイアスの向きも回るので、長時間では一部打ち消し合う)")
print("  → ジャイロバイアス b_g: 傾き b_g t が生まれ、重力 9.81 m/s² が横向きに漏れる。")
print("     誤差 ≈ g b_g t³ / 6  (10 秒で約 5.7 m の見積もり) と、t の 3 乗で増える")

# ---------------------------------------------------------------------------
print("\n[2] 事前積分 (preintegration)")


def preintegrate(omega_m, acc_m, ba, bg):
    """区間内の IMU 計測から ΔR, Δv, Δp と、バイアスに関するヤコビアンを計算する"""
    dR, dv, dp = np.eye(3), np.zeros(3), np.zeros(3)
    J_R_bg = np.zeros((3, 3))
    J_v_ba, J_v_bg = np.zeros((3, 3)), np.zeros((3, 3))
    J_p_ba, J_p_bg = np.zeros((3, 3)), np.zeros((3, 3))
    for w, a in zip(omega_m, acc_m):
        wh, ah = w - bg, a - ba
        # 位置 → 速度 → 回転 の順に更新 (古い値を使うため)
        dp = dp + dv * DT + 0.5 * dR @ ah * DT**2
        J_p_ba = J_p_ba + J_v_ba * DT - 0.5 * dR * DT**2
        J_p_bg = J_p_bg + J_v_bg * DT - 0.5 * dR @ hat(ah) @ J_R_bg * DT**2
        dv = dv + dR @ ah * DT
        J_v_ba = J_v_ba - dR * DT
        J_v_bg = J_v_bg - dR @ hat(ah) @ J_R_bg * DT
        inc = so3_exp(wh * DT)
        J_R_bg = inc.T @ J_R_bg - so3_right_jacobian(wh * DT) * DT
        dR = dR @ inc
    jac = dict(R_bg=J_R_bg, v_ba=J_v_ba, v_bg=J_v_bg, p_ba=J_p_ba, p_bg=J_p_bg)
    return dR, dv, dp, jac


def predict(Ri, vi, pi, dR, dv, dp, dt):
    """事前積分値から終点の状態を予測する (論文の IMU 因子の誤差はこれと推定値の差)"""
    Rj = Ri @ dR
    vj = vi + G * dt + Ri @ dv
    pj = pi + vi * dt + 0.5 * G * dt**2 + Ri @ dp
    return Rj, vj, pj


bias_a_true = np.array([0.04, -0.02, 0.03])
bias_g_true = np.radians([0.1, -0.05, 0.08])
om, am = make_imu(bias_a=bias_a_true, bias_g=bias_g_true)
i, j = 4 * IMU_HZ, int(4.5 * IMU_HZ)  # 0.5 秒の区間 (LiDAR フレーム 5 枚分くらい)
seg_w, seg_a = om[i:j], am[i:j]
dT = (j - i) * DT

dR, dv, dp, jac = preintegrate(seg_w, seg_a, bias_a_true, bias_g_true)
Rj, vj, pj = predict(R_true[i], v_true[i], p_true[i], dR, dv, dp, dT)
print(f"  区間 {dT:.1f} 秒、IMU {j-i} サンプル")
print(f"  真の状態との差: 位置 {np.linalg.norm(pj - p_true[j])*1000:.2f} mm, "
      f"姿勢 {np.degrees(np.linalg.norm(so3_log(Rj.T @ R_true[j]))):.4f} deg  (ノイズによるもの)")

# 2-a: 始点の状態を変えても ΔR, Δv, Δp はそのまま使える
Ri2 = R_true[i] @ so3_exp(np.array([0.05, -0.02, 0.3]))
vi2 = v_true[i] + np.array([0.3, -0.1, 0.05])
pi2 = p_true[i] + np.array([1.0, 2.0, 0.0])
Rj_a, vj_a, pj_a = predict(Ri2, vi2, pi2, dR, dv, dp, dT)  # 事前積分を再利用
# 比較: 新しい始点から愚直に積分し直す
R_, v_, p_ = Ri2.copy(), vi2.copy(), pi2.copy()
for w, a in zip(seg_w, seg_a):
    aw = G + R_ @ (a - bias_a_true)
    p_ = p_ + v_ * DT + 0.5 * aw * DT**2
    v_ = v_ + aw * DT
    R_ = R_ @ so3_exp((w - bias_g_true) * DT)
print("  始点の状態を変えて、事前積分の再利用 vs 積分やり直し の差:"
      f" 位置 {np.linalg.norm(pj_a - p_):.2e} m (≒ 0 → 再積分は不要)")

# 2-b: バイアスが変わったときの 1 次補正
print("\n  バイアス推定値が δb だけ変わったとき (最適化の途中で起こる)")
ba0, bg0 = bias_a_true + np.array([0.03, 0.02, -0.02]), bias_g_true + np.radians([0.05, 0.05, -0.05])
dR0, dv0, dp0, jac0 = preintegrate(seg_w, seg_a, ba0, bg0)  # 古いバイアスで積分済み
dba, dbg = bias_a_true - ba0, bias_g_true - bg0  # バイアスの更新量
dR_c = dR0 @ so3_exp(jac0["R_bg"] @ dbg)
dv_c = dv0 + jac0["v_ba"] @ dba + jac0["v_bg"] @ dbg
dp_c = dp0 + jac0["p_ba"] @ dba + jac0["p_bg"] @ dbg
print(f"    補正なし        : Δp の誤差 {np.linalg.norm(dp0 - dp)*1000:7.3f} mm, Δv の誤差 {np.linalg.norm(dv0 - dv)*1000:7.3f} mm/s")
print(f"    1 次補正 (J δb) : Δp の誤差 {np.linalg.norm(dp_c - dp)*1000:7.3f} mm, Δv の誤差 {np.linalg.norm(dv_c - dv)*1000:7.3f} mm/s")
print(f"    ΔR の誤差: 補正なし {np.degrees(np.linalg.norm(so3_log(dR0.T @ dR))):.4f} deg → 補正あり "
      f"{np.degrees(np.linalg.norm(so3_log(dR_c.T @ dR))):.6f} deg")
print("    → 積分をやり直さなくても、ヤコビアンを掛け算するだけでほぼ同じ値になる")

# ---------------------------------------------------------------------------
plt = setup_matplotlib()
fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.6))
ax = axes[0]
ax.plot(p_true[:, 0], p_true[:, 1], color=C_INK, lw=2.5, label="真の軌跡")
for name, c in [("ノイズのみ", C_AQUA), ("加速度バイアス 0.05 m/s^2", C_ORANGE), ("ジャイロバイアス 0.2 deg/s", C_BLUE)]:
    om_, am_ = make_imu(**cases[name])
    P = dead_reckoning(om_, am_)
    k = int(15 * IMU_HZ)
    ax.plot(P[:k, 0], P[:k, 1], color=c, label=name + " (15 秒)")
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("IMU だけで積分した軌跡 (最初の 15 秒)", color=C_INK)
ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.15), ncol=2)

ax = axes[1]
tt = t[1:]
for name, c in [
    ("ノイズのみ", C_AQUA),
    ("加速度バイアス 0.05 m/s^2", C_ORANGE),
    ("ジャイロバイアス 0.2 deg/s", C_BLUE),
]:
    ax.loglog(tt, errors[name][1:], color=c, label=name)
ax.loglog(tt, 0.5 * 0.05 * tt**2, "--", color=C_MUTED, lw=1.2, label="目安 ½ b t²")
ax.loglog(tt, 9.81 * np.radians(0.2) * tt**3 / 6, ":", color=C_MUTED, lw=1.4, label="目安 g b_g t³ / 6")
ax.set_xlabel("時間 [s] (対数)")
ax.set_ylabel("位置誤差 [m] (対数)")
ax.set_ylim(1e-4, 1e3)
ax.set_xlim(0.05, 35)
ax.set_title("誤差は時間とともに急増する", color=C_INK)
ax.legend(fontsize=8)
fig.tight_layout()
out = fig_path("ex05_imu_drift.png")
fig.savefig(out)
print("\n図を保存しました:", out)

print(
    """
やってみよう
  - SIGMA_A, SIGMA_G を 10 倍にして、ノイズだけでもどれくらいドリフトするか見る
  - 区間を 0.5 秒から 2 秒に伸ばすと、1 次補正の精度はどう変わる?
  - バイアスの変化量 (ba0, bg0 のずれ) を大きくして、1 次補正が効かなくなる様子を見る
"""
)
