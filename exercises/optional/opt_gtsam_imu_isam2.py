"""発展演習 B: GTSAM で IMU 因子 + 姿勢の観測を iSAM2 で解く

必要なもの: pip install gtsam
対応する資料: docs/05_factor_graph.md, docs/06_fixed_lag_smoothing.md

論文の実装は GTSAM (+ 著者の拡張 gtsam_points) で書かれています。
ここでは GTSAM の Python 版で、論文と同じ部品を使った小さなグラフを組みます。

  変数: X(i) 姿勢 Pose3, V(i) 速度, B(i) IMU バイアス
  因子: ImuFactor                ... IMU 事前積分因子 (論文の f^I)
        BetweenFactorPose3       ... 「スキャン間マッチング」の代わり (相対姿勢の観測)
        PriorFactorPose3         ... 「地図マッチング」の代わり (絶対姿勢の観測)。10〜20 秒は地図なし
        BetweenFactorConstantBias ... バイアスがゆっくり変化するというモデル
  解法: iSAM2 (論文と同じインクリメンタル最適化)

本物の点群マッチングの代わりに、真値にノイズを足した姿勢を観測として与えています。
GTSAM の使い方と、因子の組み合わせ方の感覚をつかむのが目的です。

注意: 教材作成環境では gtsam をインストールできなかったため、このスクリプトは
      GTSAM 4.2 の Python API に従って書いていますが、動作確認はできていません。
      エラーが出た場合は GTSAM の python/gtsam/examples (ImuFactorExample.py など) と
      見比べてください。
"""

import numpy as np

try:
    import gtsam
    from gtsam.symbol_shorthand import B, V, X
except ImportError:
    raise SystemExit("gtsam が見つかりません。 pip install gtsam を実行してください。")

rng = np.random.default_rng(0)
G = 9.81
IMU_HZ, FRAME_HZ = 200, 10
DT = 1.0 / IMU_HZ
T_END = 30.0
RADIUS, OMEGA = 8.0, 2 * np.pi / 20.0  # 半径 8 m の円を 20 秒で 1 周
BIAS_A = np.array([0.05, -0.03, 0.02])
BIAS_G = np.radians([0.05, -0.04, 0.1])
SIG_A, SIG_G = 0.02, 0.002


def state(t):
    """円運動する真値: 姿勢 (Rot3), 位置, 速度, ボディ座標の比力, ボディ座標の角速度"""
    yaw = OMEGA * t + np.pi / 2
    R = gtsam.Rot3.Rz(yaw)
    p = np.array([RADIUS * np.cos(OMEGA * t), RADIUS * np.sin(OMEGA * t), 0.0])
    v = np.array([-RADIUS * OMEGA * np.sin(OMEGA * t), RADIUS * OMEGA * np.cos(OMEGA * t), 0.0])
    a_world = np.array([-RADIUS * OMEGA**2 * np.cos(OMEGA * t), -RADIUS * OMEGA**2 * np.sin(OMEGA * t), 0.0])
    # 加速度計は「加速度 - 重力」をボディ座標で測る。重力は (0, 0, -g) なので +g が足される
    f_body = R.matrix().T @ (a_world + np.array([0.0, 0.0, G]))
    w_body = np.array([0.0, 0.0, OMEGA])
    return R, p, v, f_body, w_body


# ---------------------------------------------------------------------------
# IMU 事前積分のパラメータ (z 軸上向き、重力は -z 方向)
# ---------------------------------------------------------------------------
params = gtsam.PreintegrationParams.MakeSharedU(G)
params.setAccelerometerCovariance(np.eye(3) * (SIG_A**2) * DT)
params.setGyroscopeCovariance(np.eye(3) * (SIG_G**2) * DT)
params.setIntegrationCovariance(np.eye(3) * 1e-8)

pose_noise_map = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01, 0.05, 0.05, 0.05]))  # 回転[rad], 並進[m]
pose_noise_scan = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.002, 0.002, 0.002, 0.01, 0.01, 0.01]))
bias_rw = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-3] * 3 + [1e-4] * 3))

isam = gtsam.ISAM2(gtsam.ISAM2Params())
graph = gtsam.NonlinearFactorGraph()
values = gtsam.Values()

# 初期状態
R0, p0, v0, _, _ = state(0.0)
pose0 = gtsam.Pose3(R0, p0)
graph.add(gtsam.PriorFactorPose3(X(0), pose0, gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)))
graph.add(gtsam.PriorFactorVector(V(0), v0, gtsam.noiseModel.Isotropic.Sigma(3, 0.05)))
graph.add(gtsam.PriorFactorConstantBias(B(0), gtsam.imuBias.ConstantBias(), gtsam.noiseModel.Isotropic.Sigma(6, 0.1)))
values.insert(X(0), pose0)
values.insert(V(0), v0)
values.insert(B(0), gtsam.imuBias.ConstantBias())
isam.update(graph, values)
graph, values = gtsam.NonlinearFactorGraph(), gtsam.Values()

pim = gtsam.PreintegratedImuMeasurements(params, gtsam.imuBias.ConstantBias())
prev_nav = gtsam.NavState(pose0, v0)
prev_bias = gtsam.imuBias.ConstantBias()
steps_per_frame = IMU_HZ // FRAME_HZ
n_frames = int(T_END * FRAME_HZ)
errors = []

for i in range(1, n_frames + 1):
    # --- IMU を 1 フレーム分積分 ---
    for k in range(steps_per_frame):
        t = ((i - 1) * steps_per_frame + k + 0.5) * DT
        _, _, _, f_b, w_b = state(t)
        acc = f_b + BIAS_A + rng.normal(scale=SIG_A, size=3)
        gyr = w_b + BIAS_G + rng.normal(scale=SIG_G, size=3)
        pim.integrateMeasurement(acc, gyr, DT)

    t_i = i / FRAME_HZ
    Rt, pt, vt, _, _ = state(t_i)
    true_pose = gtsam.Pose3(Rt, pt)

    # --- 因子を追加 ---
    graph.add(gtsam.ImuFactor(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), pim))
    graph.add(gtsam.BetweenFactorConstantBias(B(i - 1), B(i), gtsam.imuBias.ConstantBias(), bias_rw))

    # スキャン間マッチングの代わり: 真の相対姿勢 + ノイズ
    Rp, pp, _, _, _ = state(t_i - 1.0 / FRAME_HZ)
    rel_true = gtsam.Pose3(Rp, pp).between(true_pose)
    rel_noisy = rel_true.compose(gtsam.Pose3.Expmap(rng.normal(scale=[0.002] * 3 + [0.01] * 3)))
    graph.add(gtsam.BetweenFactorPose3(X(i - 1), X(i), rel_noisy, pose_noise_scan))

    # 地図マッチングの代わり: 10〜20 秒の間は地図が無いので観測なし
    has_map = not (10.0 <= t_i < 20.0)
    if has_map:
        meas = true_pose.compose(gtsam.Pose3.Expmap(rng.normal(scale=[0.01] * 3 + [0.05] * 3)))
        graph.add(gtsam.PriorFactorPose3(X(i), meas, pose_noise_map))

    # --- IMU で予測した値を初期値として入れ、iSAM2 で更新 ---
    pred = pim.predict(prev_nav, prev_bias)
    values.insert(X(i), pred.pose())
    values.insert(V(i), pred.velocity())
    values.insert(B(i), prev_bias)
    isam.update(graph, values)
    est = isam.calculateEstimate()
    graph, values = gtsam.NonlinearFactorGraph(), gtsam.Values()

    prev_nav = gtsam.NavState(est.atPose3(X(i)), est.atVector(V(i)))
    prev_bias = est.atConstantBias(B(i))
    pim.resetIntegrationAndSetBias(prev_bias)

    err = np.linalg.norm(est.atPose3(X(i)).translation() - pt)
    errors.append(err)
    if i % 20 == 0:
        print(f"t = {t_i:5.1f} s  地図{'あり' if has_map else 'なし'}  位置誤差 {err*100:6.2f} cm")

print("\n推定したバイアス (最後):", prev_bias)
print("真のバイアス: acc", BIAS_A, " gyro", BIAS_G)
print(
    """
やってみよう
  - BetweenFactorPose3 (スキャン間マッチングの代わり) を入れないと、地図なし区間で何が起こる?
  - ImuFactor を入れないと? (BetweenFactorPose3 だけで地図なし区間をつなぐ)
  - iSAM2 の代わりに gtsam.LevenbergMarquardtOptimizer で全体を一括最適化して比べる
"""
)
