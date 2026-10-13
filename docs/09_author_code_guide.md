# 09. 著者のコード案内 (koide3 の GitHub)

論文著者の小出 健司さんは、研究成果の多くをオープンソースで公開しています (GitHub: [koide3](https://github.com/koide3))。
この章では、論文の構成要素が **どのリポジトリの、どのファイルにあるか** を案内します。

> **注意**
> 調べた範囲では、この論文 (既知地図上の自己位置推定) の実装そのものは公開リポジトリには見当たりませんでした。
> ただし、論文のベースになっている LiDAR-IMU オドメトリは **GLIM** の中で公開されており、
> 論文の因子 (VGICP 因子、IMU 因子、固定ラグ平滑化) は同じ部品で作られています。
> ファイル名・行の内容は 2026 年 10 月時点の `master` ブランチで確認したものです。

---

## 1. リポジトリの地図

| リポジトリ | 役割 | 論文との関係 |
| --- | --- | --- |
| [glim](https://github.com/koide3/glim) | 点群ベースの 3D 自己位置推定・地図作成フレームワーク (SLAM) | 論文のベース [5] のオドメトリを含む。最重要 |
| [gtsam_points](https://github.com/koide3/gtsam_points) | 点群 SLAM 用の GTSAM 因子と最適化器 (旧名 gtsam_ext) | VGICP 因子 (GPU 版含む)、固定ラグ平滑化の拡張 |
| [small_gicp](https://github.com/koide3/small_gicp) | 高速な点群位置合わせライブラリ (C++/Python) | ICP/GICP/VGICP を手軽に試せる |
| [fast_gicp](https://github.com/koide3/fast_gicp) | GICP 系アルゴリズムの高速実装 (small_gicp の前身) | VGICP の最初の実装 |
| [hdl_localization](https://github.com/koide3/hdl_localization) | UKF + NDT による既知地図上の自己位置推定 (ROS1) | 論文の比較手法の 1 つ |
| [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) | ポーズグラフ SLAM (ROS1) | 地図作成の古典的な構成を学べる |
| [glim_ext](https://github.com/koide3/glim_ext) | GLIM の拡張モジュール集 (ループ検出、GNSS など) | GLIM にコールバックで機能を足す例 |
| [iridescence](https://github.com/koide3/iridescence) | 3D 可視化ライブラリ | GLIM のビューア |

## 2. GLIM の全体構成

GLIM は次のモジュールが別々のスレッドで動きます (GLIM 論文, RAS 2024)。

1. **前処理** (`src/glim/preprocess/`): ダウンサンプリング、近傍探索、共分散推定
2. **オドメトリ推定** (`src/glim/odometry/`): 固定ラグ平滑化による LiDAR-IMU オドメトリ ← **論文のベース**
3. **サブマッピング** (`src/glim/mapping/sub_mapping*`): 数秒分のフレームをまとめたサブマップを作る
4. **グローバルマッピング** (`src/glim/mapping/global_mapping*`): サブマップ間のマッチングで全体の地図を最適化

自己位置推定の論文に関係が深いのは 1 と 2 です。

## 3. 論文の要素 → GLIM / gtsam_points のコード

### 3.1 状態と固定ラグ平滑化

`src/glim/odometry/odometry_estimation_imu.cpp`

- 平滑化器の生成: `smoother.reset(new FixedLagSmootherExt(params->smoother_lag, isam2_params));`
  - `FixedLagSmootherExt` の実体は `gtsam_points::IncrementalFixedLagSmootherExtWithFallback` (ヘッダ `odometry_estimation_imu.hpp`)
- 状態のキー: `X(i)` 姿勢, `V(i)` 速度, `B(i)` IMU バイアス ([04 章](04_imu_and_preintegration.md) の $\mathbf{x}_i = [\mathbf{T}_i, \mathbf{v}_i, \mathbf{b}_i]$)
- 最初の状態の固定: `LinearDampingFactor` (ゲージ自由度 = 座標系の原点の任意性を固定する)

### 3.2 IMU 因子 $f^I(\mathbf{x}_{i-1}, \mathbf{x}_i)$

同じく `odometry_estimation_imu.cpp` の `insert_frame()` の中で、

- `imu_integration->integrate_imu(...)`: 前フレームから今のフレームまでの IMU を事前積分
- `imu_integration->integrated_measurements().predict(...)`: 事前積分で新しい状態を **予測** (最適化の初期値)
- `gtsam::ImuFactor(X(last), V(last), X(current), V(current), B(last), ...)`: **IMU 因子**
- `gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(last), B(current), ...)`: バイアスのランダムウォーク因子

### 3.3 スキャンマッチング因子 $f^P(\mathbf{x}_i, \mathbf{x}_j)$

`src/glim/odometry/odometry_estimation_gpu.cpp` の `create_factors()`

```cpp
// There must be at least one factor between consecutive frames
for (int target = current - params->full_connection_window_size; target < current; target++) {
  ...
  create_binary_factor(factors, X(target), X(current), frames[target], frames[current]);
}
```

- `full_connection_window_size` (設定ファイルの値 2): 最新フレームを **直前の何フレームとつなぐか**。
  論文の $\sum_{j=i-2}^{i-1}$ に対応
- `create_binary_factor` の中身は `gtsam_points::IntegratedVGICPFactorGPU(target_key, source_key, voxelmap, source->frame, ...)`。
  ターゲットのボクセル地図の **解像度レベルごとに 1 つ** 因子を作る (マルチ解像度, [03 章](03_point_cloud_registration.md))
- 続くループで **キーフレーム** とも因子を作る。キーフレームがウィンドウ外に出ていたら
  `create_unary_factor` (キーフレームの姿勢を固定した単項因子) になる ([06 章](06_fixed_lag_smoothing.md) 4 節)

> **地図マッチング因子 $f^P(\mathbf{x}_i, \mathcal{M})$ はどこ?**
> GLIM のオドメトリには地図因子はありませんが、`create_unary_factor` が使っている
> 「姿勢を固定したターゲットとの `IntegratedVGICPFactorGPU`」は、ターゲットを地図点群にすれば
> そのまま地図マッチング因子になる構造です (ターゲットの姿勢 = 単位行列)。
> 論文の実装がこの通りかは公開コードからは確認できませんが、理解の助けになるはずです。

### 3.4 VGICP 因子の中身

`gtsam_points` の

- `include/gtsam_points/factors/integrated_matching_cost_factor.hpp`:
  点群マッチング因子の共通の基底クラス。二項 (2 つの姿勢) と単項 (固定したターゲット姿勢) のコンストラクタがあり、
  `update_correspondences()` (対応の更新) と `evaluate()` (誤差・$\mathbf{H}$・$\mathbf{b}$ の計算) を派生クラスが実装する
- `include/gtsam_points/factors/integrated_vgicp_factor.hpp`: CPU 版の VGICP 因子
- `include/gtsam_points/factors/integrated_vgicp_factor_gpu.hpp`: **GPU 版の VGICP 因子** (論文で使用)
- `include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`, `gaussian_voxelmap_gpu.hpp`: ガウス分布ボクセル地図
- `include/gtsam_points/types/point_cloud_cpu.hpp`, `point_cloud_gpu.hpp`: 点・共分散などを持つ点群型

演習 4・7 の `toy2d.py` の `GaussianVoxelMap` と `vgicp_terms()` が、これらを 2D・numpy で書き直したものに相当します。

### 3.5 キーフレームの管理

`odometry_estimation_gpu.cpp` の `update_keyframes_overlap()` など。
新しいフレームと既存キーフレームの **重なり (overlap)** を GPU で計算し、重なりが小さくなったら新しいキーフレームを追加、
不要になったものを外す、という管理をしています (ICRA 2022 論文の方法)。

### 3.6 設定ファイル

`config/config_odometry_gpu.json` の主なパラメータ ([GLIM のパラメータ説明](https://koide3.github.io/glim/parameters.html) も参照)

| パラメータ | 設定ファイルの値 | 意味 | 教材の章 |
| --- | --- | --- | --- |
| smoother_lag | 5.0 | 固定ラグ平滑化のウィンドウ長 [s] | 06 |
| full_connection_window_size | 2 | 直前の何フレームとマッチングするか | 06, 08 |
| voxel_resolution | 0.25 | VGICP の基本ボクセルサイズ [m] | 03 |
| voxelmap_levels | 2 | マルチ解像度の段数 | 03 |
| max_num_keyframes | 15 | 保持するキーフレーム数 | 06 |
| keyframe_update_strategy | OVERLAP | キーフレームの選び方 | 06 |
| isam2_relinearize_thresh | 0.1 | iSAM2 の再線形化の閾値 | 05 |

## 4. コードを読む順番のおすすめ

1. **small_gicp の Python 版** で、ICP/GICP/VGICP を実データ (自分の LiDAR の点群など) で試す
   → `exercises/optional/opt_small_gicp_3d.py`
2. **GLIM を動かす**: Docker イメージ (`koide3/glim_ros2`) と公式のデモデータで、オドメトリとマッピングを体験する
   ([Getting started](https://koide3.github.io/glim/quickstart.html))
3. **odometry_estimation_imu.cpp の insert_frame()** を、この章の 3.1, 3.2 を見ながら読む
4. **odometry_estimation_gpu.cpp の create_factors()** を 3.3 を見ながら読む
5. **gtsam_points の integrated_vgicp_factor** を、[03 章](03_point_cloud_registration.md) の式と見比べながら読む
6. (挑戦) GLIM のオドメトリに「地図マッチング因子」を足すとしたら、どこに何を書けばよいか考えてみる

> ROS 2 と Gazebo を使っているなら、Gazebo で作った環境の点群を地図にして、シミュレーション上で
> 地図の一部を消したときの振る舞いを試す、というのも良い発展課題です。

---

前へ: [08 論文精読ガイド](08_paper_walkthrough.md) ｜ 次へ: [10 用語集](10_glossary.md)
