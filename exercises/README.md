# 演習 (exercises)

すべての演習は **numpy / scipy / matplotlib だけ** で動きます。ROS・GPU・GTSAM は不要です。
実行すると、結果がターミナルに表示され、図が `docs/figures/` に保存されます。

```bash
# リポジトリのトップで
python exercises/check_env.py                      # 環境の確認
python exercises/ex01_rotation.py                  # 演習 1
...
make exercises                                     # 全部まとめて実行 (Makefile)
```

| ファイル | 内容 | 対応する章 | 実行時間 |
| --- | --- | --- | --- |
| `ex01_rotation.py` | 回転行列, Exp/Log, 摂動ヤコビアン, 相対姿勢 | 01 | 数秒 |
| `ex02a_gaussian.py` | 誤差ベクトル, 正規分布の 1/2, 共分散行列, マハラノビス距離, 重み付き平均 | 02 | 数秒 |
| `ex02b_least_squares.py` | ガウス・ニュートン法, LM 法, 局所解, 重み付け, H⁻¹ による不確かさ | 02 | 数秒 |
| `ex03a_matching_methods.py` | 前処理, 点対点・点対面 ICP, NDT, GICP, VGICP を同じ場面で図にする, コストの形 | 03 | 約 10 秒 |
| `ex03b_icp_2d.py` | 点対点 ICP, 点対線 ICP, GICP を反復させて収束の速さと収束域を比較 | 03 | 約 10 秒 |
| `ex03c_f3slam_multires.py` | F3-SLAM: ピラミッド型ボクセル地図 ((14) 式)、平面判定、点ごとのボクセル選択、NDT との成功率比較 | 03 | 約 30 秒 |
| `ex04_vgicp_2d.py` | ボクセル地図, 対応探索の速さ, マルチ解像度 | 03 | 約 5 秒 |
| `ex05_imu.py` | IMU のドリフト, 事前積分, バイアスの 1 次補正 | 04 | 数秒 |
| `ex06_factor_graph.py` | ファクターグラフ, スパース性, 周辺化, 固定ラグ平滑化 | 05, 06 | 数秒 |
| `ex07_sliding_window_localization.py` | **論文の手法の 2D 再現** (VGICP + IMU + 地図 + 周辺化) | 06, 07, 08 | 1〜3 分 |

共通モジュール

- `common.py`: 回転の関数 (hat, Exp, Log など)、図の保存先、日本語フォントの設定
- `toy2d.py`: 2D の環境・LiDAR シミュレーション・ガウス分布ボクセル地図・VGICP の誤差計算

## 発展演習 (optional)

追加のパッケージが必要です。教材作成環境ではこれらのパッケージをインストールできなかったため、
**公式ドキュメントの API に従って書いていますが、動作確認はできていません**。
動かない場合は、各ライブラリの README やサンプルと見比べてみてください (それも良い練習になります)。

| ファイル | 必要なもの | 内容 |
| --- | --- | --- |
| `optional/opt_small_gicp_3d.py` | `pip install small_gicp` | 著者の small_gicp で 3D 点群の ICP/GICP/VGICP を比較 |
| `optional/opt_gtsam_imu_isam2.py` | `pip install gtsam` | GTSAM の ImuFactor と iSAM2 で、地図が途切れる区間を IMU + 相対姿勢でつなぐ |

## 演習 7 の構成 (論文との対応)

| 論文 | `ex07_sliding_window_localization.py` |
| --- | --- |
| 状態 x_i (姿勢・速度・バイアス) | 8 次元ベクトル `[x, y, θ, vx, vy, b_ax, b_ay, b_g]` |
| f^P(x_i, x_j) (j = i-2, i-1) | `ScanFactor(target, j, voxelmap)` を `FULL_CONNECTION = 2` フレーム分 |
| f^P(x_i, M) | `ScanFactor(None, j, map_voxelmap)` (ターゲット = 世界座標に固定した地図) |
| f^I(x_{i-1}, x_i) | `ImuFactor(i, j, Preintegration(...))` + `BiasFactor` |
| C(X) | `WindowEstimator.marginalize_oldest()` が作る `LinearPrior` |
| ウィンドウ (例: 5 秒) | `SMOOTHER_LAG = 3.0` 秒 (計算を軽くするため短め) |
| iSAM2 | 密行列のガウス・ニュートン法 (小さい問題なので十分) |
| GPU | なし (numpy のベクトル演算) |

2D に簡略化した点: LiDAR は 2D、動きによる点群の歪みなし、LiDAR と IMU の外部パラメータは既知 (同じ位置)、
キーフレームなし (論文の式どおり直前 2 フレームのみ)。
