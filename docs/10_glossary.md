# 10. 用語集

論文や関連資料に出てくる用語を、日本語・英語の対応と一言の説明でまとめました。括弧内は主に説明している章です。

## 座標・回転 (01 章)

| 日本語 | 英語 | 一言で |
| --- | --- | --- |
| 姿勢 | pose | 位置と向きの組。3D では 6 自由度 |
| 6 自由度 | 6DoF (degrees of freedom) | 並進 3 + 回転 3 |
| 回転行列 | rotation matrix | 向きを表す 3×3 の直交行列。SO(3) の要素 |
| 同次変換行列 | homogeneous transformation | 回転と並進をまとめた 4×4 行列。SE(3) の要素 |
| 相対姿勢 | relative pose | 2 つの姿勢の差。T_i^-1 T_j |
| リー群 | Lie group | 回転や姿勢のように「滑らかに動かせる群」。SO(3), SE(3) |
| 接空間 / リー代数 | tangent space / Lie algebra | 群の 1 点に接する平らな空間。更新量を表す |
| 指数写像 / 対数写像 | exponential / logarithm map (Exp / Log) | 回転ベクトル ⇔ 回転行列 の変換 |
| hat 演算子 | hat operator, skew-symmetric matrix | ベクトルを外積の行列 [ω]× にする |
| 摂動 | perturbation | 推定値に微小な変化を加えること。ヤコビアンの導出に使う |
| 外部パラメータ | extrinsic parameters | センサ同士の相対姿勢 (例: T_lidar_imu) |

## 確率・最適化 (02, 05 章)

| 日本語 | 英語 | 一言で |
| --- | --- | --- |
| 共分散行列 | covariance matrix | 多次元のばらつきの大きさと向き |
| 情報行列 | information matrix | 共分散の逆行列。確信の強さ |
| マハラノビス距離 | Mahalanobis distance | ばらつきを考慮した距離 |
| 最尤推定 / 最大事後確率推定 | maximum likelihood / MAP estimation | 観測が最も起こりやすい状態を選ぶ |
| 非線形最小二乗 | nonlinear least squares | 誤差の 2 乗和の最小化 |
| ガウス・ニュートン法 | Gauss-Newton method | 線形化して正規方程式を解く反復法 |
| レーベンバーグ・マーカート法 | Levenberg-Marquardt method | ダンピング付きの GN 法 |
| ヤコビアン | Jacobian | 誤差を状態で微分した行列 |
| ヘッセ行列 (の近似) | Hessian (approximation) | H = JᵀJ |
| 線形化点 | linearization point | 1 次近似するときの基準の推定値 |
| 局所解 | local minimum | 一番低くはない谷 |
| 退化 | degeneracy | 観測から決まらない方向がある状態 |
| ロバストカーネル | robust kernel (M-estimator) | 外れ値の影響を弱める重み関数 |
| ファクターグラフ | factor graph | 変数と因子のグラフで推定問題を表す |
| 因子 | factor | 変数間の確率的な制約 (誤差項) |
| 単項 / 二項因子 | unary / binary factor | 1 つ / 2 つの変数に関係する因子 |
| スパース | sparse | 行列のほとんどの要素が 0 |
| フィルイン | fill-in | 分解や消去で新しく非ゼロ要素が増えること |
| iSAM2 | incremental Smoothing and Mapping 2 | ベイズ木を使うインクリメンタル最適化 |
| ベイズ木 | Bayes tree | iSAM2 が使う、分解結果を表す木構造 |
| GTSAM | Georgia Tech Smoothing and Mapping | ファクターグラフ最適化ライブラリ |

## 点群処理 (03 章)

| 日本語 | 英語 | 一言で |
| --- | --- | --- |
| 点群 | point cloud | 3D 点の集合 |
| スキャンマッチング / 位置合わせ | scan matching / registration | 2 つの点群を重ねる変換を求める |
| ソース / ターゲット | source / target | 動かす側 / 動かさない側の点群 |
| 対応点 | correspondence | 同じ場所を表すと思われる点のペア |
| ICP | Iterative Closest Point | 最近傍点との距離を最小化する反復法 |
| 点対面 ICP | point-to-plane ICP | 面までの距離を使う ICP |
| NDT | Normal Distributions Transform | ボクセルごとの正規分布に点を合わせる |
| GICP | Generalized ICP | 分布対分布のマッチング |
| VGICP | Voxelized GICP | ボクセル化した GICP。GPU 向き |
| ボクセル | voxel | 空間を区切った立方体のマス |
| ダウンサンプリング | downsampling | 点を間引く |
| kd 木 | k-d tree | 最近傍探索用の木構造 |
| 法線 | normal | 面に垂直な向き |
| マルチ解像度 | multi-resolution | 粗いボクセルと細かいボクセルの併用 |
| 収束域 | basin of convergence | 正しく収束できる初期値の範囲 |
| マッチングコスト因子 | matching cost factor | 位置合わせの誤差関数そのものを因子にしたもの |
| 歪み補正 | deskewing, motion compensation | スキャン中の動きによる点群の歪みを直す |

## IMU (04 章)

| 日本語 | 英語 | 一言で |
| --- | --- | --- |
| 慣性計測装置 | IMU (Inertial Measurement Unit) | 加速度計 + ジャイロ |
| 比力 | specific force | 加速度計が測る「加速度 − 重力」 |
| バイアス | bias | ゆっくり変化するオフセット誤差 |
| ランダムウォーク | random walk | バイアスの変化のモデル |
| デッドレコニング | dead reckoning | 動きの積分だけで位置を求めること |
| ドリフト | drift | 積分誤差がたまって推定がずれていくこと |
| 事前積分 | preintegration | 区間内の IMU 計測を始点非依存の量にまとめる |

## 推定の枠組み・自己位置推定 (06, 07 章)

| 日本語 | 英語 | 一言で |
| --- | --- | --- |
| オドメトリ | odometry | 相対的な動きの推定 |
| SLAM | Simultaneous Localization and Mapping | 地図作成と自己位置推定を同時に行う |
| 自己位置推定 | localization | 既知の地図上で自分の姿勢を求める |
| 事前地図 | prior map | あらかじめ作っておいた地図 |
| 疎結合 / 密結合 | loosely / tightly coupled | 結果どうしを融合 / 生の制約を 1 つの問題で同時に解く |
| 平滑化 | smoothing | 過去の状態も含めて推定し直す |
| 固定ラグ平滑化 | fixed-lag smoothing | 最新の一定時間分だけを平滑化する |
| スライディングウィンドウ | sliding window | 固定ラグ平滑化の別名に近い |
| 周辺化 | marginalization | 変数を積分して消し、情報を残す |
| シューア補行列 | Schur complement | 周辺化の計算に使う行列 |
| キーフレーム | keyframe | 代表として残す一部のフレーム |
| カルマンフィルタ / UKF | Kalman filter / Unscented Kalman Filter | 予測と更新を繰り返す逐次推定 |
| パーティクルフィルタ / MCL | particle filter / Monte Carlo Localization | 多数の仮説で分布を表す逐次推定 |
| 距離場 | distance field | 各位置から最も近い物体までの距離の表 |
| ATE | Absolute Trajectory Error | 推定軌跡と真の軌跡の誤差の指標 |

---

前へ: [09 著者のコード案内](09_author_code_guide.md) ｜ 目次: [00 学習ロードマップ](00_roadmap.md)
