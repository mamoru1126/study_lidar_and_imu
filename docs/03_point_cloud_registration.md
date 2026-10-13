# 03. スキャンマッチング (ICP → NDT → GICP → VGICP)

## この章のゴール

- 点群の前処理 (ダウンサンプリング、近傍探索、共分散推定) が分かる
- ICP・点対面 ICP・NDT・GICP・VGICP の違いを、図とコスト関数で説明できる
- VGICP が速く、GPU と相性が良い理由が分かる
- 論文が「位置合わせの結果」ではなく「位置合わせのコストそのもの」を因子にしている意味が分かる

> **論文とのつながり**
> 論文の $f^P(\mathbf{x}_i, \mathbf{x}_j)$ (フレーム同士) と $f^P(\mathbf{x}_i, \mathcal{M})$ (フレームと地図) は、
> どちらも **VGICP** (Voxelized GICP) のコストです。著者自身が提案した手法で、GPU で高速に計算できます。

---

## 1. 点群と前処理

LiDAR は周囲の物体までの距離を大量に測り、**点群** (3 次元の点の集まり) を出力します。
論文の手法は特定のセンサに依存しない設計で、回転式 LiDAR だけでなく RGB-D カメラ (Azure Kinect など) にも
適用できると述べられています。

マッチングの前に、たいてい次の前処理をします。

| 処理 | 目的 | 典型的な方法 |
| --- | --- | --- |
| ダウンサンプリング | 点を減らして高速化、密度を均一に | ボクセルグリッド (各ボクセルの重心を残す)、ランダム間引き |
| 近傍探索 | 各点の近くの点を見つける | kd-tree、ボクセルハッシュ |
| 法線・共分散の推定 | 点の周りの面の向き・形を知る | 近傍 k 点の共分散行列を固有値分解 |
| 動きによる歪みの補正 (deskew) | 1 スキャン中にセンサが動いた影響を除く | IMU で各点の時刻の姿勢を補間 |

## 2. 位置合わせ (registration) の問題

2 つの点群、**ソース** $\mathcal{P} = \lbrace\mathbf{p}_k\rbrace$ と **ターゲット** $\mathcal{Q}$ があり、
ソースを動かしてターゲットに重ねる変換 $\mathbf{T}$ を求める問題です。
ソースのどの点がターゲットのどの点に対応するかは分からないので、

1. 今の $\mathbf{T}$ で対応 (近い点どうし) を決める
2. 対応が正しいとして $\mathbf{T}$ を最適化する

を繰り返します。違いは「何と何の距離を測るか」です。

![位置合わせの種類](figures/fig_registration_types.svg)

## 3. 点対点 ICP

最も素朴な方法で、最近傍の点との距離の 2 乗和を最小化します。

```math
E(\mathbf{T}) = \sum_k \lVert \mathbf{q}_k - \mathbf{T}\mathbf{p}_k \rVert^2
```

対応が決まっていれば、重心を合わせて SVD を使うと閉じた形で解けます (演習 3 の `icp_point_to_point`)。
ただし、LiDAR の点は物体の表面を「まばらにサンプリング」したものなので、2 つのスキャンで同じ場所の点が
取れているとは限りません。そのため、壁に沿う方向に余計な力がかかり、**収束が遅く、結果が偏り**やすくなります。

## 4. 点対面 ICP

最近傍点の「面」(法線 $\mathbf{n}_k$ を持つ平面) までの距離を使います。

```math
E(\mathbf{T}) = \sum_k \left(\mathbf{n}_k^\top(\mathbf{q}_k - \mathbf{T}\mathbf{p}_k)\right)^2
```

面に沿う方向のずれは誤差にならないので、サンプリング位置の違いに強く、収束がずっと速くなります。

## 5. NDT (Normal Distributions Transform)

ターゲットを **ボクセル (立方体のマス)** に分け、各ボクセル内の点の分布を正規分布 (平均・共分散) で表します。
ソースの各点が、入ったボクセルの分布の中でどれくらい「もっともらしいか」を最大化します。

- 長所: 対応点探索が不要 (ボクセルを引くだけ)。地図との照合に広く使われている (Autoware など)
- 短所: 1 つのボクセルに十分な点がないと分布が作れない。ボクセルサイズに敏感

論文の比較手法の 1 つ「NDT スキャンマッチング」はこれです。

## 6. GICP (Generalized ICP)

GICP [Segal+ RSS2009] は **ソースの点もターゲットの点も正規分布** とみなします。
点 $\mathbf{p}_k$ の周りの面の形を共分散 $\mathbf{C}_k$ で表し、対応点 $\mathbf{q}_k$ の共分散を $\mathbf{C}'_k$ とすると、

```math
\mathbf{d}_k = \mathbf{q}_k - \mathbf{T}\mathbf{p}_k,
\qquad
E(\mathbf{T}) = \sum_k \mathbf{d}_k^\top \left(\mathbf{C}'_k + \mathbf{R}\mathbf{C}_k\mathbf{R}^\top\right)^{-1} \mathbf{d}_k
```

を最小化します。2 つの正規分布の差もまた正規分布で、その共分散が $\mathbf{C}'_k + \mathbf{R}\mathbf{C}_k\mathbf{R}^\top$ になる、
というのが式の意味です ([02 章](02_probability_and_least_squares.md) のマハラノビス距離)。

**共分散の正則化**: GICP では、近傍点から求めた共分散の固有値を $(1, 1, \varepsilon)$ ($\varepsilon \approx 10^{-3}$) に置き換え、
「面に沿う 2 方向は大きく、法線方向だけ小さい」形にそろえます。こうすると、

- 面に沿う方向のずれはほとんど誤差にならない (点対面 ICP と同じ効果)
- 両方の点群の面の向きを考える「面対面」のマッチングになる

という性質が得られ、精度と頑健さが向上します。共分散の正則化の値を変えると、ICP (等方的な分布) や
点対面 ICP に近づけることもできます。

![ICP の比較](figures/ex03_icp_2d.png)

演習 3 では、点対点 ICP が 60 回反復しても数 cm の偏りが残るのに対し、点対線 ICP と GICP は 2〜3 回で
ほぼ収束することが分かります。

<!-- demo:icp -->

> **HTML 版**: ここで 2D の ICP を一歩ずつ動かして、点対点と点対面の違いを見られます。

## 7. VGICP (Voxelized GICP)

GICP は各点について kd-tree で最近傍を探す必要があり、点が多いと重くなります。
VGICP [Koide+ ICRA2021] は、NDT のようにターゲットをボクセルに分けますが、
**各ボクセルの分布を「中の点の分布 (GICP の共分散) を平均したもの」** として作ります。

```math
\boldsymbol{\mu}^{\text{voxel}} = \frac{1}{N}\sum_{\mathbf{q} \in \text{voxel}} \mathbf{q},
\qquad
\mathbf{C}^{\text{voxel}} = \frac{1}{N}\sum_{\mathbf{q} \in \text{voxel}} \mathbf{C}_{\mathbf{q}}
```

ソースの点 $\mathbf{p}_k$ は、変換後に入ったボクセルの分布と比べるだけです。

```math
E(\mathbf{T}) = \sum_k \mathbf{d}_k^\top \left(\mathbf{C}^{\text{voxel}}_k + \mathbf{R}\mathbf{C}_k\mathbf{R}^\top\right)^{-1}\mathbf{d}_k,
\qquad \mathbf{d}_k = \boldsymbol{\mu}^{\text{voxel}}_k - \mathbf{T}\mathbf{p}_k
```

ポイントは次の 3 つです。

1. **対応探索がハッシュ表の参照 1 回で済む**: kd-tree の探索が要らない
2. **点の少ないボクセルでも使える**: NDT はボクセル内の点から共分散を計算するので点が少ないと不安定だが、
   VGICP は「点ごとの共分散 (近傍から計算済み)」を平均するので 1 点でも分布になる
3. **GPU 向き**: 全点で同じ計算 (参照 → 行列計算) をするだけで、条件分岐やデータ依存の探索がない。
   数万点を並列に処理できる

### マルチ解像度ボクセル

VGICP は「点が入ったボクセル 1 つ」としか比べないので、初期値が大きくずれていると正しいボクセルに入らず、
**収束域 (うまく合う初期値の範囲) が狭く**なります。そこで、細かいボクセルと粗いボクセルを併用します。
GLIM の設定 `voxelmap_levels` (例: 2 段) がこれです。演習 4 で、粗いボクセルを足すと収束域が広がることを確認できます。

![VGICP とマルチ解像度](figures/ex04_vgicp_2d.png)

## 8. 位置合わせの「結果」ではなく「コスト」を因子にする

ここが論文 (と著者の一連の研究) を理解するうえで重要な点です。

**よくある方法 (疎結合・ポーズグラフ)**:
ICP などで 2 フレーム間の相対姿勢を求め、その結果 (と、だいたいの共分散) を「相対姿勢の観測」として
グラフに入れます。

**論文の方法 (マッチングコスト因子)**:
VGICP の誤差関数 $E(\mathbf{T}_i^{-1}\mathbf{T}_j)$ そのものを因子 $f^P(\mathbf{x}_i, \mathbf{x}_j)$ としてグラフに入れます。
最適化のたびに、今の推定値で対応を取り直して線形化します (GLIM の `IntegratedVGICPFactorGPU`)。

この違いには次の意味があります。

- 位置合わせの結果を 1 つの正規分布で要約するときの **情報の欠落がない**
- 廊下のような退化した環境でも、「決まる方向」の情報だけが自然にグラフに入り、
  決まらない方向は IMU など他の因子が補う
- 1 つのフレームを複数のフレーム・地図と **同時に** 合わせられる

GLIM の論文では、因子 1 つあたりの線形化結果 ($\mathbf{H}$ と $\mathbf{b}$ のブロック) を、全点の和として GPU で計算しています。

```math
\mathbf{H}_{ii} = \sum_k \mathbf{A}_k^\top \boldsymbol{\Omega}_k \mathbf{A}_k, \quad
\mathbf{H}_{ij} = \sum_k \mathbf{A}_k^\top \boldsymbol{\Omega}_k \mathbf{B}_k, \quad
\mathbf{H}_{jj} = \sum_k \mathbf{B}_k^\top \boldsymbol{\Omega}_k \mathbf{B}_k
```

($\mathbf{A}_k$, $\mathbf{B}_k$ は 2 つの姿勢に対する誤差のヤコビアン。転置の位置は論文によって書き方が異なります)
演習 7 の `ScanFactor.linearize` が、これを numpy で書いたものです。

## 9. GPU のちょっとした話

GLIM の論文によれば、多数の VGICP 因子を線形化するとき、

- 線形化点 (各姿勢の推定値) をまとめて 1 回で GPU に送り、
- 因子ごとの計算は複数のストリームで並列に行い、
- 結果もまとめて 1 回で CPU に戻す

ことで、因子の数によらず CPU⇔GPU の転送を 2 回に抑えています。
「計算そのもの」より「転送や同期」がボトルネックになりやすいのが GPU プログラミングの勘所です。

## 演習 3, 4

```bash
python exercises/ex03_icp_2d.py      # 点対点 ICP, 点対線 ICP, GICP の比較
python exercises/ex04_vgicp_2d.py    # ボクセル地図, 対応探索の速さ, マルチ解像度
```

## 理解度チェック

1. 点対点 ICP が LiDAR 点群で偏りやすいのはなぜ?
2. GICP の $(\mathbf{C}'_k + \mathbf{R}\mathbf{C}_k\mathbf{R}^\top)^{-1}$ は何を表している?
3. NDT と VGICP は、どちらもボクセルごとに正規分布を持つ。違いは何?
4. VGICP が GPU で速いのはなぜ?
5. 「位置合わせの結果を因子にする」のと「位置合わせのコストを因子にする」のでは何が違う?

<details markdown="1">
<summary>答えの例</summary>

1. 2 つのスキャンで表面上の同じ位置の点が取れているとは限らず、最近傍の点が少しずれた場所にあるため、面に沿う方向に余計な誤差が生じる。
2. ソースの点の分布とターゲットの点の分布の差 (これも正規分布) の共分散の逆、つまり情報行列。面の法線方向の誤差を重く、面に沿う方向を軽く数える。
3. NDT はボクセル内の点の座標から共分散を作るので点が少ないと不安定。VGICP は点ごとに近傍から求めた共分散を平均するので、点が少なくても分布が作れる。また VGICP はソース側の点の共分散も使う (分布対分布)。
4. 対応探索がボクセルの参照 1 回で済み、全点で同じ計算をするだけなので、分岐が少なく大量に並列化できる。
5. 結果を因子にすると、相対姿勢の正規分布という要約で情報が失われる。コストを因子にすると、点ごとの誤差を最適化のたびに線形化し直すので、退化方向の扱いや複数フレームとの同時マッチングが自然にできる。

</details>

## もっと知りたい人へ

- A. Segal, D. Haehnel, S. Thrun, "Generalized-ICP," RSS 2009.
- K. Koide, M. Yokozuka, S. Oishi, A. Banno, "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration," ICRA 2021.
- K. Koide, M. Yokozuka, S. Oishi, A. Banno, "Globally Consistent 3D LiDAR Mapping with GPU-accelerated GICP Matching Cost Factors," IEEE RA-L, 2021.
- 著者の実装: [fast_gicp](https://github.com/koide3/fast_gicp), [small_gicp](https://github.com/koide3/small_gicp), [gtsam_points](https://github.com/koide3/gtsam_points)

---

前へ: [02 確率と最小二乗](02_probability_and_least_squares.md) ｜ 次へ: [04 IMU と事前積分](04_imu_and_preintegration.md)
