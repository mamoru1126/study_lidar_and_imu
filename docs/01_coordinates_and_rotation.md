# 01. 座標と回転 (SO(3), SE(3), Lie 群)

## この章のゴール

- センサの「姿勢 (pose)」を回転行列と並進ベクトルで表し、点の座標変換ができる
- 相対姿勢 $\mathbf{T}_i^{-1}\mathbf{T}_j$ の意味が分かる
- 回転は普通のベクトルのように足し算できないこと、そのために Lie 群の道具 (Exp / Log) を使うことが分かる
- 「微小回転を掛ける」ことでヤコビアンを求める方法 (摂動) が分かる

> **論文とのつながり**
> 論文の状態 $\mathbf{x}_i$ には 6 自由度の姿勢が含まれます。スキャンマッチング因子 $f^P(\mathbf{x}_i, \mathbf{x}_j)$ は
> 「フレーム j の点群を、相対姿勢でフレーム i の座標に移して比べる」ものですし、
> 最適化 (ガウス・ニュートン法) では姿勢に「微小な更新量」を足していく必要があります。
> この章の内容はすべての因子の土台です。

---

## 1. 姿勢 = 位置 + 向き

3 次元空間の剛体の姿勢は、**位置 3 自由度 + 向き 3 自由度 = 6 自由度 (6DoF)** で表せます。
論文タイトルの「6DoF 姿勢推定」はこれのことです。

- 位置: 並進ベクトル $\mathbf{t} \in \mathbb{R}^3$
- 向き: 回転行列 $\mathbf{R} \in SO(3)$

回転行列は $3 \times 3$ の行列ですが、次の条件を満たすものだけが「回転」です。

```math
\mathbf{R}^\top \mathbf{R} = \mathbf{I}, \qquad \det(\mathbf{R}) = 1
```

この条件を満たす行列全体を **SO(3)** (3 次元の特殊直交群) と呼びます。
9 個の数字を持っていますが、条件のせいで自由に動かせるのは 3 自由度だけです。

## 2. 座標変換と同次変換行列

センサ (LiDAR) が観測した点 $\mathbf{p}$ はセンサの座標系で表されています。
センサの姿勢が $(\mathbf{R}, \mathbf{t})$ のとき、その点の世界座標は

```math
\mathbf{p}^{\text{world}} = \mathbf{R}\,\mathbf{p}^{\text{sensor}} + \mathbf{t}
```

です。これを 4×4 の **同次変換行列** でまとめると、掛け算 1 回で書けて便利です。

```math
\mathbf{T} = \begin{bmatrix} \mathbf{R} & \mathbf{t} \\ \mathbf{0}^\top & 1 \end{bmatrix} \in SE(3),
\qquad
\begin{bmatrix} \mathbf{p}^{\text{world}} \\ 1 \end{bmatrix} = \mathbf{T} \begin{bmatrix} \mathbf{p}^{\text{sensor}} \\ 1 \end{bmatrix}
```

このような行列全体を **SE(3)** (3 次元の特殊ユークリッド群) と呼びます。

### 合成と逆変換

- 合成: 変換を続けて行うのは行列の掛け算 $\mathbf{T}_a \mathbf{T}_b$
- 逆変換: 回転行列の逆は転置で済むので、次のように簡単に計算できる

```math
\mathbf{T}^{-1} = \begin{bmatrix} \mathbf{R}^\top & -\mathbf{R}^\top \mathbf{t} \\ \mathbf{0}^\top & 1 \end{bmatrix}
```

### 名前の付け方のコツ: `T_world_sensor`

著者のコード (GLIM など) では `T_world_imu` のような変数名がよく出てきます。
これは「imu 座標系の点を world 座標系に移す変換」という意味です。
添字を「左 ← 右」と読むと、合成が正しいかを目で確認できます。

```math
\mathbf{T}_{\text{world}\leftarrow\text{lidar}} = \mathbf{T}_{\text{world}\leftarrow\text{imu}}\;\mathbf{T}_{\text{imu}\leftarrow\text{lidar}}
```

(内側の添字 imu が打ち消し合っている)

### 相対姿勢

世界座標での姿勢 $\mathbf{T}_i, \mathbf{T}_j$ から、**フレーム j の座標をフレーム i の座標に移す変換** は

```math
\mathbf{T}_{ij} = \mathbf{T}_i^{-1}\,\mathbf{T}_j
```

です。論文のスキャン間マッチング因子は、まさにこの $\mathbf{T}_{ij}$ でフレーム j の点をフレーム i に移して
比べています ([03 章](03_point_cloud_registration.md))。

## 3. 回転の表し方いろいろ

| 表現 | 数字の個数 | 長所 | 短所 |
| --- | --- | --- | --- |
| 回転行列 | 9 | 合成・点の変換が掛け算で簡単 | 冗長。誤差が溜まると直交性が崩れる |
| オイラー角 (roll, pitch, yaw) | 3 | 人間に分かりやすい | ジンバルロック (特異点) がある。合成が面倒 |
| 四元数 (クォータニオン) | 4 | コンパクトで特異点なし。数値的に安定 | 直感的でない。q と -q が同じ回転 |
| 回転ベクトル (軸 × 角度) | 3 | 最小の 3 個。微小回転の表現に最適 | 合成は直接できない (Exp/Log を経由) |

最適化では「**姿勢そのものは回転行列 (や四元数) で持ち、更新量は回転ベクトルで表す**」のが定番です。
GTSAM もこの方式です。

## 4. 回転は足し算できない → Lie 群の考え方

ガウス・ニュートン法では「今の推定値 + 更新量」を繰り返します。
しかし回転行列に普通に行列を足すと、$\mathbf{R}^\top\mathbf{R} = \mathbf{I}$ が崩れて回転行列でなくなってしまいます。

そこで、

1. 更新量は 3 次元の **回転ベクトル** $\boldsymbol{\delta} \in \mathbb{R}^3$ で表す (これは自由に足し引きできる)
2. それを **指数写像 Exp** で回転行列に直す
3. 掛け算で今の回転に合成する

という手順を踏みます。

```math
\mathbf{R} \leftarrow \mathbf{R}\,\operatorname{Exp}(\boldsymbol{\delta})
```

### hat 演算子と指数写像

回転ベクトル $\boldsymbol{\omega} = (\omega_x, \omega_y, \omega_z)$ から作る歪対称行列を $[\boldsymbol{\omega}]_\times$ と書きます (hat 演算子)。

```math
[\boldsymbol{\omega}]_\times = \begin{bmatrix} 0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0 \end{bmatrix},
\qquad [\boldsymbol{\omega}]_\times \mathbf{p} = \boldsymbol{\omega} \times \mathbf{p}
```

指数写像は、回転角 $\theta = \lVert\boldsymbol{\omega}\rVert$、回転軸 $\mathbf{a} = \boldsymbol{\omega}/\theta$ として
**ロドリゲスの公式** で計算できます。

```math
\operatorname{Exp}(\boldsymbol{\omega}) = \mathbf{I} + \sin\theta\,[\mathbf{a}]_\times + (1 - \cos\theta)\,[\mathbf{a}]_\times^2
```

逆向きの **対数写像 Log** は回転行列から回転ベクトルを取り出します。

イメージとしては、SO(3) は「曲がった空間 (多様体)」で、その 1 点に接する平らな空間 (接空間, 3 次元) の上で
計算してから Exp で曲がった空間に戻している、という感じです。

### 微小回転の近似

$\boldsymbol{\delta}$ が小さいときは

```math
\operatorname{Exp}(\boldsymbol{\delta}) \approx \mathbf{I} + [\boldsymbol{\delta}]_\times
```

と近似できます。演習 1 の図で、この近似の誤差が角度の 2 乗で小さくなることを確かめられます。

![1 次近似の精度](figures/ex01_exp_approx.png)

## 5. 摂動 (perturbation) でヤコビアンを求める

最適化には「状態を少し動かしたとき、誤差がどれだけ変わるか (ヤコビアン)」が必要です。
回転を少し動かすことを「$\mathbf{R}$ に右から $\operatorname{Exp}(\boldsymbol{\delta})$ を掛ける」と定義すると、
計算がとても簡単になります。

例として、点を回転させる関数 $f(\mathbf{R}) = \mathbf{R}\mathbf{p}$ を考えます。

```math
\mathbf{R}\operatorname{Exp}(\boldsymbol{\delta})\,\mathbf{p}
\approx \mathbf{R}(\mathbf{I} + [\boldsymbol{\delta}]_\times)\mathbf{p}
= \mathbf{R}\mathbf{p} + \mathbf{R}(\boldsymbol{\delta}\times\mathbf{p})
= \mathbf{R}\mathbf{p} - \mathbf{R}[\mathbf{p}]_\times\boldsymbol{\delta}
```

したがって

```math
\frac{\partial (\mathbf{R}\mathbf{p})}{\partial \boldsymbol{\delta}} = -\mathbf{R}\,[\mathbf{p}]_\times
```

です。最後の変形では外積の性質 $\boldsymbol{\delta}\times\mathbf{p} = -\mathbf{p}\times\boldsymbol{\delta}$ を使いました。

GLIM の論文 (RAS 2024) に出てくる VGICP 因子のヤコビアン
$\mathbf{A}_k = [\,\mathbf{R}_{ij}[\boldsymbol{\mu}_k]_\times,\ -\mathbf{R}_{ij}\,]$ なども、この計算の応用です
(符号は誤差の定義の向きで変わります)。

> **右摂動と左摂動**
> $\mathbf{R}\operatorname{Exp}(\boldsymbol{\delta})$ (右から掛ける) と $\operatorname{Exp}(\boldsymbol{\delta})\mathbf{R}$ (左から掛ける) の 2 流派があります。
> 右はセンサ座標系で、左は世界座標系で微小回転を考えることに相当します。GTSAM は右摂動です。
> 論文やコードを読むときは、どちらを使っているかを最初に確認しましょう。

## 6. SE(3) の場合

姿勢全体 $\mathbf{T}$ も同じように、6 次元の更新量 $\boldsymbol{\xi} = (\boldsymbol{\delta}_\text{rot}, \boldsymbol{\delta}_\text{trans})$ を使って
$\mathbf{T} \leftarrow \mathbf{T}\operatorname{Exp}(\boldsymbol{\xi})$ と更新します。
GTSAM の `Pose3` は回転 3 成分 → 並進 3 成分の順に並べます (ライブラリによって順番が違うので注意)。

## 7. 演習で使う 2D 版: SE(2)

演習では計算を簡単にするため、2D の姿勢 $(x, y, \theta)$ を使います。

```math
\mathbf{R}(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix},
\qquad \mathbf{p}^{\text{world}} = \mathbf{R}(\theta)\mathbf{p} + \begin{bmatrix} x \\ y \end{bmatrix}
```

2D の回転は角度 1 個で表せて、角度は (範囲を $[-\pi, \pi)$ に丸めれば) 足し算できます。
3D の考え方を 2D で練習してから 3D に戻ると理解しやすくなります。

## 演習 1

```bash
python exercises/ex01_rotation.py
```

- 回転行列の性質、Exp と Log が逆であることを確認
- 回転の合成の順番を入れ替えると結果が変わることを確認
- 解析的に求めたヤコビアン $-\mathbf{R}[\mathbf{p}]_\times$ と数値微分が一致することを確認
- 相対姿勢 $\mathbf{T}_i^{-1}\mathbf{T}_j$ で点を変換

## 理解度チェック

1. 回転行列が 9 個の数字を持つのに、自由度が 3 なのはなぜ?
2. $\mathbf{T}_i$, $\mathbf{T}_j$ が世界座標での姿勢のとき、フレーム j で観測した点をフレーム i の座標に移す変換は?
3. 最適化の更新で $\mathbf{R} + \Delta\mathbf{R}$ としてはいけないのはなぜ? 代わりにどうする?
4. $\dfrac{\partial(\mathbf{R}\operatorname{Exp}(\boldsymbol{\delta})\mathbf{p})}{\partial\boldsymbol{\delta}}$ を導出せよ。

<details markdown="1">
<summary>答えの例</summary>

1. $\mathbf{R}^\top\mathbf{R} = \mathbf{I}$ (6 個の独立な条件) があるので、9 − 6 = 3 自由度。
2. $\mathbf{T}_{ij} = \mathbf{T}_i^{-1}\mathbf{T}_j$。
3. 足し算すると直交性が崩れて回転行列でなくなる。回転ベクトル $\boldsymbol{\delta}$ で更新量を表し、$\mathbf{R}\operatorname{Exp}(\boldsymbol{\delta})$ と掛け算で更新する。
4. $\operatorname{Exp}(\boldsymbol{\delta}) \approx \mathbf{I} + [\boldsymbol{\delta}]_\times$ を使って $-\mathbf{R}[\mathbf{p}]_\times$。

</details>

## もっと知りたい人へ

- J. Solà, J. Deray, D. Atchuthan, "A micro Lie theory for state estimation in robotics," arXiv:1812.01537, 2018.
  (Lie 群を「使う側」の視点で短くまとめた定番資料)
- T. D. Barfoot, *State Estimation for Robotics*, Cambridge University Press. (無料 PDF あり)

---

前へ: [00 学習ロードマップ](00_roadmap.md) ｜ 次へ: [02 確率と最小二乗](02_probability_and_least_squares.md)
