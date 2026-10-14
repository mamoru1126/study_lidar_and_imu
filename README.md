# study_lidar_and_imu — LiDAR×IMU 論文ノート

次の論文を理解するために必要な知識を、**解説 (Markdown / HTML)** と **手を動かす演習 (Python)** で順に身につけるための学習環境です。

> **点群・IMU 制約のウィンドウ最適化に基づく既知地図上での 6DoF 姿勢推定**
> 小出 健司, 大石 修士, 横塚 将志, 阪野 貴彦 (産総研), ROBOMECH2023
> PDF: <https://staff.aist.go.jp/k.koide/assets/pdf/robomech2023.pdf> ／ 著者の GitHub: <https://github.com/koide3>

![論文の全体像](docs/figures/fig_overview.svg)

## 読み方は 2 通り

| 形式 | 場所 | 特徴 |
| --- | --- | --- |
| Markdown | [`docs/`](docs/00_roadmap.md) | GitHub 上でそのまま読める (数式も表示される) |
| HTML | [Web 版](https://mamoru1126.github.io/study_lidar_and_imu/html/) / [`html/index.html`](html/index.html) | 目次・読了チェック付き。**ICP・IMU ドリフト・式と因子の対応・再現実験の再生** を動かせるインタラクティブデモ入り |

HTML 版の開き方:

- **Web で読む**: <https://mamoru1126.github.io/study_lidar_and_imu/html/>
- **ローカル**: クローンして `html/index.html` をブラウザで開くだけ (数式表示用の KaTeX を同梱しているのでオフラインでも読めます)

## 学習ロードマップ

![知識の地図](docs/figures/fig_knowledge_map.svg)

| 章 | 内容 | 演習 |
| --- | --- | --- |
| [00 学習ロードマップ](docs/00_roadmap.md) | 論文をひとことで。目的関数の全体像 | - |
| [01 座標と回転](docs/01_coordinates_and_rotation.md) | 回転行列、SO(3)/SE(3)、Exp/Log、摂動によるヤコビアン | ex01 |
| [02 確率と最小二乗](docs/02_probability_and_least_squares.md) | 誤差ベクトル、正規分布と共分散行列、最尤推定、ガウス・ニュートン法 | ex02a, ex02b |
| [03 スキャンマッチング](docs/03_point_cloud_registration.md) | ICP、NDT、GICP、VGICP (NDT と GICP の違い)、マッチングコスト因子 | ex03a, ex03b, ex04 |
| [04 IMU と事前積分](docs/04_imu_and_preintegration.md) | 比力、バイアス、ドリフト、ΔR・Δv・Δp | ex05 |
| [05 ファクターグラフ](docs/05_factor_graph.md) | 因子と変数、スパース性、GTSAM、iSAM2 | ex06 |
| [06 固定ラグ平滑化と周辺化](docs/06_fixed_lag_smoothing.md) | ウィンドウ最適化、シューア補行列、キーフレーム | ex06, ex07 |
| [07 既知地図上の自己位置推定](docs/07_map_based_localization.md) | NDT・UKF・MCL と提案手法の違い | ex07 |
| [08 論文精読ガイド](docs/08_paper_walkthrough.md) | 論文を節ごとに読む。式と因子の対応 | ex07 |
| [09 著者のコード案内](docs/09_author_code_guide.md) | GLIM・gtsam_points のどこに何があるか | optional |
| [10 用語集](docs/10_glossary.md) | 日本語・英語の対応表 | - |

## 演習の環境構築

ROS も GPU も不要です。Python 3.9 以上で動きます。

```bash
git clone https://github.com/mamoru1126/study_lidar_and_imu.git
cd study_lidar_and_imu
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python exercises/check_env.py
```

図の日本語が □ になる場合は、日本語フォントを入れてください (Ubuntu: `sudo apt install fonts-noto-cjk`)。

### Docker を使う場合

```bash
docker build -t study-lidar-imu .
docker run --rm -it -v "$PWD":/work study-lidar-imu make exercises
```

### make で一括実行

```bash
make check       # 環境の確認
make exercises   # 演習 ex01〜ex07 を全部実行 (図を docs/figures に保存。3 分ほど)
make html        # docs/ から html/ を再生成
```

## 演習の一覧

詳しくは [`exercises/README.md`](exercises/README.md) を見てください。

| 演習 | 内容 |
| --- | --- |
| `ex01_rotation.py` | 回転行列、Exp/Log、摂動ヤコビアンを数値で確認 |
| `ex02a_gaussian.py` | 誤差ベクトル、正規分布の 1/2、共分散行列、マハラノビス距離を数値で確認 |
| `ex02b_least_squares.py` | ガウス・ニュートン法と LM 法。局所解と重み付けの効果 |
| `ex03a_matching_methods.py` | ICP・NDT・GICP・VGICP が「何と何を比べるか」を同じ場面で図にする |
| `ex03b_icp_2d.py` | 点対点 ICP・点対線 ICP・GICP の比較 |
| `ex04_vgicp_2d.py` | ガウス分布ボクセル地図と VGICP、マルチ解像度 |
| `ex05_imu.py` | IMU のドリフト、事前積分、バイアスの 1 次補正 |
| `ex06_factor_graph.py` | ファクターグラフのスパース性、周辺化、固定ラグ平滑化 |
| `ex07_sliding_window_localization.py` | **論文の手法を 2D で再現** (地図の半分を消した環境で 3 手法を比較) |
| `optional/` | 著者の small_gicp、GTSAM の ImuFactor + iSAM2 を使う発展演習 (要追加インストール) |

演習 7 の結果 (地図が無い右側の領域でも提案手法は推定を続け、地図に戻ると誤差が数 mm〜数 cm に戻る):

![2D 再現実験](docs/figures/ex07_sliding_window_localization.png)

## ディレクトリ構成

```
.
├── docs/                 解説 (Markdown) と図 (docs/figures)
├── html/                 HTML 版 (tools/build_html.py で docs/ から生成)
├── exercises/            Python 演習 (numpy / scipy / matplotlib)
│   └── optional/         発展演習 (small_gicp, gtsam)
├── tools/
│   ├── build_html.py     HTML 生成スクリプト
│   └── site/             HTML のスタイル・デモ・KaTeX
├── requirements.txt
├── Makefile
└── Dockerfile
```

## 注意

- 解説は論文 PDF の内容と、著者の公開コード (GLIM, gtsam_points; 2026 年 10 月時点) をもとに作成しています。
  細部は必ず原文と照らし合わせてください。
- `exercises/optional/` の 2 本は、作成環境でパッケージをインストールできなかったため動作未確認です。
  それ以外の演習はすべて実行して結果を確認済みです。
- HTML 版に同梱している KaTeX は MIT ライセンスです (`tools/site/katex/LICENSE.txt`)。
