"""docs/*.md から HTML 版 (html/) を生成する

使い方:
    pip install markdown pymdown-extensions
    python tools/build_html.py

- Markdown の数式 ($...$, ```math) は KaTeX で表示します (tools/site/katex に同梱。オフラインでも表示できます)
- <!-- demo:NAME --> の位置にインタラクティブデモを埋め込みます (tools/site/demos.js)
- 図 (docs/figures) と共通アセット (tools/site) を html/ にコピーします
- html/data/sliding_window_result.js は演習 7 が生成します (このスクリプトでは消しません)
"""

from __future__ import annotations

import argparse
import html
import re
import shutil
from pathlib import Path

import markdown
from pymdownx import arithmatex, slugs

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
OUT = ROOT / "html"
SITE = Path(__file__).resolve().parent / "site"

SITE_NAME = "LiDAR×IMU 論文ノート"
REPO_URL = "https://github.com/mamoru1126/study_lidar_and_imu"
PAPER_URL = "https://staff.aist.go.jp/k.koide/assets/pdf/robomech2023.pdf"

# (ファイル名の stem, 短いタイトル, 要約, タグ, 目安時間)
CHAPTERS = [
    ("00_roadmap", "学習ロードマップ", "論文をひとことで。目的関数の全体像と、必要な知識の地図。", [], "15 分"),
    ("01_coordinates_and_rotation", "座標と回転", "回転行列、SO(3) と SE(3)、Exp と Log、摂動でヤコビアンを求める。", ["math"], "2〜3 時間"),
    ("02_probability_and_least_squares", "確率と最小二乗", "誤差ベクトル、正規分布の −1/2、共分散行列、最尤推定、ガウス・ニュートン法。", ["math", "opt"], "3 時間"),
    ("03_point_cloud_registration", "スキャンマッチング", "ICP・NDT・GICP・VGICP を図で比べる。NDT と GICP の違い。コストを因子にする意味。", ["lidar"], "3〜4 時間"),
    ("04_imu_and_preintegration", "IMU と事前積分", "比力とバイアス、ドリフトの速さ、ΔR・Δv・Δp による事前積分。", ["imu"], "3 時間"),
    ("05_factor_graph", "ファクターグラフ", "変数と因子、H のスパース性、GTSAM の使い方、iSAM2 の考え方。", ["opt"], "2 時間"),
    ("06_fixed_lag_smoothing", "固定ラグ平滑化と周辺化", "ウィンドウ最適化、シューア補行列、キーフレーム、密結合。", ["opt", "lidar", "imu"], "2〜3 時間"),
    ("07_map_based_localization", "既知地図上の自己位置推定", "NDT・UKF・MCL と提案手法の違い。2D 再現実験の結果。", ["map"], "1〜2 時間"),
    ("08_paper_walkthrough", "論文精読ガイド", "論文を節ごとに読む。式と因子の対応、実験の読み方。", ["lidar", "imu", "map"], "2 時間"),
    ("09_author_code_guide", "著者のコード案内", "GLIM・gtsam_points のどこに論文の要素があるか。", ["code"], "適宜"),
    ("10_glossary", "用語集", "日本語と英語の対応、一言での説明。", [], "辞書として"),
]
TAG_LABEL = {"math": ("math", "数学"), "opt": ("opt", "最適化"), "lidar": ("lidar", "LiDAR"), "imu": ("imu", "IMU"), "map": ("map", "地図"), "code": ("math", "コード")}

MATH = """<link rel="stylesheet" href="assets/katex/katex.css">
<script defer src="assets/katex/katex.min.js"></script>
<script defer src="assets/katex/auto-render.min.js"></script>"""

FONTS = (
    '<link rel="preconnect" href="https://fonts.googleapis.com">\n'
    '<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>\n'
    '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=BIZ+UDPGothic:wght@400;700'
    '&family=IBM+Plex+Mono:wght@400;500&family=Zen+Kaku+Gothic+New:wght@500;700&display=swap">'
)

BRAND_MARK = (
    '<svg class="brand-mark" viewBox="0 0 24 24" aria-hidden="true">'
    '<circle cx="6" cy="18" r="2.6" fill="var(--imu)"/>'
    '<path d="M6 11a7 7 0 0 1 7 7" fill="none" stroke="var(--lidar)" stroke-width="2" stroke-linecap="round"/>'
    '<path d="M6 5a13 13 0 0 1 13 13" fill="none" stroke="var(--lidar)" stroke-width="2" stroke-linecap="round" opacity=".55"/>'
    "</svg>"
)


def md_converter():
    return markdown.Markdown(
        extensions=[
            "tables",
            "md_in_html",
            "attr_list",
            "sane_lists",
            "toc",
            "pymdownx.arithmatex",
            "pymdownx.superfences",
            "pymdownx.tasklist",
        ],
        extension_configs={
            "toc": {"slugify": slugs.slugify(case="lower"), "permalink": False},
            "pymdownx.arithmatex": {"generic": True},
            "pymdownx.superfences": {
                "custom_fences": [
                    {"name": "math", "class": "arithmatex", "format": arithmatex.arithmatex_fenced_format(which="generic")}
                ]
            },
            "pymdownx.tasklist": {"custom_checkbox": False},
        },
    )


def preprocess(text: str) -> str:
    # 末尾の「前へ / 次へ」ナビゲーションはテンプレート側で作るので取り除く
    idx = text.rfind("\n---\n")
    if idx >= 0 and re.search(r"(前へ|次へ|目次)", text[idx:]):
        text = text[:idx] + "\n"
    # 「HTML 版では…」の注記は HTML では不要
    text = re.sub(r"^> \*\*HTML 版\*\*.*\n(?:>.*\n)*", "", text, flags=re.M)
    # デモの埋め込み位置
    text = re.sub(r"<!-- demo:(\w+) -->", r'<div data-demo="\1"></div>', text)
    # リンクの書き換え
    text = re.sub(r"\]\((\d\d_[\w]+)\.md(#[^)]*)?\)", lambda m: f"]({m.group(1)}.html{m.group(2) or ''})", text)
    text = text.replace("](../README.md)", "](index.html)")
    text = re.sub(r"\]\(\.\./exercises/[^)]*\)", lambda m: m.group(0).replace("../", REPO_URL + "/blob/master/"), text)
    return text


def chips(tags, time_label=None):
    out = []
    for t in tags:
        cls, label = TAG_LABEL[t]
        out.append(f'<span class="chip {cls}">{label}</span>')
    if time_label:
        out.append(f'<span class="chip time">目安 {html.escape(time_label)}</span>')
    return "".join(out)


def rail(current: str | None) -> str:
    items = []
    for stem, title, _, _, _ in CHAPTERS:
        num = stem[:2]
        cur = ' aria-current="page"' if stem == current else ""
        items.append(
            f'<li><a href="{stem}.html"{cur}><span class="num">{num}</span><span>{html.escape(title)}</span>'
            f'<span class="done" data-done-slug="{stem}" title="読了"></span></a></li>'
        )
    home_cur = ' aria-current="page"' if current is None else ""
    return (
        '<nav class="rail" aria-label="章の一覧">'
        '<h2>目次</h2>'
        f'<ul class="toc"><li><a href="index.html"{home_cur}>'
        '<span class="num">◆</span><span>はじめに</span><span></span></a></li>' + "".join(items) + "</ul>"
        '<h2>リンク</h2>'
        f'<p class="rail-note"><a href="{PAPER_URL}" target="_blank" rel="noopener">論文 PDF</a><br>'
        f'<a href="{REPO_URL}" target="_blank" rel="noopener">このリポジトリ</a><br>'
        '<a href="https://github.com/koide3" target="_blank" rel="noopener">著者の GitHub (koide3)</a></p>'
        '<p class="rail-note">各章の最後で「読んだ」にチェックすると、ここに印が付きます (このブラウザにだけ保存)。</p>'
        "</nav>"
    )


def page(title: str, body: str, current: str | None, extra_scripts: str = "") -> str:
    return f"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>{html.escape(title)}</title>
{FONTS}
<link rel="stylesheet" href="assets/style.css">
{MATH}
</head>
<body>
<header class="topbar">
  <a class="brand" href="index.html">{BRAND_MARK}<span>{SITE_NAME}</span></a>
  <button class="menu-btn" type="button" aria-expanded="false" aria-label="目次を開く">目次</button>
</header>
<div class="layout">
{rail(current)}
<main>
{body}
</main>
</div>
{extra_scripts}<script src="assets/app.js"></script>
<script src="assets/demos.js"></script>
</body>
</html>
"""


def build_chapter(i: int):
    stem, title, summary, tags, time_label = CHAPTERS[i]
    text = (DOCS / f"{stem}.md").read_text(encoding="utf-8")
    md = md_converter()
    content = md.convert(preprocess(text))
    prev_link = next_link = ""
    if i > 0:
        p = CHAPTERS[i - 1]
        prev_link = f'<a class="prev" href="{p[0]}.html"><small>前へ</small>{p[0][:2]} {html.escape(p[1])}</a>'
    else:
        prev_link = '<a class="prev" href="index.html"><small>前へ</small>はじめに</a>'
    if i < len(CHAPTERS) - 1:
        n = CHAPTERS[i + 1]
        next_link = f'<a class="next" href="{n[0]}.html"><small>次へ</small>{n[0][:2]} {html.escape(n[1])}</a>'
    body = f"""<article class="article">
<div class="chapter-meta"><span class="chip time">第 {stem[:2]} 章</span>{chips(tags, time_label)}</div>
{content}
<label class="read-toggle"><input type="checkbox" id="read-toggle" data-slug="{stem}"> この章を読んだ</label>
<nav class="pager" aria-label="前後の章">{prev_link}{next_link}</nav>
</article>"""
    extra = ""
    if 'data-demo="replay"' in content:
        extra = '<script src="data/sliding_window_result.js"></script>\n'
    out = page(f"{title} | {SITE_NAME}", body, stem, extra)
    (OUT / f"{stem}.html").write_text(out, encoding="utf-8")


def build_index():
    cards = []
    for stem, title, summary, tags, time_label in CHAPTERS:
        cards.append(
            f'<a class="card" href="{stem}.html"><span class="n"><span>CHAPTER {stem[:2]}</span>'
            f'<span class="done-badge" data-done-badge="{stem}" hidden>読了</span></span>'
            f'<span class="ttl">{html.escape(title)}</span><span class="sum">{html.escape(summary)}</span>'
            f'<span class="chips">{chips(tags, time_label)}</span></a>'
        )
    body = f"""<div class="article" style="max-width:none">
<section class="hero">
  <div>
    <div class="eyebrow">ROBOMECH2023 を読むための学習ノート</div>
    <h1>LiDAR と IMU で、欠けた地図の上でも迷わない自己位置推定を理解する</h1>
    <p>この教材は、点群と IMU の制約をウィンドウ最適化でまとめて解く自己位置推定の論文を、
    数学の道具から順に積み上げて読み解くためのものです。各章に numpy だけで動く演習があり、
    最後は論文の手法を 2D で再現します。</p>
    <div class="paper-card">
      <div class="t">点群・IMU 制約のウィンドウ最適化に基づく既知地図上での 6DoF 姿勢推定</div>
      <div class="a">小出 健司, 大石 修士, 横塚 将志, 阪野 貴彦 (産総研) / ROBOMECH2023</div>
      <div><a href="{PAPER_URL}" target="_blank" rel="noopener">論文 PDF を開く</a> ・ <a href="00_roadmap.html">ロードマップから始める</a></div>
    </div>
  </div>
  <div class="scan-hero">
    <canvas id="hero-scan" aria-label="2D LiDAR のスキャンと推定軌跡のアニメーション"></canvas>
    <p>2D 再現実験の 1 シーン。灰色の点が事前地図で、右側の色付きの領域は地図を消してある。青い線が提案手法の推定軌跡、放射状の線が LiDAR のスキャン。</p>
  </div>
</section>

<section class="section">
  <h2>論文の中心にある 1 本の式</h2>
  <p class="lead">ウィンドウ内の状態 X について、4 種類の誤差の和を最小にします。</p>
  <div class="arithmatex">\\[
F(\\mathcal{{X}}) = \\sum_{{\\mathbf{{x}}_i \\in \\mathcal{{X}}}} \\sum_{{j=i-2}}^{{i-1}} f^{{P}}(\\mathbf{{x}}_i, \\mathbf{{x}}_j)
 + \\sum_{{\\mathbf{{x}}_i \\in \\mathcal{{X}}}} f^{{P}}(\\mathbf{{x}}_i, \\mathcal{{M}})
 + \\sum_{{\\mathbf{{x}}_i \\in \\mathcal{{X}}}} f^{{I}}(\\mathbf{{x}}_{{i-1}}, \\mathbf{{x}}_i)
 + C(\\mathcal{{X}})
\\]</div>
  <div class="term-grid">
    <div class="term" style="--c:var(--lidar)"><b>フレーム同士のマッチング</b>最新フレームを直前の 2 フレームと VGICP で比べる。→ 03 章</div>
    <div class="term" style="--c:var(--map)"><b>地図とのマッチング</b>フレームを既知の地図点群と比べる。地図がない場所では自然に効かなくなる。→ 07 章</div>
    <div class="term" style="--c:var(--imu)"><b>IMU 事前積分</b>連続する状態の動きを IMU と整合させる。バイアスも推定。→ 04 章</div>
    <div class="term" style="--c:var(--muted)"><b>周辺化因子</b>ウィンドウから出た過去の情報を残す。→ 06 章</div>
  </div>
</section>

<section class="section">
  <h2>進め方</h2>
  <ol class="steps">
    <li>論文を一度ざっと読み、分からない言葉に印をつける。</li>
    <li>01〜07 章を順に読み、各章の Python 演習を実行する。数式は演習の数値で確かめる。</li>
    <li>08 章を横に置いて論文を読み直す。印をつけた言葉を全部説明できるか確認する。</li>
    <li>余裕があれば 09 章を頼りに、著者の GLIM のコードで同じ部品を探す。</li>
  </ol>
</section>

<section class="section">
  <h2>章の一覧</h2>
  <p class="lead">色の印は、その章で主に扱うものです。LiDAR は青、IMU はオレンジ、地図は緑、最適化は紫。</p>
  <div class="cards">{''.join(cards)}</div>
</section>

<section class="section">
  <h2>演習の環境</h2>
  <p class="lead">ROS も GPU も不要です。Python 3.9 以上と numpy / scipy / matplotlib があれば全部動きます。</p>
<pre><code>git clone {REPO_URL}.git
cd study_lidar_and_imu
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python exercises/check_env.py
python exercises/ex07_sliding_window_localization.py   # 論文の手法の 2D 再現</code></pre>
</section>

<footer class="site">
  Markdown 版は <a href="{REPO_URL}/tree/master/docs" target="_blank" rel="noopener">docs/</a> にあります。
  この HTML は <code>python tools/build_html.py</code> で docs/ から生成しています。
</footer>
</div>"""
    out = page(SITE_NAME, body, None, '<script src="data/sliding_window_result.js"></script>\n')
    (OUT / "index.html").write_text(out, encoding="utf-8")


def artifact_entry(path: Path):
    """Artifact として公開するとき用: doctype / html / head / body のタグを外した index を書き出す"""
    src = (OUT / "index.html").read_text(encoding="utf-8")
    head = re.search(r"<head>(.*)</head>", src, re.S).group(1)
    head = re.sub(r'<meta charset="utf-8">\s*', "", head)
    head = re.sub(r"<meta name=\"viewport\"[^>]*>\s*", "", head)
    body = re.search(r"<body>(.*)</body>", src, re.S).group(1)
    path.write_text(head.strip() + "\n" + body.strip() + "\n", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--artifact-entry", type=Path, help="Artifact 公開用の index を書き出す先")
    args = ap.parse_args()

    OUT.mkdir(exist_ok=True)
    if (OUT / "assets").exists():
        shutil.rmtree(OUT / "assets")
    shutil.copytree(SITE, OUT / "assets")
    if (OUT / "figures").exists():
        shutil.rmtree(OUT / "figures")
    shutil.copytree(DOCS / "figures", OUT / "figures")
    for i in range(len(CHAPTERS)):
        build_chapter(i)
    build_index()
    if not (OUT / "data" / "sliding_window_result.js").exists():
        print("[注意] html/data/sliding_window_result.js がありません。先に exercises/ex07_sliding_window_localization.py を実行してください。")
    if args.artifact_entry:
        artifact_entry(args.artifact_entry)
    print(f"HTML を生成しました: {OUT / 'index.html'} ({len(CHAPTERS)} 章)")


if __name__ == "__main__":
    main()
