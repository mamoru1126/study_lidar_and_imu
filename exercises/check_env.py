"""学習環境のチェック

実行: python exercises/check_env.py
必須パッケージと、発展演習で使う任意パッケージが入っているかを確認します。
"""

import importlib
import sys

print(f"Python {sys.version.split()[0]}")
ok = True
for name, required in [
    ("numpy", True),
    ("scipy", True),
    ("matplotlib", True),
    ("markdown", False),
    ("pymdownx", False),
    ("small_gicp", False),
    ("gtsam", False),
]:
    try:
        mod = importlib.import_module(name)
        ver = getattr(mod, "__version__", "")
        print(f"  [OK]   {name:12s} {ver}")
    except ImportError:
        tag = "必須" if required else "任意"
        print(f"  [なし] {name:12s} ({tag})")
        ok = ok and not required

try:
    from matplotlib import font_manager

    names = {f.name for f in font_manager.fontManager.ttflist}
    jp = [n for n in names if any(k in n for k in ["CJK JP", "Noto Sans JP", "IPA", "Hiragino", "Yu Gothic", "Meiryo", "Takao"])]
    print("  日本語フォント:", ", ".join(sorted(jp)[:3]) if jp else "見つかりません (図の日本語が□になります)")
except Exception:
    pass

print()
if ok:
    print("必須パッケージはそろっています。演習 ex01〜ex07 を実行できます。")
else:
    print("必須パッケージが足りません。 pip install -r requirements.txt を実行してください。")
print("任意: markdown/pymdownx → HTML 版の再生成, small_gicp / gtsam → exercises/optional の発展演習")
