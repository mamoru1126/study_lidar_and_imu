"""演習スクリプト共通のユーティリティ

- 図の保存先 (docs/figures) の管理
- 日本語フォントの自動設定
- 2D / 3D の回転・姿勢を扱う小さな関数群
- 図で使う色 (教材全体で統一)
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# 出力先
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parent.parent
FIG_DIR = REPO_ROOT / "docs" / "figures"
DATA_DIR = REPO_ROOT / "html" / "data"


def fig_path(name: str) -> Path:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    return FIG_DIR / name


# ---------------------------------------------------------------------------
# 色 (教材全体で統一)
#   LiDAR 同士のマッチング = 青, 地図マッチング = 水色寄りの緑, IMU = オレンジ
# ---------------------------------------------------------------------------
C_INK = "#1f2733"      # 真値・文字
C_MUTED = "#8a93a3"    # 補助線・地図点群
C_BLUE = "#2a78d6"     # 提案手法 / スキャン間マッチング
C_ORANGE = "#eb6834"   # 比較手法 / IMU
C_AQUA = "#1baf7a"     # 3 つ目の系列 / 地図マッチング
C_SHADE = "#f3d9cf"    # 地図が無い領域の背景


def setup_matplotlib():
    """matplotlib を非対話モードにし、日本語フォントを探して設定する。"""
    import matplotlib

    if os.environ.get("MPLBACKEND") is None:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    candidates = [
        "Noto Sans CJK JP",
        "Noto Sans JP",
        "IPAexGothic",
        "IPAPGothic",
        "IPAGothic",
        "Hiragino Sans",
        "Hiragino Kaku Gothic ProN",
        "Yu Gothic",
        "Meiryo",
        "MS Gothic",
        "TakaoGothic",
        "VL Gothic",
    ]
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available:
            plt.rcParams["font.family"] = name
            break
    else:
        print(
            "[注意] 日本語フォントが見つかりませんでした。図の日本語が□になる場合は\n"
            "  Ubuntu: sudo apt install fonts-noto-cjk  /  または pip install japanize-matplotlib\n"
            "  を実行してください。"
        )
    plt.rcParams.update(
        {
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "grid.color": "#e3e6eb",
            "grid.linewidth": 0.6,
            "axes.edgecolor": "#9aa3b0",
            "axes.labelcolor": C_INK,
            "xtick.color": "#4a5260",
            "ytick.color": "#4a5260",
            "figure.dpi": 110,
            "savefig.dpi": 130,
            "savefig.bbox": "tight",
            "legend.frameon": False,
            "lines.linewidth": 2.0,
            "axes.unicode_minus": False,
        }
    )
    return plt


# ---------------------------------------------------------------------------
# 2D の回転・姿勢
# ---------------------------------------------------------------------------
def rot2(theta: float) -> np.ndarray:
    """2D 回転行列 R(θ)"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def drot2(theta: float) -> np.ndarray:
    """dR(θ)/dθ"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[-s, -c], [c, -s]])


def wrap_angle(a):
    """角度を [-π, π) に丸める"""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# 3D の回転 (SO(3))
# ---------------------------------------------------------------------------
def hat(w: np.ndarray) -> np.ndarray:
    """ベクトル w ∈ R^3 を歪対称行列 [w]× に変換する (hat 演算子)"""
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def vee(W: np.ndarray) -> np.ndarray:
    """歪対称行列からベクトルを取り出す (vee 演算子, hat の逆)"""
    return np.array([W[2, 1], W[0, 2], W[1, 0]])


def so3_exp(w: np.ndarray) -> np.ndarray:
    """指数写像 Exp: R^3 -> SO(3) (ロドリゲスの公式)"""
    theta = np.linalg.norm(w)
    if theta < 1e-10:
        return np.eye(3) + hat(w)
    K = hat(w / theta)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * K @ K


def so3_log(R: np.ndarray) -> np.ndarray:
    """対数写像 Log: SO(3) -> R^3"""
    cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < 1e-10:
        return vee(R - R.T) / 2.0
    if np.pi - theta < 1e-6:
        # θ ≈ π の特異点付近: R ≈ 2 a a^T - I から回転軸 a を取り出す
        A = (R + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(A)))
        a = A[:, i] / np.sqrt(A[i, i])
        return theta * a / np.linalg.norm(a)
    return theta / (2.0 * np.sin(theta)) * vee(R - R.T)


def so3_right_jacobian(w: np.ndarray) -> np.ndarray:
    """右ヤコビアン Jr(w)。 Exp(w + δ) ≈ Exp(w) Exp(Jr(w) δ)"""
    theta = np.linalg.norm(w)
    if theta < 1e-8:
        return np.eye(3) - 0.5 * hat(w)
    K = hat(w)
    return (
        np.eye(3)
        - (1.0 - np.cos(theta)) / theta**2 * K
        + (theta - np.sin(theta)) / theta**3 * K @ K
    )


def se3(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """回転 R と並進 t から 4x4 の同次変換行列を作る"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def se3_inv(T: np.ndarray) -> np.ndarray:
    """同次変換行列の逆行列 (R^T, -R^T t)"""
    R, t = T[:3, :3], T[:3, 3]
    return se3(R.T, -R.T @ t)


def transform(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """点群 pts (N,3) に同次変換 T を適用する"""
    return pts @ T[:3, :3].T + T[:3, 3]
