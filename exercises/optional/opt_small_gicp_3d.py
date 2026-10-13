"""発展演習 A: 著者 (koide3) の small_gicp で 3D 点群の位置合わせを試す

必要なもの: pip install small_gicp
対応する資料: docs/03_point_cloud_registration.md, docs/09_author_code_guide.md

small_gicp は論文著者の小出さんが公開している、高速な点群位置合わせライブラリです
(fast_gicp の後継)。ICP / 点対面 ICP / GICP / VGICP を同じインターフェースで試せます。

ここでは「箱がいくつか置かれた部屋」の 3D 点群を人工的に作り、
既知の変換でずらしたコピーとの位置合わせを各手法で比べます。

注意: このスクリプトは教材作成環境では small_gicp をインストールできなかったため、
      README に記載の API に従って書いていますが、動作確認はできていません。
      動かない場合は https://github.com/koide3/small_gicp の README を確認してください。
"""

import time

import numpy as np

try:
    import small_gicp
except ImportError:
    raise SystemExit("small_gicp が見つかりません。 pip install small_gicp を実行してください。")


def rot(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def make_room(rng, n_per_m2=60):
    """床・4 面の壁・箱 3 つの表面に点を撒いた 3D 点群"""
    planes = []  # (原点, u ベクトル, v ベクトル)
    W, D, H = 12.0, 8.0, 3.0
    planes.append(((0, 0, 0), (W, 0, 0), (0, D, 0)))  # 床
    planes.append(((0, 0, 0), (W, 0, 0), (0, 0, H)))  # 壁
    planes.append(((0, D, 0), (W, 0, 0), (0, 0, H)))
    planes.append(((0, 0, 0), (0, D, 0), (0, 0, H)))
    planes.append(((W, 0, 0), (0, D, 0), (0, 0, H)))
    for cx, cy, s, h in [(3, 2, 1.0, 1.2), (8, 5, 1.5, 0.8), (6, 1.5, 0.8, 2.0)]:
        x0, y0 = cx - s / 2, cy - s / 2
        planes += [
            ((x0, y0, 0), (s, 0, 0), (0, 0, h)),
            ((x0, y0 + s, 0), (s, 0, 0), (0, 0, h)),
            ((x0, y0, 0), (0, s, 0), (0, 0, h)),
            ((x0 + s, y0, 0), (0, s, 0), (0, 0, h)),
            ((x0, y0, h), (s, 0, 0), (0, s, 0)),
        ]
    pts = []
    for o, u, v in planes:
        o, u, v = map(np.asarray, (o, u, v))
        area = np.linalg.norm(np.cross(u, v))
        a, b = rng.random((2, int(area * n_per_m2)))
        pts.append(o + a[:, None] * u + b[:, None] * v)
    pts = np.vstack(pts)
    return pts + rng.normal(scale=0.01, size=pts.shape)


rng = np.random.default_rng(0)
target = make_room(rng)

# 正解の変換 T_target_source (ソース座標 → ターゲット座標)
T_true = np.eye(4)
T_true[:3, :3] = rot([0.1, 0.2, 1.0], 0.15)
T_true[:3, 3] = [0.4, -0.3, 0.05]
# ソース = ターゲットを T_true^{-1} で動かし、別のノイズを足したもの
Tinv = np.linalg.inv(T_true)
source = target @ Tinv[:3, :3].T + Tinv[:3, 3]
keep = rng.random(len(source)) < 0.7  # 3 割の点を落として、ターゲットと点の並びを変える
source = source[keep] + rng.normal(scale=0.01, size=(int(keep.sum()), 3))
print(f"ターゲット {len(target)} 点, ソース {len(source)} 点")

for reg in ["ICP", "PLANE_ICP", "GICP", "VGICP"]:
    t0 = time.perf_counter()
    result = small_gicp.align(
        target,
        source,
        init_T_target_source=np.eye(4),
        registration_type=reg,
        downsampling_resolution=0.15,
        voxel_resolution=0.5,
        max_correspondence_distance=1.0,
        num_threads=4,
    )
    dt = time.perf_counter() - t0
    T = result.T_target_source
    t_err = np.linalg.norm(T[:3, 3] - T_true[:3, 3])
    r_err = np.degrees(np.arccos(np.clip((np.trace(T[:3, :3].T @ T_true[:3, :3]) - 1) / 2, -1, 1)))
    print(
        f"{reg:10s}: 並進誤差 {t_err*100:6.2f} cm, 回転誤差 {r_err:5.2f} deg, "
        f"反復 {result.iterations:3d}, 収束 {result.converged}, 時間 {dt*1000:7.1f} ms"
    )

print(
    """
やってみよう
  - downsampling_resolution を変えると、速度と精度はどう変わる?
  - T_true の回転を大きく (0.15 → 0.6 rad) すると、どの手法が先に失敗する?
  - result.H (6x6 のヘッセ行列) の固有値を見ると、拘束の弱い方向が分かる
"""
)
