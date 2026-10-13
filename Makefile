# 学習環境の便利コマンド
PY ?= python3

.PHONY: setup check exercises html all

setup:
	$(PY) -m pip install -r requirements.txt

check:
	$(PY) exercises/check_env.py

exercises:
	$(PY) exercises/ex01_rotation.py
	$(PY) exercises/ex02_least_squares.py
	$(PY) exercises/ex03_icp_2d.py
	$(PY) exercises/ex04_vgicp_2d.py
	$(PY) exercises/ex05_imu.py
	$(PY) exercises/ex06_factor_graph.py
	$(PY) exercises/ex07_sliding_window_localization.py

html:
	$(PY) tools/build_html.py

all: exercises html
