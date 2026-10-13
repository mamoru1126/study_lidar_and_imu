# 演習用の最小環境 (ROS・GPU なし)
#   docker build -t study-lidar-imu .
#   docker run --rm -it -v "$PWD":/work study-lidar-imu make exercises
FROM python:3.11-slim

RUN apt-get update \
    && apt-get install -y --no-install-recommends make fonts-noto-cjk \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

CMD ["bash"]
