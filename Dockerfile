# Hunter Bipedal Robot Simulation Dockerfile
# Based on Ubuntu 22.04

FROM ubuntu:22.04

# 環境変数設定
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 作業ディレクトリ
WORKDIR /workspace

# システムパッケージのアップデートと必要なツールのインストール
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    git \
    wget \
    curl \
    vim \
    # PyBullet GUIに必要なパッケージ
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    # X11関連（GUI用）
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Python依存パッケージのインストール
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# プロジェクトファイルをコピー
COPY . /workspace/hunter

# 作業ディレクトリを設定
WORKDIR /workspace/hunter

# デフォルトコマンド
CMD ["/bin/bash"]
