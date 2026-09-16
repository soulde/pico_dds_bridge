#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    pkg-config \
    bison \
    flex \
    python3 \
    python3-pip \
    python3-venv \
    nlohmann-json3-dev
