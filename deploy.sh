#!/usr/bin/env bash
set -euo pipefail

PI_USER="${PI_USER:-aaron}"
PI_HOST="${PI_HOST:-atlas.local}"
PI_WORKSPACE="${PI_WORKSPACE:-/workspace}"

build_workspace() {
    if [ -f "/.dockerenv" ] || [ -f "/opt/ros/humble/setup.bash" ]; then
        colcon build \
            --symlink-install \
            --event-handlers console_cohesion+ \
            --cmake-args -G Ninja
    else
        LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) docker compose run --rm ros bash -lc \
            "cd /workspace && \
            source /opt/ros/humble/setup.bash && \
            if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && \
            colcon build --symlink-install --event-handlers console_cohesion+ --cmake-args -G Ninja"
    fi
}

build_workspace

echo "Syncing install/ to ${PI_USER}@${PI_HOST}:${PI_WORKSPACE}/install/"

rsync -avz --delete \
    install/ \
    "${PI_USER}@${PI_HOST}:${PI_WORKSPACE}/install/"

echo "Payload sent to payload :)"
