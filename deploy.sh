#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
PI_SSH_TARGET="${PI_SSH_TARGET:-atlas}"
PI_REMOTE_USER="${PI_REMOTE_USER:-aaron}"
PI_WORKSPACE="${PI_WORKSPACE:-/home/${PI_REMOTE_USER}/atlas_ws}"
PI_ARCH="${PI_ARCH:-}"
PI_RUN_CMD="${PI_RUN_CMD:-}"
DOCKER_IMAGE="${DOCKER_IMAGE:-atlas-deploy:${ROS_DISTRO}}"
DEPLOY_BUILD_BASE="${DEPLOY_BUILD_BASE:-build_pi}"
DEPLOY_INSTALL_BASE="${DEPLOY_INSTALL_BASE:-install_pi}"
DEPLOY_LOG_BASE="${DEPLOY_LOG_BASE:-log_pi}"

usage() {
    cat <<EOF
Usage: ./deploy.sh [--run "remote command"]

Environment overrides:
  PI_SSH_TARGET     SSH target to reach the Pi (default: atlas)
  PI_REMOTE_USER    Remote username used for the default workspace path (default: aaron)
  PI_WORKSPACE      Remote workspace root (default: /home/\$PI_REMOTE_USER/atlas_ws)
  PI_ARCH           Override remote architecture detection (example: aarch64)
  PI_RUN_CMD        Remote command to run after sync
  ROS_DISTRO        ROS distro to source in the build and on the Pi (default: humble)
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run)
            [[ $# -ge 2 ]] || {
                echo "Missing value for --run" >&2
                exit 1
            }
            PI_RUN_CMD="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || {
        echo "Missing required command: $1" >&2
        exit 1
    }
}

require_cmd docker
require_cmd rsync
require_cmd ssh

ssh_remote() {
    ssh "${PI_SSH_TARGET}" "$@"
}

detect_remote_arch() {
    if [[ -n "${PI_ARCH}" ]]; then
        printf '%s\n' "${PI_ARCH}"
        return
    fi

    if ! ssh_remote uname -m; then
        cat >&2 <<EOF
Unable to query the Pi architecture through SSH.

Fix local SSH first, or rerun with PI_ARCH set explicitly:
  PI_ARCH=aarch64 ./deploy.sh
EOF
        exit 1
    fi
}

docker_platform_for_arch() {
    case "$1" in
        aarch64|arm64)
            printf '%s\n' "linux/arm64"
            ;;
        armv7l|armv7*|armhf)
            cat >&2 <<EOF
Remote machine reports '$1'.

This deploy flow currently targets a 64-bit Pi userspace because the Docker image and ROS 2 Humble
dependencies in this repo are set up for amd64/arm64 builds. Use a 64-bit OS on the Pi
(uname -m should report aarch64) or build directly on the Pi instead of cross-deploying.
EOF
            exit 1
            ;;
        *)
            echo "Unsupported remote architecture: $1" >&2
            exit 1
            ;;
    esac
}

verify_remote_runtime() {
    if ! ssh_remote "test -f /opt/ros/${ROS_DISTRO}/setup.bash"; then
        cat >&2 <<EOF
ROS ${ROS_DISTRO} does not appear to be installed on the Pi.

Install the target runtime first, then rerun deploy. This repo already includes
local_installer.sh for machine setup.
EOF
        exit 1
    fi

    if ! ssh_remote "command -v rsync >/dev/null 2>&1"; then
        cat >&2 <<EOF
rsync is not installed on the Pi.

Install it first:
  sudo apt update && sudo apt-get install -y rsync
EOF
        exit 1
    fi
}

build_deploy_workspace() {
    local docker_platform="$1"

    rm -rf "${DEPLOY_BUILD_BASE}" "${DEPLOY_INSTALL_BASE}" "${DEPLOY_LOG_BASE}"

    docker buildx build \
        --load \
        --platform "${docker_platform}" \
        --tag "${DOCKER_IMAGE}" \
        .

    docker run --rm \
        --platform "${docker_platform}" \
        -e LOCAL_UID="$(id -u)" \
        -e LOCAL_GID="$(id -g)" \
        -e LOCAL_USER=rosdev \
        -e ROS_DISTRO="${ROS_DISTRO}" \
        -v "${PWD}:/workspace:rw" \
        -w /workspace \
        "${DOCKER_IMAGE}" \
        bash -lc "
            set -euo pipefail
            source /opt/ros/${ROS_DISTRO}/setup.bash
            colcon build \
                --build-base ${DEPLOY_BUILD_BASE} \
                --install-base ${DEPLOY_INSTALL_BASE} \
                --log-base ${DEPLOY_LOG_BASE} \
                --merge-install \
                --event-handlers console_cohesion+ \
                --cmake-args -G Ninja
        "
}

sync_to_pi() {
    ssh_remote "mkdir -p '${PI_WORKSPACE}/install'"

    echo "Syncing ${DEPLOY_INSTALL_BASE}/ to ${PI_SSH_TARGET}:${PI_WORKSPACE}/install/"

    rsync -az --delete \
        "${DEPLOY_INSTALL_BASE}/" \
        "${PI_SSH_TARGET}:${PI_WORKSPACE}/install/"
}

run_on_pi() {
    [[ -n "${PI_RUN_CMD}" ]] || return 0

    local remote_cmd
    remote_cmd="
        set -euo pipefail
        source /opt/ros/${ROS_DISTRO}/setup.bash
        source '${PI_WORKSPACE}/install/setup.bash'
        ${PI_RUN_CMD}
    "

    echo "Running on ${PI_SSH_TARGET}: ${PI_RUN_CMD}"
    ssh_remote bash -lc "$(printf '%q' "${remote_cmd}")"
}

REMOTE_ARCH="$(detect_remote_arch)"
DOCKER_PLATFORM="$(docker_platform_for_arch "${REMOTE_ARCH}")"

echo "Remote architecture: ${REMOTE_ARCH}"
echo "Docker platform: ${DOCKER_PLATFORM}"

verify_remote_runtime
build_deploy_workspace "${DOCKER_PLATFORM}"
sync_to_pi
run_on_pi

echo "Payload sent to payload :)"
