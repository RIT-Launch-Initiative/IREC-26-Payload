#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
PI_SSH_TARGET="${PI_SSH_TARGET:-atlas}"
PI_REMOTE_USER="${PI_REMOTE_USER:-aaron}"
PI_WORKSPACE="${PI_WORKSPACE:-/home/${PI_REMOTE_USER}/atlas_ws}"
PI_ARCH="${PI_ARCH:-}"
PI_RUN_CMD="${PI_RUN_CMD:-}"
DOCKER_IMAGE="${DOCKER_IMAGE:-atlas-deploy:${ROS_DISTRO}}"
DEPLOY_BUILDER="${DEPLOY_BUILDER:-atlas-arm64}"
DEPLOY_DOCKERFILE="${DEPLOY_DOCKERFILE:-Dockerfile.deploy}"
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
  DEPLOY_BUILDER    docker buildx builder to use for ARM builds (default: atlas-arm64)
  DEPLOY_DOCKERFILE Dockerfile used for deploy builds (default: Dockerfile.deploy)
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

ensure_builder_supports_platform() {
    local docker_platform="$1"
    local platforms

    if ! platforms="$(docker buildx inspect "${DEPLOY_BUILDER}" --bootstrap 2>/dev/null | awk -F': ' '/Platforms:/ {print $2}')"; then
        cat >&2 <<EOF
Docker buildx builder '${DEPLOY_BUILDER}' is not available.

Create and bootstrap one that supports ARM64 first:
  docker run --privileged --rm tonistiigi/binfmt --install arm64
  docker buildx create --name ${DEPLOY_BUILDER} --driver docker-container --use
  docker buildx inspect --bootstrap
EOF
        exit 1
    fi

    if [[ "${platforms}" != *"${docker_platform}"* ]]; then
        cat >&2 <<EOF
Docker buildx builder '${DEPLOY_BUILDER}' does not support ${docker_platform}.

Reported platforms: ${platforms}

Install ARM64 emulation and re-bootstrap the builder:
  docker run --privileged --rm tonistiigi/binfmt --install arm64
  docker buildx inspect ${DEPLOY_BUILDER} --bootstrap
EOF
        exit 1
    fi
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
        --builder "${DEPLOY_BUILDER}" \
        --file "${DEPLOY_DOCKERFILE}" \
        --load \
        --platform "${docker_platform}" \
        --tag "${DOCKER_IMAGE}" \
        .

    docker run --rm \
        --entrypoint bash \
        --platform "${docker_platform}" \
        --user "$(id -u):$(id -g)" \
        -e ROS_DISTRO="${ROS_DISTRO}" \
        -v "${PWD}:/workspace:rw" \
        -w /workspace \
        "${DOCKER_IMAGE}" \
        -lc "
            set -euo pipefail
            set +u
            source /opt/ros/${ROS_DISTRO}/setup.bash
            set -u
            colcon --log-base ${DEPLOY_LOG_BASE} build \
                --build-base ${DEPLOY_BUILD_BASE} \
                --install-base ${DEPLOY_INSTALL_BASE} \
                --merge-install \
                --event-handlers console_cohesion+ \
                --cmake-args -G Ninja -DCMAKE_POLICY_DEFAULT_CMP0148=OLD
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
        set +u
        source /opt/ros/${ROS_DISTRO}/setup.bash
        source '${PI_WORKSPACE}/install/setup.bash'
        set -u
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
ensure_builder_supports_platform "${DOCKER_PLATFORM}"
build_deploy_workspace "${DOCKER_PLATFORM}"
sync_to_pi
run_on_pi

echo "Deploy complete."
