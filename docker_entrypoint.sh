#!/usr/bin/env bash
set -e

# Host UID/GID passed via docker compose env
USER_ID="${LOCAL_UID:-1000}"
GROUP_ID="${LOCAL_GID:-1000}"
USER_NAME="${LOCAL_USER:-rosdev}"

if ! getent group "${GROUP_ID}" >/dev/null 2>&1; then
  groupadd -g "${GROUP_ID}" "${USER_NAME}" >/dev/null 2>&1 || true
fi

if ! id -u "${USER_NAME}" >/dev/null 2>&1; then
  useradd -m -u "${USER_ID}" -g "${GROUP_ID}" -s /bin/bash "${USER_NAME}" >/dev/null 2>&1 || true
fi

echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}"
chmod 0440 "/etc/sudoers.d/${USER_NAME}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

exec gosu "${USER_ID}:${GROUP_ID}" "$@"

