#!/usr/bin/env bash
# Boot wrapper for the ATLAS flight stack (run by atlas-flight.service).
#
# Picks the active flight directory:
#  - reuses the most recent dir under $FLIGHT_BASE unless it contains
#    new_dir_please.flag (written by captain on Command_NewFlightDanger)
#  - creates a fresh timestamped dir on first boot or when flagged
# then launches flight.launch.py with it.
set -euo pipefail

FLIGHT_BASE="${FLIGHT_BASE:-/home/aaron/flights}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE="${ATLAS_WORKSPACE:-/home/aaron/atlas_ws}"
FLAG_NAME="new_dir_please.flag"

mkdir -p "${FLIGHT_BASE}"

latest_dir=""
# flight dirs are named flight_YYYYMMDD_HHMMSS so lexicographic sort is chronological
for d in "${FLIGHT_BASE}"/flight_*/; do
    [[ -d "${d}" ]] && latest_dir="${d%/}"
done

need_new=0
if [[ -z "${latest_dir}" ]]; then
    echo "[flight_boot] No existing flight dir under ${FLIGHT_BASE}"
    need_new=1
elif [[ -e "${latest_dir}/${FLAG_NAME}" ]]; then
    echo "[flight_boot] ${latest_dir} is flagged for rotation"
    rm -f "${latest_dir}/${FLAG_NAME}"
    need_new=1
fi

if [[ "${need_new}" -eq 1 ]]; then
    flight_dir="${FLIGHT_BASE}/flight_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "${flight_dir}"
    echo "[flight_boot] Created new flight dir: ${flight_dir}"
else
    flight_dir="${latest_dir}"
    echo "[flight_boot] Reusing flight dir: ${flight_dir}"
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    source "${WORKSPACE}/install/setup.bash"
fi
set -u

exec ros2 launch cubesat_bringup flight.launch.py "flight_dir:=${flight_dir}"
