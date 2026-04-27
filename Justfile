set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

local_uid := `id -u`
local_gid := `id -g`

default:
  @just --list

# Bring up Docker container
up:
  LOCAL_UID={{local_uid}} LOCAL_GID={{local_gid}} docker compose up --build -d

# Tear down Docker container
down:
  LOCAL_UID={{local_uid}} LOCAL_GID={{local_gid}} docker compose down

# Build the Docker image
build-image:
  LOCAL_UID={{local_uid}} LOCAL_GID={{local_gid}} docker compose build

# Run a shell inside the container
shell:
  LOCAL_UID={{local_uid}} LOCAL_GID={{local_gid}} docker compose exec ros bash

exec: shell

# View container logs
logs:
  LOCAL_UID={{local_uid}} LOCAL_GID={{local_gid}} docker compose logs -f ros

# Build the ROS workspace
build:
  if [ -f "/.dockerenv" ] || [ -f "/opt/ros/humble/setup.bash" ]; then colcon build --symlink-install --event-handlers console_cohesion+ --cmake-args -G Ninja; else LOCAL_UID={{local_uid}} LOCAL_GID={{local_gid}} docker compose run --rm ros bash -lc 'cd /workspace && source /opt/ros/humble/setup.bash && if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && colcon build --symlink-install --event-handlers console_cohesion+ --cmake-args -G Ninja'; fi

# Build with compile commands for clangd
compile-commands:
  if [ ! -f "/.dockerenv" ] && [ ! -f "/opt/ros/humble/setup.bash" ]; then echo "Run this inside the container or on the target machine."; exit 1; fi
  colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja

# Build with symlinks for development
colcon:
  colcon build --symlink-install

# Blast build artifacts
clean:
  rm -rf build/ install/ log/
