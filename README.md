![Project Preview](./rit-launch-logo-large_1.png)

# ATLAS Software

Arm That Looks At Stuff (ATLAS) 

## About

Controls the robotic arm for the payload of RISK, The 2026 RIT IREC rocket.

## Installation

```
git clone https://github.com/RIT-Launch-Initiative/IREC-26-Payload
cd IREC-26-Payload
```

## Deploying To The Pi

`deploy.sh` is a deploy-only path, separate from the normal dev build:

```bash
./deploy.sh
```

It will:
- detect the Pi architecture over SSH
- build a non-symlink install tree for the Pi in `install_pi/`
- sync that install tree to `/home/aaron/atlas_ws/install` on the Pi

To run a command on the Pi right after deploy:

```bash
./deploy.sh --run "ros2 run <package> <executable>"
```

Important constraints:

- This flow expects the Pi to be reachable as `ssh atlas` by default. Override with `PI_SSH_TARGET=<host>`.
- The deploy build currently targets a 64-bit Pi userspace. `uname -m` on the Pi should report `aarch64`.
- ROS 2 Humble and `rsync` must already be installed on the Pi.
- Local Docker needs an ARM-capable buildx builder. The default builder name in the script is `atlas-arm64`.
- Deploy builds use `Dockerfile.deploy`, which is intentionally slimmer than the main dev image.
- If your machine has not been prepared for ARM64 Docker builds yet:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
docker buildx create --name atlas-arm64 --driver docker-container --use
docker buildx inspect --bootstrap
```

- The normal local dev build still uses `--symlink-install`; that is intentional for development, but it is not suitable for deployment.
