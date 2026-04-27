#!/usr/bin/env python3
import argparse
import datetime
import os
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Create a flight directory and launch the flight stack."
    )
    parser.add_argument(
        "--base",
        default="/data/flights",
        help="Base directory under which the timestamped flight folder is created (default: /data/flights)",
    )
    args = parser.parse_args()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    flight_dir = os.path.join(args.base, f"flight_{timestamp}")

    try:
        os.makedirs(flight_dir, exist_ok=False)
    except FileExistsError:
        print(f"[start_flight] Directory already exists: {flight_dir}", file=sys.stderr)
        sys.exit(1)
    except OSError as exc:
        print(f"[start_flight] Failed to create {flight_dir}: {exc}", file=sys.stderr)
        sys.exit(1)

    print(f"[start_flight] Flight directory: {flight_dir}")

    cmd = [
        "ros2",
        "launch",
        "cubesat_bringup",
        "flight.launch.py",
        f"flight_dir:={flight_dir}",
    ]

    print(f"[start_flight] Executing: {' '.join(cmd)}")

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\n[start_flight] Shutdown requested.")
    except subprocess.CalledProcessError as exc:
        print(
            f"[start_flight] Launch exited with code {exc.returncode}", file=sys.stderr
        )
        sys.exit(exc.returncode)


if __name__ == "__main__":
    main()
