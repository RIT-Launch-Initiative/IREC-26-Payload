#!/usr/bin/env python3
import argparse
import datetime
import os
import subprocess
import sys



def nextIdInDir(dir):
    lastHighest = 0
    sawAny = False
    for path in os.listdir(path):
        try:
            id = int(path)
            sawAny = True
            if (id > lastHighest):
                lastHighest = id
        except:
            continue
    if sawAny:
        lastHighest+=1
    return lastHighest

def dirWantsFresh(dir):
    if os.exists(dir+'/new_dir_please.flag'):
        return True
    return False

def main():
    parser = argparse.ArgumentParser(
        description="Create a flight directory and launch the flight stack."
    )
    parser.add_argument(
        "--base",
        default="~/flight_data",
        help="Base directory under which the timestamped flight folder is created (default: ~/flight_data)",
    )
    args = parser.parse_args()
    if len(os.listdir(args.base)) == 0:
        dirId = 0
    else:
        nextDirId = nextIdInDir(args.base)
        prevDirId = next 
    print("Next Dir ID: ")

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
        f"next_image_id:={next_image_id}"
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

