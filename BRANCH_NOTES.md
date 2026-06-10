# feature/flight-dir-rotation

## What

Implements the full flight-directory rotation flow that previously dead-ended
at an unimplemented `restart_system()` stub.

### cubesat_captain
- `restart_system()` now runs a configurable `restart_command` parameter
  (default `sudo systemctl restart atlas-flight.service`) via a double-fork +
  `execl("/bin/sh", ...)`. The grandchild is reparented to init so the restart
  survives the captain itself being killed by the restart; the captain never
  blocks on it. The first child is reaped to avoid zombies.
- `Command_NewFlightDanger` from the ground station is now wired in
  `handle_packet`: it calls `flag_for_new_flight_dir()` (writes
  `<flight_dir>/new_dir_please.flag`) and then `restart_system()`. Previously
  the command fell through to the "unimplemented handler" branch.

### cubesat_bringup
- New `scripts/flight_boot.sh` boot wrapper (installed to
  `share/cubesat_bringup/scripts/`):
  - finds the most recent `flight_YYYYMMDD_HHMMSS` dir under `$FLIGHT_BASE`
    (default `/home/aaron/flights`)
  - creates a fresh timestamped dir on first boot, or when the latest dir
    contains `new_dir_please.flag` (clearing the flag first so a crash loop
    can't burn through directories)
  - sources ROS + the workspace install and `exec`s
    `ros2 launch cubesat_bringup flight.launch.py flight_dir:=<dir>`
- New `systemd/atlas-flight.service` (installed to `share/cubesat_bringup/systemd/`):
  runs flight_boot.sh as user aaron, `Restart=always`, `TimeoutStopSec=20` with
  `KillMode=mixed` so `ros2 launch` can shut nodes down cleanly. Install/enable
  instructions and the required sudoers line for the passwordless
  `systemctl restart` are in the unit file header.

## Why

Over-the-air "start a fresh flight" (NewFlightDanger) is the recovery hammer
for a corrupted flight dir or a stale launched-state on the pad. Without
restart_system the flag file was written but nothing ever consumed it, so the
command was a no-op. Putting directory selection in the boot wrapper (instead
of the captain) means every restart path — OTA command, watchdog kill, power
cycle — goes through exactly the same logic.

## Testing

- `bash -n` clean on flight_boot.sh; workspace builds clean in dev:humble.
- Restart command is a parameter, so bench testing can point it at
  `systemctl --user` or a stub script without code changes.
