# feature/watchdog-and-health-monitor

## What

Adds a node-silence watchdog to the captain so a dead or hung pi_io, stm_bridge,
or radio node is detected, announced, and (for the arm) acted on.

### cubesat_msgs
- New `SystemHealth.msg`: per-node ok flags plus seconds-since-last-heard for
  pi_io, stm_bridge, and radio (-1 means never heard since captain start).

### cubesat_captain
- Tracks the last time each node was heard from:
  - pi_io: any of `pi/lis3dh`, `pi/power`, `pi/gps`
  - stm_bridge: `stm/arm_status`
  - radio: `radio/state` heartbeat or any `radio/rx_packet`
- A `health_check_period_s` (default 1 s) timer publishes `pi/system_health`
  every tick, computing ok = heard within `node_silence_timeout_s` (default 5 s,
  both configurable in captain.yaml).
- On the healthy -> silent transition (only on the edge, so a dead node does not
  buzz forever): logs a WARN and requests a BEEP_CODE_3_EQUAL x5 buzzer sequence.
- If stm_bridge goes silent while the captain is in Unfolding or AutoCamera
  (an autonomous arm sequence is in progress), the captain cancels all in-flight
  ExtendArm goals via `async_cancel_all_goals()` — without stm feedback there is
  no result and no timeout path on the client side, so the sequence must not be
  allowed to keep driving blind. The abort latch resets when stm_bridge recovers.

### cubesat_radio
- The radio node previously only published on `radio/state` ... never. It now
  publishes a 1 Hz `RadioState` liveness heartbeat (with the current outbound
  queue depth, read under the queue mutex). The timer is created before the
  hardware-failure early-returns in the constructor so it reflects process
  liveness rather than RF health.

## Why

The Pi Zero 2 W stack has already shown failure modes where one node dies and
the rest of the system keeps going (camera node dying on high-power TX, i2c bus
lockups). Launch respawns crashed processes, but a *hung* process (blocked SPI
ioctl, wedged camera) never exits and was previously invisible. The watchdog
makes that state observable on the ground (SystemHealth in the bag + telemetry
potential), audible on the pad (buzzer), and safe in the one case where blind
actuation can do mechanical damage (arm extension).

## Testing

Workspace builds clean in the dev:humble container (`colcon build` for
cubesat_msgs, cubesat_captain, cubesat_radio and dependents).
