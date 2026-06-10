# feature/parameter-persistence

## What

Radio profile changes negotiated over the air are now persisted to
`<flight_dir>/radio_profile.json` and reloaded on startup, replacing the
commented-out `loadProfileFromFile`/`serializeProfile` stubs.

### cubesat_radio
- The radio node now declares `flight_dir` (the launch file was already passing
  it to every node, but radio never declared it, so it was silently dropped).
- `persistProfile()` writes the five profile fields as plain JSON, to a `.tmp`
  file first and then `rename()`s into place, so a power cut mid-write cannot
  leave a truncated file that poisons the next boot.
- It is called at the two places a profile becomes "stable":
  - immediate reconfigure via `setProfileNow` (radioLoop commit point)
  - successful over-the-air negotiation (`LinkTestHeard` in processEvent,
    right where `stableProfile = profileUnderTest`)
  Both run on the radio thread; profiles that are merely *under test* are
  deliberately not persisted — a profile only hits disk after the ground
  station has proven it can talk on it.
- `loadProfileFromFile()` is a hand-rolled key scanner (find `"key"`, skip to
  `:`, `stoll`) — no JSON library dependency on the Pi Zero 2 W. Every field
  must be present and pass range validation (sx1262 tuning range 137–1020 MHz,
  SF 5–12, CR 4/5–4/8, power −17…+22 dBm) or the file is ignored and the node
  falls back to the yaml/declared parameters.

## Why

After a landing the team retunes the link (lower SF for images, power changes
to keep the camera alive). Any respawn of the radio node — and launch respawns
it automatically — silently reverted to the boot parameters, desyncing payload
and ground station: the payload would come back on a profile the GS had long
since left. Persisting only *negotiated* profiles keeps the failure mode safe:
a reboot can never come up on a profile that was never proven to work.

## Testing

- Workspace builds clean in dev:humble (`colcon build --packages-up-to cubesat_radio`).
- Validation rejects missing fields and out-of-range values; an absent or
  invalid file leaves the parameter-derived profile untouched.
