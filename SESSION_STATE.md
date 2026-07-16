# autotrim Session State

Current live state for the autotrim work, captured so a session reset does not
lose the thread.

## What Is Proven

- The normal serial console is not a credible cause of the main-loop backup.
- `Serial.begin(921600)` is the active baud rate for the main console.
- The ordinary runtime log volume is tiny compared with the available serial
  bandwidth.
- The old CAN dump / CAN debug paths can flood the serial line, but those are
  explicit debug modes and should stay off in production.
- Knob gesture detection works correctly on actual hardware.
- The delayed `KNOB:` debug line was only a logging artifact, not a sign that
  gesture detection failed.

## Current Project Direction

- The current ILS simulator is selected by `mode 5`.
- `mode 4` is currently the NAV mode in the existing mode map.
- Placeholder `mode 6` is now being used for a synthetic VOR simulation.
- The VOR sim creates a synthetic station one nautical mile ahead of the
  aircraft's current course when mode 6 is entered and fresh GPS/track data is
  available.
- The VOR sim drives lateral CDI from the aircraft's radial relative to that
  station and the current VLOC/OBS knob setting.
- VOR cone of confusion is modeled as a fixed 0.2 nautical mile distance from
  the synthetic station. Until SL30 invalid-CDI flags are decoded, the sim pegs
  the lateral needle inside that radius.

## Important Constraints

- Do not casually reuse `mode 4` for VOR unless the existing NAV behavior is
  intentionally being replaced.
- Keep the current mode semantics intact unless explicitly revised.
- Keep CAN debug/dump behavior behind explicit commands such as `canserial`
  and `canudp`, not user-facing mode numbers.
- Mode 5 remains the ILS simulation mode.
- The old automatic mode-6 CDI needle sweep is no longer the mode-6 behavior;
  use the explicit `cdi` command for debug needle motion.

## Open Questions

- Should VOR be a lateral-only CDI simulation, or should it also expose any
  extra synthetic state for testing?
- Should the ILS and VOR sims be factored into a shared geometry/helper core
  before adding the new mode?
- Should the SL30 wrapper learn no-glideslope/no-vertical flags instead of
  centering the vertical needle in VOR mode?
- Should the SL30 wrapper expose a CDI-valid/invalid flag so VOR cone of
  confusion can mark the needle invalid instead of pegging it?

## Good Resume Point

- The next useful step is validating the placeholder VOR behavior in csim or on
  hardware, then deciding whether to factor more ILS/VOR shared plumbing.
