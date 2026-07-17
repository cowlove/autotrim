# Session Handoff — autotrim

Updated 2026-07-17.

## Current repository state

- `master` is synchronized with `origin/master`.
- Latest pushed commit: `b985d38` — `Fix G5 knob wrap handling`.
- The repository depends on the sibling `../winglevlr` checkout for
  `WaypointNav.h` and simulator helpers.

## Durable behavior and constraints

- User-facing modes remain integer toggle counts: `0–3` HDG, `4` NAV, `5`
  ILS simulation, and `6` placeholder synthetic VOR simulation.
- Use `setMode(newMode)` for transitions. Entering/leaving mode 5 creates or
  destroys the ILS simulator; re-enter mode 5 after changing VLOC/OBS.
- Keep CAN dump/debug behind explicit commands such as `canserial` and
  `canudp`; do not consume user-facing mode numbers for debug behavior.
- Mode 5 uses fresh NMEA GPS position, track, speed, and altitude. It must not
  silently substitute CAN/GDL90 pressure altitude for the GPS absolute
  altitude reference.
- GPS freshness is based on age/timestamps, not TinyGPSPlus's sticky
  `location.isValid()` alone. The current stale-fix limit is 2000 ms.
- `SensorFusion` smooths GPS altitude and extrapolates position between fixes;
  use its fused values in the 100 ms CDI update loop.
- Do not clamp meaningful full-scale ILS CDI/glideslope output inside ±2.0.

## Build and test

```sh
make
make BOARD=csim
./test.sh
```

The host csim path requires the sibling `~/src/lv_port_linux` tree to be
built with CMake, and uses the shared TinyGPSPlus git checkout rather than an
Arduino-index version:

```sh
cd ~/src/lv_port_linux
cmake -B build
make -C build -j2
```

Before code commits, run `git diff --check`, the csim regression harness, and
the firmware build when applicable.

## Open work

The synthetic VOR remains intentionally provisional: a station is created one
NM ahead of the current course, full-scale CDI is modeled as 10 degrees, and a
fixed 0.2 NM cone-of-confusion radius currently pegs the needle because the
SL30 wrapper does not yet expose complete validity/TO-FROM flags.

## More complete engineering context

### Navigation data boundaries

- NMEA is accepted on `Serial2`, through `NMEA=` command lines, and through
  raw `$...`/`!...` command-parser lines. TinyGPSPlus owns the NMEA parse.
- `navFix` is the internal GPS-derived state for the ILS simulator. Do not
  grow `GDL90Parser::State` with unrelated navigation state; that structure is
  the GDL90 wire/parser shape.
- CAN pressure altitude remains raw G5/CAN data in `isrData.palt`. It is not a
  corrected absolute altitude and must not be fed into `navFix.altMeters`
  without an explicit conversion policy.
- The GPS fix may coast for about two seconds (`GPS_FIX_STALE_MS == 2000`),
  after which mode 5 must stop updating CDI and report that it is waiting for
  GPS again.
- TinyGPSPlus location validity is sticky. Freshness is therefore determined
  with `location.age()` and timestamps, not `location.isValid()` alone.

### ILS geometry and entry behavior

- Mode 5 with VLOC/OBS course zero chooses the nearest compatible built-in
  approach through `findBestApproach()`.
- A nonzero VLOC/OBS course synthesizes an ILS ahead of the current GPS
  position. The course becomes the final approach course, the altitude bug
  provides touchdown-zone elevation, and the intercept point is placed about
  3 km ahead of current track.
- Re-enter mode 5 after changing VLOC/OBS so the simulator is rebuilt with the
  new entry path.
- Lateral CDI uses a projected localizer-antenna reference; glideslope uses
  touchdown-zone point and TDZE. This difference is deliberate, even though
  both currently use the approximation of a 6,000 ft runway plus 1,000 ft
  beyond the departure end.
- Full-scale ±2.0 CDI/glideslope values are meaningful pegged-needle states.
  Do not clamp them inward merely to make the csim follower look stable.

### Debugging lessons

- The 921600-baud normal console and ordinary log volume were not the cause of
  the observed main-loop backup. Explicit CAN dump/debug modes can flood the
  serial line and must remain off in normal operation.
- Delayed `KNOB:` output was a logging artifact; real hardware knob gesture
  detection was working.
- If vertical guidance appears more lively than lateral guidance in csim,
  first inspect the simple vertical follower. Its angular-error-derived
  vertical speed becomes more sensitive near the touchdown-zone point.
- The csim dependency chain is `autotrim` → `winglevlr` navigation helpers →
  host `lv_port_linux`/LVGL artifacts. Diagnose that chain before changing
  application geometry.

### Current verification standard

Run the host regression harness after behavior changes. A passing firmware
compile alone is insufficient for simulator changes; a passing csim scenario
alone is insufficient for board-target changes. Keep source changes separate
from generated `build/`, simulator output, and serial-capture artifacts.
