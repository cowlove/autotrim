# AGENTS.md - autotrim Project Notes

This file is repo-local guidance for AI/coding agents working on `autotrim`.
Keep private/user-specific memory out of this file; commit only project facts,
design intent, caveats, and workflows that future agents need.

## Project Shape

- `autotrim` is an embedded ESP32/Arduino-style navigation display project.
- The main runtime is in `autotrim.ino`.
- `Makefile` owns the build targets.
- `README.md` is user-facing documentation; keep this file agent-facing.
- Helper/test utilities include `test.sh`, `parseG5cat.cpp`, `parse_sweep.cpp`,
  and `sendcmd.sh`.

## Build And Test

- Default target: `make`
  - Builds the ESP32-S3 firmware target.
- Host simulation target: `make BOARD=csim`
  - Builds `./autotrim_csim`.
- Regression harness: `./test.sh`
  - Builds/runs the csim target.
  - Exercises mode 5 ILS simulation through `--tracksim`.
  - Looks for `TSIM` status output from the simulated track model.
- The csim build has a known benign `ARDUINO` redefinition warning from the
  `esp32csim` include path.
- Before committing code changes, prefer at least:
  - `git diff --check`
  - `./test.sh`
  - `make -j2` when target firmware behavior may be affected

## Mode Semantics

- `mode` is intentionally a user-facing integer.
- Do not refactor `mode` into an enum/state machine merely for aesthetics.
- The integer maps to the user's input toggle count.
- Current intended modes:
  - `0-3`: HDG modes
  - `4`: NAV mode
  - `5`: ILS simulation
  - `6`: CDI needle test movement
- CAN debug/dump behavior should not be assigned special user-facing mode
  values. Use explicit commands such as `canserial` and `canudp`.

## Mode Transitions

- Use `setMode(newMode)` for mode changes.
- Do not write `isrData.mode` directly unless there is a very specific reason.
- `setMode()` resets the active ILS simulator when entering or leaving mode 5.
- Expected user behavior:
  - toggling into mode 5 starts a fresh ILS simulation on the next ILS update
  - leaving mode 5 deletes any existing ILS simulation
  - returning to mode 5 after another toggle sequence must not reuse old ILS
    state

## GPS / NMEA State

- The restored ILS code uses the historical `GDL90Parser::State` shape via
  `currentState`.
- NMEA is parsed with TinyGPSPlus.
- NMEA can arrive as:
  - raw NMEA on `Serial2`
  - command lines prefixed with `NMEA=`
  - raw `$...` / `!...` NMEA lines passed through the command parser
- Mode 5 waits for a valid GPS fix before creating/updating the ILS simulator.
- Open field concern: verify real hardware provides usable lat/lon and either
  GPS altitude or an acceptable pressure-altitude fallback.

## ILS Simulation

- Mode 5 revives the historical `WaypointNav::IlsSimulator` behavior.
- `autotrim.ino` includes nav helpers from the sibling `../winglevlr` repo:
  - `../winglevlr/GDL90Parser.h`
  - `../winglevlr/WaypointNav.h`
- The ILS simulator computes CDI/glideslope percentages, scales them by `2.0`,
  and sends them to the SL30 path with `sl30.setCDI(hd, vd)`.
- The CDI model is ILS-like angular guidance, not GPS-style discrete CDI
  scaling.
- Lateral CDI and glideslope use different reference points:
  - glideslope uses the touchdown-zone point and TDZE
  - lateral CDI uses a projected localizer antenna reference point
- The current localizer projection assumes:
  - 6,000 ft runway length
  - plus 1,000 ft beyond the departure end
  - projected from TDZ along the final approach course
- The approach database does not currently store runway-specific lengths or
  explicit localizer antenna coordinates.

## ILS Entry Paths

Mode 5 chooses its ILS source when the simulator is created:

- VLOC/OBS course zero:
  - call `findBestApproach()`
  - choose the nearest compatible built-in approach
- VLOC/OBS course nonzero:
  - synthesize a fictional ILS ahead of the current GPS position
  - use the selected VLOC course as final approach course
  - use the altitude bug to derive touchdown-zone elevation
  - place the intercept point 3 km ahead of the current GPS track

To switch between these paths, leave mode 5 and re-enter mode 5 after changing
the VLOC/OBS course. This is intentional because entering mode 5 creates a fresh
simulation based on current GPS/knob state.

## csim Track Simulation

- `--tracksim` is re-enabled in the csim path.
- The track simulator uses `WaypointNav::WaypointSequencer`.
- Simulated GPS state is fed back through `currentState`, matching the ILS code
  path used on hardware.
- A stable `TSIM` line is emitted for the regression harness.
- `test.sh` should use the current csim binary flow:
  - `make BOARD=csim`
  - `./autotrim_csim`

## Cross-Repo Dependency

- `autotrim` depends on the sibling `../winglevlr` repo for nav simulation
  helpers.
- Changes to `WaypointNav.h` may affect other `winglevlr` behavior. Treat that
  as a deliberate cross-repo edit and validate both sides when practical.
- Recent relevant `winglevlr` behavior:
  - `WaypointTracker::setCDI(hd, vd, decisionHeight)` feeds `corrH`/`corrV`
  - `corrH` adjusts simulated steering
  - `corrV` adjusts commanded altitude only when altitude is valid
  - vertical speed is guarded when no altitude command is valid

## Generated Artifacts

- These generated paths are intentionally ignored:
  - `autotrim_csim`
  - `build/`
  - `tun-rp1`

## Commit Convention

- Commits made by AI agents should include `AI generated.` in the commit body.
- Keep commits focused. When both `autotrim` and `winglevlr` change, prefer
  separate commits in each repo.
