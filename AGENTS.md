# AGENTS.md - autotrim Project Notes

This file is repo-local guidance for AI/coding agents working on `autotrim`.
Keep private/user-specific memory out of this file; commit only project facts,
design intent, caveats, and workflows that future agents need.

## Documentation Style

- Keep this file verbose as the project evolves. When a decision, boundary,
  assumption, hardware caveat, simulation behavior, or debugging lesson affects
  future work, write it down here in enough detail that the next session can
  recover the reasoning without needing chat history.
- Prefer explicit project notes over terse reminders. This project has several
  subtle data-source boundaries, mode semantics, and simulator-vs-hardware
  differences; document those boundaries when they are discovered or clarified.
- Do not use this file for private/user-specific memory. It should contain
  durable project facts and engineering guidance that belongs with the repo.
- Use `SESSION_STATE.md` as the lightweight handoff file for the current live
  project state when a reset or session switch is likely. Keep the handoff
  concise and factual so the next session can resume without rereading chat.

## Project Shape

- `autotrim` is an embedded ESP32/Arduino-style navigation display project.
- The main runtime is in `autotrim.ino`.
- `Makefile` owns the build targets.
- `README.md` is user-facing documentation; keep this file agent-facing.
- Helper/test utilities include `test.sh`, `tests/csim/run.py`,
  `parseG5cat.cpp`, `parse_sweep.cpp`, and `sendcmd.sh`.

## Build And Test

- Default target: `make`
  - Builds the ESP32-S3 firmware target.
- Host simulation target: `make BOARD=csim`
  - Builds `./autotrim_csim`.
- Regression harness: `./test.sh`
  - Thin compatibility wrapper around `python3 tests/csim/run.py`.
  - Builds the csim target once with `make BOARD=csim -j2`.
  - Runs every JSON scenario in `tests/csim/scenarios/` unless specific
    scenario names are passed.
  - Writes generated track files and simulator output under `tests/csim/out/`.
  - Parses the final `TSIM` status line into named fields and compares those
    fields against scenario-defined pass/fail ranges.
  - Supports output regex assertions such as built-in approach selection
    (`Chose 'KBFI 14R'`) or synthetic ILS creation (`Started ILS`).
- The csim build has a known benign `ARDUINO` redefinition warning from the
  `esp32csim` include path.
- `autotrim` depends on the sibling `../winglevlr` repo for `WaypointNav.h`
  and the ILS/VOR simulator helpers.
- The csim path also depends on the host `lv_port_linux` checkout used by
  `winglevlr`; if that tree is not built, `autotrim BOARD=csim` can fail at the
  link step even when the firmware sources compile cleanly.
- The current working csim fix path is:
  - install/use the same `TinyGPSPlus` git checkout on both machines instead
    of `arduino-cli lib install TinyGPSPlus@...`
  - build `lv_port_linux` with CMake, not the old top-level `make`:
    - `cd ~/src/lv_port_linux`
    - `cmake -B build`
    - `make -C build -j2`
  - then rebuild `autotrim`/`winglevlr` csim targets
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
  - `6`: placeholder VOR simulation
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

- ILS calculations use the internal `NavFixState` shape via `navFix`.
- Do not use `GDL90Parser::State` as general navigation state. That structure
  belongs to the GDL90 parser/wire format and should not grow unrelated
  internal fix fields.
- NMEA is parsed with TinyGPSPlus.
- NMEA can arrive as:
  - raw NMEA on `Serial2`
  - command lines prefixed with `NMEA=`
  - raw `$...` / `!...` NMEA lines passed through the command parser
- Mode 5 waits for valid NMEA GPS-derived position, track, speed, and altitude
  before creating/updating the ILS simulator.
- ILS simulation must not backfill missing NMEA values from CAN or GDL90 data.
  CAN/GDL90 paths may still parse, report, and forward their own data elsewhere
  in the program.
- CAN pressure altitude remains raw G5/CAN data in `isrData.palt`. Do not feed
  its absolute value into `navFix.altMeters` unless an explicit,
  altimeter-corrected conversion policy is added.
- Vertical CDI calculations use `SensorFusion` in `SensorFusion.h`: NMEA GPS altitude is kept
  as the absolute reference and smoothed with a small linear fit evaluated at
  the latest GPS altitude timestamp. This avoids the lag a moving average would
  introduce during a steady climb or descent. G5 pressure altitude contributes
  only the high-rate delta since the latest GPS altitude anchor.
- A previously valid GPS fix may be reused only briefly. The current limit is
  `GPS_FIX_STALE_MS == 2000`, so mode 5 can coast for about two seconds after
  the last fresh NMEA location, then it stops updating CDI and reports that it
  is waiting for GPS again.
- While the NMEA fix is fresh, the 100 ms G5/SL30 loop should call `setCDI()`
  using `SensorFusion::fusedPosition()` rather than holding the last NMEA
  lat/lon until the next sentence arrives.
- `SensorFusion::fusedPosition()` extrapolates lat/lon from the latest GPS
  position, track, and groundspeed at 10 Hz, so lateral CDI is recomputed
  smoothly between slower GPS position fixes.
- The vertical needle should use `fusedGpsAltitudeMeters()` rather than reading
  `navFix.altMeters` directly, so the 100 ms loop can reflect smooth pressure
  altitude changes without losing the GPS absolute reference.
- TinyGPSPlus `location.isValid()` is sticky after a prior good fix, so NMEA
  freshness should be judged with `location.age()` and `navFix.timestamp`,
  not `isValid()` alone.
- Open field concern: verify real hardware provides usable lat/lon and GPS
  altitude, or deliberately design a corrected baro-altitude fallback.

## ILS Simulation

- Mode 5 revives the historical `WaypointNav::IlsSimulator` behavior.
- `autotrim.ino` includes nav helpers from the sibling `../winglevlr` repo:
  - `../winglevlr/WaypointNav.h`
- The ILS simulator computes CDI/glideslope percentages, scales them by `2.0`,
  and sends them to the SL30 path with `sl30.setCDI(hd, vd)`.
- Do not clamp mode-5 CDI/glideslope output to just inside `+/-2.0`.
  Full-scale `+/-2.0` is meaningful: it means the simulated needle is pegged
  and should not yet be treated as usable capture guidance by the csim
  autopilot/follower.
- The CDI model is ILS-like angular guidance, not GPS-style discrete CDI
  scaling.
- Lateral CDI and glideslope use different reference points:
  - glideslope uses the touchdown-zone point and TDZE
  - lateral CDI uses a projected localizer antenna reference point
- This reference split is intentional. The localizer angular course should be
  measured from the localizer antenna geometry, which is projected beyond the
  departure end. The glideslope angular error should be measured against a
  course to the touchdown-zone point/TDZE, which is the current approximation
  for the glideslope antenna focal geometry.
- The current localizer projection assumes:
  - 6,000 ft runway length
  - plus 1,000 ft beyond the departure end
  - projected from TDZ along the final approach course
- The approach database does not currently store runway-specific lengths or
  explicit localizer antenna coordinates.
- If the vertical needle looks livelier than the lateral needle in csim, first
  suspect the simple vertical follower, not the ILS geometry. The csim follower
  currently derives vertical speed directly from angular glideslope error, and
  angular vertical error gets more sensitive as distance to TDZ shrinks.

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

## VOR Simulation

- Mode 6 is currently the placeholder VOR simulation mode.
- Entering mode 6 resets any existing VOR simulator instance. The runtime then
  creates a new synthetic VOR when the 100 ms G5/SL30 loop sees a fresh GPS fix
  with a valid track.
- The synthetic station is placed one nautical mile ahead of the aircraft's
  current course at creation time.
- The station position is fixed after creation. Leave and re-enter mode 6 to
  create a new station from a new aircraft position/course.
- Lateral CDI is based on the aircraft's radial from the synthetic station
  and bearing to the synthetic station compared with the current VLOC/OBS
  course. The VLOC/OBS course is treated as magnetic and converted to true with
  `magToTrue()`, matching the ILS path.
- The SL30 wrapper's VOR overload emits NAV-valid/superflag bits and sets the
  TO or FROM flag according to whether the selected OBS course points toward
  or away from the station. The VOR model uses the smaller of the
  inbound-to-station and outbound-radial angular errors for CDI selection.
- Mode 6 also emits the active-VOR radial as a valid `PMRRV23` message. It does
  not echo an OBS `PMRRV22` message because the G5 owns the OBS setting.
- VOR CDI full-scale deflection is currently modeled as 10 degrees.
- VOR cone of confusion is intentionally modeled as a fixed distance, not an
  angular or altitude-dependent volume. The current threshold is 0.2 nautical
  miles from the synthetic station.
- Inside the cone of confusion, mode 6 pegs the lateral needle and clears both
  TO and FROM flags, matching the SL30 protocol's ambiguous-direction state.
- VOR output currently centers the vertical needle (`vd = 0`) because the SL30
  wrapper does not yet expose separate no-glideslope/no-vertical flags.
- The old automatic mode-6 CDI needle sweep was replaced by the VOR placeholder.
  The explicit `cdi` command can still set `debugMoveNeedles` for test motion.

## csim Track Simulation

- `--tracksim` is re-enabled in the csim path.
- The track simulator uses `WaypointNav::WaypointSequencer`.
- Simulated GPS state is fed back through `navFix`, matching the ILS code path
  used on hardware.
- A stable `TSIM` line is emitted for the regression harness.
- `test.sh` is not intended to aim the airplane straight at the runway. It uses
  the first track point as the starting fix over Bainbridge Island and the
  second track point as a scripted intercept leg for KBFI ILS 14R.
- The `test.sh` track points should represent a realistic ILS intercept flow:
  lateral guidance should be intercepted and captured well before the simulated
  aircraft reaches glideslope capture. If the test begins too close to the
  airport, the csim can make localizer and glideslope capture appear too
  simultaneous, which is less representative of a real approach.
- The current `test.sh` geometry is the earlier Bainbridge-to-KBFI 14R
  intercept translated about two nautical miles farther from the airport along
  the runway 14R approach corridor. Both the starting point and the scripted
  intercept waypoint were moved outbound along the reciprocal of the 14R final
  approach course, preserving the roughly 30 degree intercept angle while
  giving the simulated aircraft more time on lateral guidance before the
  glideslope becomes active.
- The first `test.sh` waypoint altitude is intentionally lower than the old
  `1000` meter value. The track simulator waypoint altitudes are meters, and a
  lower start altitude helps the aircraft intercept the localizer before the
  glideslope without moving the whole scenario so far from KBFI that approach
  selection becomes delayed or less representative.
- The current second `test.sh` track point is still about 10 NM from the start
  on roughly `180.5` degrees true. That sets up about a 30 degree intercept to
  the KBFI 14R final approach course, which is about `150.5` degrees true after
  magnetic variation.
- The `test.sh` csim duration should be long enough for the farther-out start
  to descend to a low altitude after localizer/glideslope capture. It is
  intentionally longer than the original near-airport test duration.
- While the lateral CDI is pegged at `abs(hd) >= 2.0`, the csim ILS follower
  should not steer from the CDI. It should let the scripted waypoint leg fly
  the intercept, similar to how a pilot would avoid chasing a pegged CDI.
- Once the lateral CDI comes alive (`abs(hd) < 2.0`), the csim ILS follower can
  steer from the needle and should settle onto the final approach course with
  small lateral CDI and cross-track error.
- Vertical correction follows the same alive/pegged idea independently:
  glideslope correction should wait while `abs(vd) >= 2.0`.
- Do not confuse the SL30 output path with csim aircraft dynamics. The runtime
  code emits `sl30.setCDI(hd, vd)` from the 100 ms loop so hardware sees smooth
  needle commands, but csim aircraft motion is driven from the `hd`/`vd` globals
  inside `runTrackSim()`.
- The newer csim ILS follower path (`flyIlsNeedles()`) bypasses
  `WaypointTracker::setCDI()` once an active, non-pegged ILS needle is available
  and applies its own per-timestep heading/vertical motion. The older fallback
  path still calls `tSim.wptTracker.setCDI(hd, vd, ...)` from the track-sim loop;
  that helper contains hardcoded correction gains and derivative terms, so
  changing the track-sim call rate can effectively change simulated aircraft
  gain. If the regression still captures the localizer/glideslope and `TSIM`
  values remain sane, do not over-tune this low-resolution test model.
- After the GPS-coasting CDI smoothing work, `out.txt` may show the simulated
  ILS approach finishing with CDI needle values clamped closer to zero than
  before. This is acceptable for now. The purpose of the csim regression is to
  catch gross failures in approach selection, needle capture, final-course
  tracking, and localizer cross-track error; it is not a high-fidelity aircraft
  dynamics model. Leave the tighter convergence alone unless the simulation
  starts hiding a real regression or mis-flying the scripted intercept.
- The `TSIM range` field is distance to the active `WaypointSequencer`
  waypoint, not necessarily distance to the runway or TDZ. With the current
  intercept-leg test, do not use `range` as a runway-arrival assertion.
- Csim regression scenarios live in `tests/csim/scenarios/*.json`.
  Scenario files should be named for behavior, not sequence numbers. Prefer
  names such as `kbfi_14r_builtin.json` and `synthetic_ils_obs140.json` over
  root-level scripts such as `test2.sh`.
- Each csim scenario owns:
  - `seconds`: simulator duration
  - `track`: the literal `WaypointSequencer` track/input lines to feed through
    `--tracksim`
  - `expect_output`: regexes that must appear in simulator output
  - `reject_output`: regexes that must not appear
  - `expect_final_tsim`: exact values or `[min, max]` ranges for parsed final
    `TSIM` fields
- Parsed final `TSIM` fields currently include:
  - `t`
  - `mode`
  - `hd`
  - `vd`
  - `lat`
  - `lon`
  - `alt`
  - `trk`
  - `range`
  - `xte`
- The built-in KBFI regression assertion checks:
  - built-in approach selection chose `KBFI 14R`
  - mode is `5`
  - lateral CDI is near centered (`hd` close to zero)
  - simulated track is near the KBFI 14R final approach course
  - localizer cross-track error is small
- `synthetic_ils_obs140.json` exercises the synthetic/fake ILS entry path
  instead of the built-in KBFI approach-selection path. In this path, entering
  mode 5 with a nonzero VLOC/OBS course creates a fictional ILS in front of the
  simulated aircraft.
- For synthetic ILS csim tests, do not set `INPUT.MODE 5` before the simulator
  has flown enough to establish a meaningful current track. If mode 5 is set
  before or at the first waypoint, `navFix.track` can still be `0.0`, so the
  fake ILS is projected in front of a northbound/unknown track instead of in
  front of the intended intercept leg. A short setup leg before `INPUT.MODE 5`
  gives the simulator a real track, then the next leg can intercept the
  fictional localizer.
- Track file altitude units are easy to mix up:
  - waypoint altitude fields in `tracksim.txt` are parsed as feet and converted
    to meters by `WaypointSequencer`
  - `INPUT.ALTBUG` feeds `g5KnobValues[2]` directly and the current synthetic
    ILS code treats it as meters when deriving touchdown-zone elevation
  Keep that distinction in mind when setting up fake ILS tests. A value such as
  `INPUT.ALTBUG 100` means roughly a 100 meter altitude bug for the synthetic
  TDZE calculation, not 100 feet.

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
