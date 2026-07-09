# autotrim

Embedded ESP32/Arduino project for driving navigation display behavior and
sniffing reverse-engineered Garmin G5 AHRS data on CAN.

The active target board in the `Makefile` is currently `esp32s3`.

## Build

```sh
make
```

Builds the default ESP32-S3 firmware target.

```sh
make BOARD=csim
```

Builds the host-side simulation target.

## Runtime Inputs

The sketch reads Garmin G5 data from CAN and sends SL30-compatible serial
messages on `Serial2`.

NMEA position data can arrive in three forms:

- Raw NMEA lines on `Serial2`, such as `$GPRMC` / `$GPGGA`
- Command lines prefixed with `NMEA=`
- Raw NMEA command lines passed through the command parser

NMEA is parsed with TinyGPSPlus and stored in `navFix`. Mode 5 uses GPS as the
absolute position and altitude reference. For smoother vertical CDI movement,
GPS altitude is smoothed over a small sample window and high-rate G5 pressure
altitude contributes only the change since the latest GPS altitude anchor.

## Modes

`mode` is intentionally an integer because it represents the user's input
toggle count.

- `0-3`: HDG modes
- `4`: NAV mode
- `5`: ILS simulation
- `6`: CDI needle test movement

CAN debug output is controlled by explicit commands such as `canserial` and
`canudp`, not by special user-facing mode values.

## ILS Simulation

Mode 5 revives the historical ILS simulator from the `WaypointNav` package.
When a GPS fix is available, the sketch either creates a fake approach from the
selected VLOC course and altitude bug or chooses the best known nearby approach.
It then computes CDI and glideslope deflections and sends them through
`sl30.setCDI(hd, vd)`.

Lateral CDI and vertical glideslope use different reference points. Glideslope
uses the touchdown-zone point and TDZE. Lateral CDI projects a localizer antenna
reference point from the touchdown zone along the final approach course. Until
the approach database carries runway-specific lengths, the simulator assumes a
6,000 ft runway plus 1,000 ft beyond the departure end.

The two ILS entry paths are selected by the VLOC/OBS course at the moment mode 5
creates the simulator:

- VLOC/OBS course zero: choose the nearest compatible runway from the built-in
  approach database.
- VLOC/OBS course nonzero: synthesize a fictional ILS in front of the current
  GPS position. The selected course becomes the final approach course, the
  altitude bug shapes the touchdown-zone elevation, and the intercept point is
  placed 3 km ahead of the current track. Synthetic approaches use the same
  6,000 ft runway-length assumption for localizer projection.

To switch between these paths, leave mode 5 and re-enter it after changing the
VLOC/OBS course.
