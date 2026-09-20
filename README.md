# nanoCH32V003 + LIS2HH12 accelerometer

Bring-up and test project for a [LIS2HH12TR breakout](https://github.com/tdarlic/LIS2HH12TR-devboard)
accelerometer wired to a nanoCH32V003 dev board, programmed/debugged with a
WCH-LinkE. The MCU firmware talks to the accelerometer over I2C and exposes a
simple command protocol over UART; a Python terminal dashboard drives it for
interactive testing (self-test, FIFO/stream mode, interrupts, live plotting).

## Hardware

- nanoCH32V003 dev board (CH32V003)
- WCH-LinkE programmer, connected to the board's debug pins and to your PC via USB
- LIS2HH12TR breakout board - see [tdarlic/LIS2HH12TR-devboard](https://github.com/tdarlic/LIS2HH12TR-devboard)
  for the breakout's schematic/pinout

### Wiring

**I2C (accelerometer):**

| LIS2HH12TR breakout | nanoCH32V003 |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | PC1 |
| SCL | PC2 |

PC1/PC2 are the CH32V003's default I2C1 pins (no remap needed).

> **You must add pull-up resistors on SDA and SCL** (e.g. 4.7kΩ from each
> line to 3.3V) unless your breakout already has them on board. The
> CH32V003's I2C pins are open-drain only - with nothing pulling the lines
> high, the bus will never ACK and you'll see zero devices on a scan.

**UART (commands / live data), via the WCH-LinkE's own RXD/TXD pins:**

| WCH-LinkE | nanoCH32V003 |
|---|---|
| RXD | PD5 (MCU TX) |
| TXD | PD6 (MCU RX) |
| GND | GND (shared with the debug connection) |

No separate USB-serial adapter needed - the LinkE's RXD/TXD pins bridge to
a USB virtual COM port (`/dev/ttyACM0` on Linux) over the same USB cable
used for programming.

**INT1/INT2 (real electrical interrupts):**

| LIS2HH12TR breakout (J2 header) | nanoCH32V003 |
|---|---|
| Pin 2 (INT1) | PD4 |
| Pin 3 (INT2) | PD3 |

The sensor drives both push-pull, active-high by default, so these are
genuine edge-triggered hardware interrupts (`EXTI4`/`EXTI3`), not just
I2C status polling - see `INT PINSTATUS`/`IMPACT PINSTATUS` below. GND
is already shared via the I2C wiring above.

### A note on dupont wires

Loose/marginal dupont wire connections were the cause of nearly every
"weird" symptom encountered while building this (frozen sensor readings,
garbled UART commands, and once a full lockup - see below). If you see
readings that look stuck, wildly out of range, garbled command output,
or the board goes completely silent, reseat the wiring - especially
VCC/GND to the breakout - before suspecting the code.

The INT1/INT2 pins are configured with a weak internal pull-down as a
safety net: a genuinely floating or marginal connection on an
edge-triggered interrupt pin (both edges armed) can free-run the ISR
fast enough to starve the main loop entirely - it happened once during
development and looked exactly like a dead board (no serial output at
all, not even `ACC` streaming). The sensor's push-pull output easily
overrides the weak pull when actually connected, so this doesn't affect
normal operation.

## Firmware

Located in `firmware/`, built on the [ch32fun](https://github.com/cnlohr/ch32fun)
framework (vendored as a git submodule at `firmware/ch32v003fun`).

```sh
git submodule update --init --recursive   # first time only
cd firmware
make flash      # build and flash
```

`nanoCH32V003.c` brings up I2C1, finds the LIS2HH12 (WHO_AM_I check,
tries both possible I2C addresses), and streams `ACC,x,y,z,temp_mc` (x/y/z
in mg, temp_mc in milli-degC from the sensor's embedded temperature
sensor - relative/drift-compensation only, not a calibrated absolute
reading) over UART1 at 115200 baud, 20 times/second. It also accepts
commands over the same UART link (see below).

All firmware I/O goes over the real UART (PD5/PD6), not the WCH-LinkE's
debug channel - so `make monitor`/`minichlink -T` won't show anything
useful here. Read the UART directly (e.g. `stty -F /dev/ttyACM0 115200
raw -echo && cat /dev/ttyACM0`) or use the Python dashboard below.

### Command protocol

Send a line (`command\n`) over the UART at 115200 baud:

| Command | Effect |
|---|---|
| `STREAM ON` / `STREAM OFF` | Enable/disable the periodic `ACC,x,y,z,temp_mc` output |
| `ODR <0-6>` | Change the sensor's output data rate: 0=power-down, 1=10Hz, 2=50Hz, 3=100Hz, 4=200Hz, 5=400Hz, 6=800Hz |
| `SELFTEST` | Run the ST-spec self-test (electrostatic force actuation) and report PASS/FAIL per axis |
| `FIFO MODE <mode> [thresh]` | Set FIFO mode: `BYPASS`, `FIFO`, `STREAM`, `STREAM2FIFO`, `BYPASS2STREAM`, `BYPASS2FIFO`; threshold 0-31 |
| `FIFO STATUS` | Report FIFO sample count / empty / overrun / watermark flags |
| `FIFO READ` | Drain and print all samples currently buffered in the FIFO |
| `INT ON <X\|Y\|Z\|ANY> <HIGH\|LOW> [thresh_mg] [dur]` | Arm interrupt generator 1 (`IG1` -> `INT1`) on a threshold-crossing event - exploratory/generic |
| `INT OFF` | Disarm `IG1` |
| `INT STATUS` | Report `IG1`'s interrupt source register (I2C-polled `IG_SRC1`) |
| `INT PINSTATUS` | Report the real INT1 pin's current level and edge count (PD4/`EXTI4`) |
| `WAKE ON [thresh_mg] [dur]` | Arm Activity/Inactivity wake-on-movement -> `INT1` (see below) |
| `WAKE OFF` | Disarm it |
| `IMPACT ON [thresh_mg] [dur]` | Arm interrupt generator 2 (`IG2`, all axes, `HIGH`) -> `INT2`, for collision/impact detection |
| `IMPACT OFF` | Disarm `IG2` |
| `IMPACT STATUS` | Report `IG2`'s interrupt source register (I2C-polled `IG_SRC2`) |
| `IMPACT PINSTATUS` | Report the real INT2 pin's current level and edge count (PD3/`EXTI3`) |
| `FS [2\|4\|8]` | Set, or with no argument report, the full-scale range (default: `+-8g`) |
| `PEAK` | Report the highest \|g\| seen since the last reset |
| `PEAK RESET` | Reset the peak-hold value |
| `HELP` | List commands |

The LIS2HH12 supports switchable full-scale ranges like most accelerometers
(2g/4g/8g here - some parts also offer 16g, this one doesn't). Changing `FS`
updates the sensor's `CTRL4` register and the firmware's mg-per-LSB
conversion factor together, so `ACC` output stays correctly scaled in mg
regardless of range. **Defaults to `+-8g`** - at `+-2g`, any real
acceleration over 2g just clips/saturates and is lost, so anything meant
to catch real shocks/impacts needs the headroom. The dashboard queries
and displays the actual current range on startup rather than assuming a
value, so it's always accurate even if firmware defaults change later.

Peak-hold (`PEAK`) tracks the highest combined magnitude
(`sqrt(x^2+y^2+z^2)`) seen on every sample, independent of whether
`STREAM` is on or off - so it keeps capturing shocks/peaks even while the
live output is paused.

**`HIGH` vs `LOW` are not mirror images of each other** - this isn't
documented in the datasheet and was verified empirically on real
hardware, sweeping the threshold against known axis readings:
- `HIGH`: the signed axis value exceeds `+thresh_mg` - a normal
  positive-direction excursion detector.
- `LOW`: `|axis value|` drops *below* `thresh_mg`, regardless of sign.
  It is **not** a "swung strongly negative" detector, despite the
  naming suggesting the opposite of `HIGH`. This sensor's interrupt
  generator is really built for 6D/4D orientation recognition, where
  `XLIE`/`YLIE`/`ZLIE` mean "this axis is near zero" (used together
  with the unused `AOI`/`6D` combination bits to detect which face is
  "up"), not "large negative acceleration". There is no single-bit
  "large negative excursion" event on this chip - if you need one,
  you'd have to build it externally (e.g. compare `ACC` values in
  software).

### Two interrupts, two purposes

The LIS2HH12 has two independent interrupt generators (`IG1`, `IG2`) and
two physical output pins (`INT1`, `INT2`); either generator can be routed
to either pin. This project uses them for two deliberately different,
non-generic purposes, motivated by a real product use case (a boat
tracker that needs to wake on movement and separately detect a
collision):

- **`WAKE` -> `INT1`**: not `IG1` at all, but the sensor's dedicated
  **Activity/Inactivity** hardware block - a different, purpose-built
  low-power feature. It auto-drops the sensor to 10Hz sampling while
  still, and restores full rate instantly on movement. It can *only* be
  routed to `INT1`, and there is **no I2C status register for it at
  all** - the physical pin is the only way to observe it, which is also
  how you'd use it in a real low-power product (MCU asleep, woken by
  the GPIO edge). `WAKE` and the generic `INT ON` (`IG1`) both target
  `INT1` and will fight over the same pin if both armed - arming one
  clears the other's routing.

  **PD4's polarity is inverted from the intuitive reading in `WAKE`
  mode** - verified with a real physical still-then-shake test, and not
  documented anywhere in the datasheet: **HIGH means STILL/inactive,
  LOW means MOVING/active.** The underlying bit is literally named
  `INT1_INACT` - it reports inactivity, not activity. A boat sitting
  still at the dock will correctly show this pin HIGH almost all the
  time; that's expected, not a fault. The dashboard already accounts
  for this and shows `[WAKE: STILL]`/`[WAKE: MOVING]` rather than a raw
  level for this mode; if you're driving `INT PINSTATUS` yourself,
  remember to invert it.
- **`IMPACT` -> `INT2`**: interrupt generator 2, armed on all three axes
  with `HIGH` direction (a collision can hit from any direction), using
  a single shared threshold (`IG2` has one `IG_THS2` register, not
  per-axis like `IG1`).

`INT STATUS`/`IMPACT STATUS` poll the sensor's `IG_SRC1`/`IG_SRC2`
registers over I2C (work with no extra wiring, but only reflect whatever
was true the last time polled), while `INT PINSTATUS`/`IMPACT PINSTATUS`
and the spontaneous `INTPIN`/`IMPACTPIN` lines reflect the real INT1/INT2
pins, updated instantly from `EXTI4`/`EXTI3` the moment the electrical
signal changes - genuine hardware interrupts, not polling.

Output lines include `ACC,..`, `SELFTEST,..`, `FIFOSTATUS,..`,
`FIFOSAMPLE,..`, `INTSTATUS,..`, `IMPACTSTATUS,..`, `INTPINSTATUS,..`,
`IMPACTPINSTATUS,..`, `PEAK,..`, spontaneous `INTEVENT,..`/`IMPACTEVENT,..`
(I2C-polled) and `INTPIN,..`/`IMPACTPIN,..` (real pins, on every edge),
plus `OK,..` / `ERR,..` acknowledgements.

## Python test interface

```sh
cd python
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python3 dashboard.py            # defaults to /dev/ttyACM0
.venv/bin/python3 dashboard.py /dev/ttyUSB0   # or specify a different port
```

`dashboard.py` is a terminal dashboard (curses) covering all of the above:

- Live bar-graph visualization of X/Y/Z (scaled to the current full-scale
  range) plus `|g|` magnitude, the peak-hold value, and temperature
- Live INT1 and INT2 pin readouts (real electrical level + edge count,
  PD4/`EXTI4` and PD3/`EXTI3`), each labeled with whether/how it's
  currently armed (`IG1`, `WAKE`, `IMPACT`, or `not armed`), updated the
  instant either physical interrupt fires. The armed label is tracked
  from this session's own actions (there's no single I2C query that
  reports what's currently routed to a pin) - the pin's real level/edge
  count is always ground truth regardless.
- Raw X/Y/Z readout pinned at the bottom of the screen at all times
- `s` - run self-test
- `o` - change ODR
- `g` - change full-scale range (2/4/8g)
- `f` - FIFO/stream mode submenu (set mode+threshold, check status, read samples)
- `i` - generic interrupt submenu (`IG1`/`INT1`: arm/disarm, check status -
  shows both the I2C-polled status and the real pin side by side)
- `w` - wake-on-movement submenu (Activity/Inactivity/`INT1`)
- `c` - impact/collision submenu (`IG2`/`INT2`)
- `p` - reset the peak-hold value
- `r` - pause/resume the live stream
- `q` - quit

`lis2hh12_link.py` is the underlying serial link (background reader
thread + command protocol) and can be reused standalone, e.g. in a
Python REPL or script, if you want to script tests instead of using the
interactive dashboard.

`read_accel.py` is a minimal non-interactive alternative that just prints
parsed readings (and can send the pause/resume command), if you don't
need the full dashboard.
