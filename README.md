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

### A note on dupont wires

Loose/marginal dupont wire connections were the cause of nearly every
"weird" symptom encountered while building this (frozen sensor readings,
garbled UART commands). If you see readings that look stuck, wildly out of
range, or garbled command output, reseat the wiring - especially VCC/GND
to the breakout - before suspecting the code.

## Firmware

Located in `firmware/`, built on the [ch32fun](https://github.com/cnlohr/ch32fun)
framework (vendored as a git submodule at `firmware/ch32v003fun`).

```sh
git submodule update --init --recursive   # first time only
cd firmware
make flash      # build and flash
```

`nanoCH32V003.c` brings up I2C1, finds the LIS2HH12 (WHO_AM_I check,
tries both possible I2C addresses), and streams `ACC,x,y,z` (in mg) over
UART1 at 115200 baud, 20 times/second. It also accepts commands over the
same UART link (see below).

All firmware I/O goes over the real UART (PD5/PD6), not the WCH-LinkE's
debug channel - so `make monitor`/`minichlink -T` won't show anything
useful here. Read the UART directly (e.g. `stty -F /dev/ttyACM0 115200
raw -echo && cat /dev/ttyACM0`) or use the Python dashboard below.

### Command protocol

Send a line (`command\n`) over the UART at 115200 baud:

| Command | Effect |
|---|---|
| `STREAM ON` / `STREAM OFF` | Enable/disable the periodic `ACC,x,y,z` output |
| `ODR <0-6>` | Change the sensor's output data rate: 0=power-down, 1=10Hz, 2=50Hz, 3=100Hz, 4=200Hz, 5=400Hz, 6=800Hz |
| `SELFTEST` | Run the ST-spec self-test (electrostatic force actuation) and report PASS/FAIL per axis |
| `FIFO MODE <mode> [thresh]` | Set FIFO mode: `BYPASS`, `FIFO`, `STREAM`, `STREAM2FIFO`, `BYPASS2STREAM`, `BYPASS2FIFO`; threshold 0-31 |
| `FIFO STATUS` | Report FIFO sample count / empty / overrun / watermark flags |
| `FIFO READ` | Drain and print all samples currently buffered in the FIFO |
| `INT ON <X\|Y\|Z\|ANY> <HIGH\|LOW> [thresh_mg] [dur]` | Arm the interrupt generator on a threshold-crossing event |
| `INT OFF` | Disarm the interrupt generator |
| `INT STATUS` | Report the interrupt source register |
| `FS <2\|4\|8>` | Change the full-scale range (+-2g / +-4g / +-8g) |
| `PEAK` | Report the highest \|g\| seen since the last reset |
| `PEAK RESET` | Reset the peak-hold value |
| `HELP` | List commands |

The LIS2HH12 supports switchable full-scale ranges like most accelerometers
(2g/4g/8g here - some parts also offer 16g, this one doesn't). Changing `FS`
updates the sensor's `CTRL4` register and the firmware's mg-per-LSB
conversion factor together, so `ACC` output stays correctly scaled in mg
regardless of range.

Peak-hold (`PEAK`) tracks the highest combined magnitude
(`sqrt(x^2+y^2+z^2)`) seen on every sample, independent of whether
`STREAM` is on or off - so it keeps capturing shocks/peaks even while the
live output is paused.

Output lines include `ACC,..`, `SELFTEST,..`, `FIFOSTATUS,..`,
`FIFOSAMPLE,..`, `INTSTATUS,..`, `PEAK,..`, spontaneous `INTEVENT,..` when
an armed interrupt fires, plus `OK,..` / `ERR,..` acknowledgements.

## Python test interface

```sh
cd python
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python3 dashboard.py            # defaults to /dev/ttyACM0
.venv/bin/python3 dashboard.py /dev/ttyUSB0   # or specify a different port
```

`dashboard.py` is a terminal dashboard (curses) covering all of the above:

- Live bar-graph visualization of X/Y/Z (scaled to the current full-scale
  range) plus `|g|` magnitude and the peak-hold value
- Raw X/Y/Z readout pinned at the bottom of the screen at all times
- `s` - run self-test
- `o` - change ODR
- `g` - change full-scale range (2/4/8g)
- `f` - FIFO/stream mode submenu (set mode+threshold, check status, read samples)
- `i` - interrupt submenu (arm/disarm, check status)
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
