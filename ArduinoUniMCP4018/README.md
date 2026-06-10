# SMA Variable Voltage Controller — Firmware

PlatformIO / Arduino Uno firmware for the **SMA Variable Voltage Controller**
board (PN 008-300A-0003, Hybrid Dynamic Robotics Lab, University of Michigan).

It drives a shape-memory-alloy (SMA) actuator at programmable voltages by
setting a **TPS7A57** adjustable LDO through an **MCP4018** I²C digital
potentiometer, and measures the SMA current through a 0.33 Ω shunt read by an
**INA296A1** current-sense amplifier.

## Signal chain

```
Uno (I2C) --> MCP4018 wiper --> TPS7A57 REF --> V_LDO (~0.5..5.2 V)
                                                   |
                              0.33 ohm shunt (R1) -+--> SMA --> Q1 (SMA_EN gate) --> GND
                                                   |
                          INA296A1 (gain 10 V/V) across shunt --> ISNS_OUT
V_LDO --(R9/R11 47k/47k divider, /2)--> V_LDO sense
```

## Wiring (Arduino Uno)

The board's signal headers are single-pin flying leads, so these pins are a
wiring choice defined in `include/config.h`:

| Board net / header | Uno pin | Notes |
|--------------------|---------|-------|
| MCU_SCL (J3)       | A5      | I²C clock (fixed HW TWI) |
| MCU_SDA (J4)       | A4      | I²C data (fixed HW TWI) |
| SMA_EN (J9)        | D8      | HIGH = SMA current enabled |
| ISNS_OUT (J8)      | A0      | INA296A1 output |
| V_LDO sense (J6)   | A1      | reads V_LDO / 2 |
| VIN (J10/J11)      | —       | board power |

Change any of these in `include/config.h` if you wire them differently.

## Build & flash

```bash
pio run                 # compile
pio run -t upload       # flash the Uno
pio device monitor -b 115200
```

## Serial commands (115200 baud)

| Command | Description |
|---------|-------------|
| `help` | list commands |
| `scan` | probe the digipot on I²C; report address + variant (**req. 1**) |
| `cal` | sweep wiper 0→127, measure V_LDO at each, store table in EEPROM (**req. 2**) |
| `caldump` | print the stored calibration table as CSV |
| `setv <volts>` | set V_LDO to the nearest calibrated voltage (**req. 3**) |
| `setcode <0-127>` | set the wiper code directly |
| `read` | print V_LDO, ISNS, and computed SMA current once (**req. 4**) |
| `en <0\|1>` | enable / disable the SMA current path |
| `cycle <vH> <tH> <vC> <tC> <n>` | heat/cool cycle with CSV logging (**req. 3**) |
| `vref [volts]` | show or set the ADC reference (default 5.0 V) |

### Cycle example

`cycle 3.5 100 0.5 2000 5` → set V_LDO to 3.5 V for 100 ms (heat), then 0.5 V
for 2000 ms (cool), repeated 5 times. Press any key to abort. The firmware
streams CSV throughout:

```
t_ms,phase,target_v,wiper,vldo_v,isns_v,current_a
```

## Calibration & EEPROM

`cal` runs with the SMA disabled (unloaded LDO), measures V_LDO at every wiper
code, and writes the 128-point table to EEPROM (~258 of the Uno's 1 KB,
"kept onboard"). It auto-loads on boot, so calibration survives power cycles.
`setv` and `cycle` use this table to pick the closest wiper code for a target
voltage. The mapping is monotonic but nonlinear (it depends on the TPS7A57
reference current × digipot resistance), which is exactly why the lookup is
calibrated rather than computed.

For best accuracy, measure your actual 5 V rail and set it with `vref 4.98`
(for example) **before** running `cal`.

## Current measurement

`I_SMA = ISNS_OUT / (INA_GAIN × RSHUNT) = ISNS_OUT / (10 × 0.33)`.

Usable range is ~0–1.45 A before the INA296 output saturates near its 5 V
supply rail.

## Digipot library

`lib/MCP4018/` is a self-contained driver. The board is populated with an
MCP4018, but the included datasheet (DS20002152) is the MCP40D variant, which
uses a different I²C address and write framing. The library scans the bus
(`0x2F` → MCP4017/18/19 simple write; `0x2E`/`0x3E` → MCP40D command-code
write) and selects the right protocol automatically — so `scan` works and
`setcode`/`setv` write correctly regardless of which part is fitted.

## Visualization

`analysis/visualize.ipynb` loads a serial capture and plots V_LDO and SMA
current vs. time (with heat phases shaded) plus the calibration curve. Capture
a run with:

```bash
pio device monitor -b 115200 | tee analysis/logs/run1.csv
```

Then set `CYCLE_CSV` / `CAL_CSV` in the notebook. Install deps with
`pip install -r analysis/requirements.txt`.
