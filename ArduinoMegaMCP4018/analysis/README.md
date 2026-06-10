# Analysis & Logging

Host-side tools for the SMA Variable Voltage Controller: an auto-recorder that
captures serial output to CSV, and a Jupyter notebook to plot it.

## Setup

```bash
pip install -r requirements.txt
```

(`requirements.txt`: pandas, numpy, matplotlib, jupyter, pyserial.)

Find your board's serial port first:

```bash
pio device list          # look for the Arduino Mega 2560 entry (e.g. COM4)
```

> Close any open serial monitor before running the recorder — only one program
> can hold the port at a time.

## `record_cycle.py` — auto-record

The firmware streams CSV over USB but can't write files on your PC, so this
script opens the port, sends a command, captures the output, and saves a
timestamped CSV into `logs/`.

### Single cycle

```bash
python record_cycle.py --port COM4 --cycle 3.5 100 0.5 2000 5
```

Sends `cycle 3.5 100 0.5 2000 5` (heat 3.5 V for 100 ms, cool 0.5 V for 2000 ms,
5 times), captures until `# END cycle`, and writes `logs/cycle_<timestamp>.csv`.

### Unattended / repeated logging

```bash
# 20 runs, 60 s apart
python record_cycle.py --port COM4 --cycle 3.5 100 0.5 2000 5 --repeat 20 --interval 60

# repeat forever until Ctrl-C, 5 min apart
python record_cycle.py --port COM4 --cycle 3.5 100 0.5 2000 5 --repeat 0 --interval 300
```

Each run is saved to its own `logs/cycle_<timestamp>.csv`. The port stays open
across runs, so the board only reboots once (at the start), not every run.

### Capture a calibration table

```bash
python record_cycle.py --port COM4 --command cal --until "# Calibration complete"
```

Saves `logs/cal_<timestamp>.csv` (rename to `logs/cal.csv` to use it as the
notebook's default calibration file).

### Free-running capture (no command sent)

```bash
python record_cycle.py --port COM4 --duration 30        # record 30 s of output
```

### Options

| Flag | Meaning | Default |
|------|---------|---------|
| `--port` | serial port (required) | — |
| `--baud` | baud rate | 115200 |
| `--cycle vH tH vC tC n` | shortcut for a `cycle` command | — |
| `--command "..."` | any raw command (`cal`, `read`, `setv 3.0`, …) | — |
| `--until "text"` | stop a capture when a line contains this | `# END cycle` |
| `--duration S` | capture for S seconds instead of `--until` | 0 (off) |
| `--repeat N` | number of runs; `0` = forever until Ctrl-C | 1 |
| `--interval S` | seconds to wait between runs | 0 |
| `--outdir DIR` | where to save CSVs | `analysis/logs` |

> Opening the port toggles DTR, which resets the Mega. The script waits 2 s for
> the reboot before sending the first command — this is expected.

## `visualize.ipynb` — plots

```bash
jupyter notebook visualize.ipynb
```

- Auto-loads the newest `logs/cycle_*.csv`. Override `CYCLE_CSV` / `CAL_CSV` in
  the second cell to pick a specific file.
- Plots three stacked panels — **V_LDO**, **SMA current**, **SMA resistance** —
  vs. time, with HEAT phases shaded orange.
- Prints a per-phase summary table (means/min/max, including resistance).
- Plots the calibration curve (wiper code → V_LDO) if a `cal` capture is present.

The loader skips `#` comment lines and the echoed command, so files captured
either with this script or with `pio device monitor | tee` both work.

## CSV column reference

**Cycle log** (`cycle_*.csv`):

| Column | Units | Meaning |
|--------|-------|---------|
| `t_ms` | ms | time since cycle start |
| `phase` | — | `HEAT` or `COOL` |
| `target_v` | V | commanded V_LDO for the phase |
| `wiper` | 0–127 | digipot code chosen for that target |
| `vldo_v` | V | measured LDO output |
| `isns_v` | V | INA296A1 output voltage |
| `current_a` | A | SMA current = `isns_v / (10 × 0.33)` |
| `r_sma_ohm` | Ω | SMA resistance = `(vldo_v − current_a×0.33) / current_a`; `nan` when current < 5 mA |

**Calibration table** (`cal_*.csv`): `code` (0–127), `vldo_v` (measured unloaded LDO output).
