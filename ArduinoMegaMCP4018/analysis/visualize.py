#!/usr/bin/env python3
"""SMA Variable Voltage Controller - log visualization.

Standalone replacement for visualize.ipynb. Plots data captured over the serial
console from the firmware (src/main.cpp).

  Cycle logs        output of a `cycle` command. Columns:
                    t_ms, phase, target_v, wiper, vldo_v, isns_v, current_a[, r_sma_ohm]
  Calibration table output of `cal` / `caldump`. Columns: code, vldo_v

By default it loads the NEWEST cycle_*.csv in logs/ (the file most recently
written by record_cycle.py), draws the cycle chart, prints per-phase stats, and
draws the calibration curve if logs/cal.csv exists.

Examples
--------
  # newest run, show windows interactively
  python visualize.py

  # newest run, also save PNGs next to the log (no window)
  python visualize.py --export --no-show

  # a specific capture, save PNGs into ./out at 200 dpi
  python visualize.py --cycle logs/cycle_20260610_162140.csv --export out --dpi 200
"""

import argparse
import glob
import importlib.util
import os
import subprocess
import sys


def _ensure_deps():
    """Install any missing runtime packages into THIS interpreter.

    Mirrors the old notebook's setup cell: a fast no-op once everything is
    present. sys.executable targets the same Python running this script.
    """
    required = {"numpy": "numpy", "pandas": "pandas", "matplotlib": "matplotlib"}
    missing = [pip_name for mod, pip_name in required.items()
               if importlib.util.find_spec(mod) is None]
    if not missing:
        return
    print("Installing missing packages:", ", ".join(missing))
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-q", *missing])
    except subprocess.CalledProcessError:
        # Common on system Pythons that mark the env "externally managed".
        print("Retrying with --user ...")
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", "-q", "--user", *missing])
    print("Installed:", ", ".join(missing))


_ensure_deps()

import numpy as np
import pandas as pd
import matplotlib

# Board constants (mirror include/config.h)
RSHUNT = 0.33     # ohms
INA_GAIN = 10.0   # V/V (INA296A1)


def find_latest_cycle(logs_dir):
    """Return the newest cycle_*.csv in logs_dir, or None if there are none."""
    cyc = sorted(glob.glob(os.path.join(logs_dir, "cycle_*.csv")))
    return cyc[-1] if cyc else None


def load_capture(path):
    """Load a serial capture, tolerating '#' comments and echoed command lines.

    The first line that looks like a CSV header (has commas + letters) wins.
    """
    rows, header = [], None
    with open(path, "r", errors="ignore") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            if header is None:
                if "," in s and any(c.isalpha() for c in s):
                    header = [h.strip() for h in s.split(",")]
                continue
            parts = s.split(",")
            if len(parts) == len(header):      # skip stray/partial lines
                rows.append(parts)
    if header is None:
        raise ValueError(f"No CSV header found in {path}")
    df = pd.DataFrame(rows, columns=header)
    for c in df.columns:
        if c == "phase":
            continue
        df[c] = pd.to_numeric(df[c], errors="coerce")   # 'nan' -> NaN
    return df.dropna(how="all")


def plot_cycle(df, title):
    """Build the 3-panel cycle figure (V_LDO, current, R_SMA) with HEAT shading."""
    import matplotlib.pyplot as plt

    df = df.copy()
    df["t_s"] = df["t_ms"] / 1000.0
    if "r_sma_ohm" not in df.columns:   # older logs without the resistance column
        df["r_sma_ohm"] = np.where(
            df["current_a"] >= 0.005,
            (df["vldo_v"] - df["current_a"] * RSHUNT) / df["current_a"],
            np.nan,
        )

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    ax1.plot(df["t_s"], df["vldo_v"], color="tab:blue", lw=1.2, label="V_LDO (measured)")
    if "target_v" in df:
        ax1.plot(df["t_s"], df["target_v"], color="tab:gray", ls="--", lw=1.0, label="target V")
    ax1.set_ylabel("V_LDO [V]"); ax1.legend(loc="upper right"); ax1.grid(alpha=0.3)

    ax2.plot(df["t_s"], df["current_a"], color="tab:red", lw=1.2, label="I_SMA")
    ax2.set_ylabel("current [A]"); ax2.legend(loc="upper right"); ax2.grid(alpha=0.3)

    ax3.plot(df["t_s"], df["r_sma_ohm"], color="tab:green", lw=1.2, label="R_SMA")
    ax3.set_ylabel("R_SMA [ohm]"); ax3.set_xlabel("time [s]")
    ax3.legend(loc="upper right"); ax3.grid(alpha=0.3)

    if "phase" in df:
        in_heat = (df["phase"] == "HEAT").values
        t = df["t_s"].values
        start = None
        for i, h in enumerate(in_heat):
            if h and start is None:
                start = t[i]
            elif not h and start is not None:
                for ax in (ax1, ax2, ax3):
                    ax.axvspan(start, t[i], color="orange", alpha=0.12)
                start = None
        if start is not None:
            for ax in (ax1, ax2, ax3):
                ax.axvspan(start, t[-1], color="orange", alpha=0.12)

    fig.suptitle(title)
    fig.tight_layout()
    return fig


def phase_stats(df):
    """Return a per-phase summary DataFrame, or None if there's no phase column."""
    if "phase" not in df.columns:
        return None
    df = df.copy()
    if "r_sma_ohm" not in df.columns:
        df["r_sma_ohm"] = np.where(
            df["current_a"] >= 0.005,
            (df["vldo_v"] - df["current_a"] * RSHUNT) / df["current_a"],
            np.nan,
        )
    return df.groupby("phase").agg(
        n=("t_ms", "size"),
        vldo_mean=("vldo_v", "mean"),
        vldo_max=("vldo_v", "max"),
        current_mean=("current_a", "mean"),
        current_max=("current_a", "max"),
        r_sma_mean=("r_sma_ohm", "mean"),
        r_sma_min=("r_sma_ohm", "min"),
        r_sma_max=("r_sma_ohm", "max"),
    )


def plot_calibration(cal):
    """Build the calibration curve figure (wiper code vs. V_LDO)."""
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(9, 5))
    plt.plot(cal["code"], cal["vldo_v"], marker=".", ms=4, lw=1, color="tab:green")
    plt.xlabel("wiper code (0-127)"); plt.ylabel("V_LDO [V]")
    plt.title("MCP4018 wiper code -> V_LDO (TPS7A57)")
    plt.grid(alpha=0.3); plt.tight_layout()
    return fig


def export_path(out_dir, source_csv, suffix):
    """Build an output PNG path: <out_dir>/<source stem>_<suffix>.png."""
    stem = os.path.splitext(os.path.basename(source_csv))[0]
    return os.path.join(out_dir, f"{stem}_{suffix}.png")


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Visualize SMA controller serial-capture logs.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--cycle", metavar="PATH",
                   help="cycle CSV to plot (default: newest cycle_*.csv in --logs)")
    p.add_argument("--cal", metavar="PATH", default=None,
                   help="calibration CSV (default: <logs>/cal.csv if present)")
    p.add_argument("--logs", metavar="DIR", default=None,
                   help="logs directory (default: 'logs' next to this script)")
    p.add_argument("--export", nargs="?", const="__SOURCE__", metavar="DIR",
                   help="save PNGs. With no value, saves next to the source CSV; "
                        "otherwise saves into DIR")
    p.add_argument("--dpi", type=int, default=150, help="PNG resolution")
    p.add_argument("--no-show", action="store_true",
                   help="don't open interactive windows (useful with --export)")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    # Resolve directories relative to the script so it works from any cwd.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    logs_dir = args.logs or os.path.join(script_dir, "logs")

    # matplotlib backend: headless when we won't show windows.
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # --- pick cycle file (keeps the "latest run" behaviour) ---
    cycle_csv = args.cycle or find_latest_cycle(logs_dir)
    if not cycle_csv:
        sys.exit(f"No cycle CSV found. Looked for cycle_*.csv in {logs_dir!r}.\n"
                 f"Capture one with: python record_cycle.py --port COM4 --cycle 3.5 100 0.5 2000 5")
    if not os.path.isfile(cycle_csv):
        sys.exit(f"Cycle file not found: {cycle_csv}")
    print(f"Using cycle file: {cycle_csv}")

    figures = []   # (fig, source_csv, suffix)

    # --- cycle chart ---
    df = load_capture(cycle_csv)
    figures.append((plot_cycle(df, "SMA cycle - orange = HEAT phase"), cycle_csv, "cycle"))

    # --- per-phase stats ---
    summary = phase_stats(df)
    if summary is not None:
        print("\nPer-phase stats:")
        print(summary.to_string())

    # --- calibration curve (optional) ---
    cal_csv = args.cal or os.path.join(logs_dir, "cal.csv")
    if os.path.isfile(cal_csv):
        cal = load_capture(cal_csv)
        figures.append((plot_calibration(cal), cal_csv, "calibration"))
        print(f"\nV_LDO range: {cal['vldo_v'].min():.3f} .. {cal['vldo_v'].max():.3f} V")
    else:
        print(f"\nNo calibration capture at {cal_csv} yet. Capture one with:")
        print("  python record_cycle.py --port COM4 --command cal --until '# Calibration complete'")

    # --- export PNGs ---
    if args.export is not None:
        for fig, source_csv, suffix in figures:
            if args.export == "__SOURCE__":
                out_dir = os.path.dirname(os.path.abspath(source_csv))
            else:
                out_dir = args.export
                os.makedirs(out_dir, exist_ok=True)
            path = export_path(out_dir, source_csv, suffix)
            fig.savefig(path, dpi=args.dpi, bbox_inches="tight")
            print(f"Saved {path}")

    # --- show ---
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
