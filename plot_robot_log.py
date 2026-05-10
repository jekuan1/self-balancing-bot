#!/usr/bin/env python3
"""Plot BalanceBot run logs.

Examples:
    python3 plot_robot_log.py logs/robot_20260509_220530.csv
    python3 plot_robot_log.py logs/robot_20260509_220530.csv --save plots/run.png
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta
import os
from pathlib import Path
import sys
import tempfile


def parse_float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    text = row.get(key, "")
    if text == "":
        return default
    try:
        return float(text)
    except ValueError:
        return default


def parse_bool(row: dict[str, str], key: str, default: bool = True) -> bool:
    text = row.get(key, "")
    if text == "":
        return default
    try:
        return bool(int(float(text)))
    except ValueError:
        return text.lower() in ("true", "yes", "active")


def parse_timestamp(text: str, previous: datetime | None) -> datetime:
    current = datetime.strptime(text, "%H:%M:%S.%f")
    if previous is not None:
        while current < previous:
            current += timedelta(days=1)
    return current


def load_log(path: Path) -> dict[str, list[float]]:
    data = {
        "t": [],
        "pitch": [],
        "target": [],
        "dynamic_target": [],
        "tilt_rate": [],
        "desired_tilt_rate": [],
        "left_hz": [],
        "right_hz": [],
        "cap_hz": [],
        "saturation_pct": [],
    }

    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        previous_ts = None
        start_ts = None

        for row in reader:
            if not parse_bool(row, "is_active", default=True):
                continue

            timestamp_text = row.get("timestamp", "")
            if not timestamp_text:
                continue

            ts = parse_timestamp(timestamp_text, previous_ts)
            previous_ts = ts
            if start_ts is None:
                start_ts = ts

            control_output = parse_float(row, "control_output_hz")
            left_hz = parse_float(row, "left_step_hz", control_output)
            right_hz = parse_float(row, "right_step_hz", -control_output)

            data["t"].append((ts - start_ts).total_seconds())
            data["pitch"].append(parse_float(row, "pitch_deg"))
            data["target"].append(parse_float(row, "target_pitch_deg"))
            data["dynamic_target"].append(parse_float(row, "dynamic_target_pitch_deg", parse_float(row, "target_pitch_deg")))
            data["tilt_rate"].append(parse_float(row, "tilt_rate_dps"))
            data["desired_tilt_rate"].append(0.0)
            data["left_hz"].append(left_hz)
            data["right_hz"].append(right_hz)
            data["cap_hz"].append(parse_float(row, "max_step_hz"))
            data["saturation_pct"].append(parse_float(row, "saturation_pct"))

    return data


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot BalanceBot pitch, tilt rate, and motor commands.")
    parser.add_argument("log", type=Path, help="CSV log from monitor.py")
    parser.add_argument("--save", type=Path, help="Save plot image instead of only showing it")
    parser.add_argument("--no-show", action="store_true", help="Do not open an interactive plot window")
    args = parser.parse_args()

    if not args.log.is_file():
        print(f"[!] Log file not found: {args.log}", file=sys.stderr)
        return 2

    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "balancebot_matplotlib"))

    try:
        import matplotlib
        if args.save or args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[!] matplotlib is required. Install it with: python3 -m pip install matplotlib", file=sys.stderr)
        return 2

    data = load_log(args.log)
    if not data["t"]:
        print("[!] No active/run rows found in this log.", file=sys.stderr)
        return 1

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 9))
    fig.suptitle(args.log.name)

    axes[0].plot(data["t"], data["pitch"], label="pitch", linewidth=1.4)
    axes[0].plot(data["t"], data["target"], label="target", linestyle="--", linewidth=1.0)
    axes[0].plot(data["t"], data["dynamic_target"], label="dynamic target", linestyle=":", linewidth=1.2)
    axes[0].set_ylabel("Pitch (deg)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(data["t"], data["tilt_rate"], label="tilt rate", linewidth=1.4)
    axes[1].plot(data["t"], data["desired_tilt_rate"], label="desired", linestyle="--", linewidth=1.0)
    axes[1].set_ylabel("Tilt rate (deg/s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].plot(data["t"], data["left_hz"], label="left motor", linewidth=1.2)
    axes[2].plot(data["t"], data["right_hz"], label="right motor", linewidth=1.2)
    if any(v > 0.0 for v in data["cap_hz"]):
        axes[2].plot(data["t"], data["cap_hz"], label="+cap", linestyle="--", linewidth=0.9)
        axes[2].plot(data["t"], [-v for v in data["cap_hz"]], label="-cap", linestyle="--", linewidth=0.9)
    axes[2].set_ylabel("Motor speed (Hz)")
    axes[2].set_xlabel("Time since start (s)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right")

    fig.tight_layout()

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=160)
        print(f"[*] Saved plot to {args.save}")

    if not args.no_show:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
