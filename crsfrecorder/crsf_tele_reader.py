#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path
import math

import matplotlib.pyplot as plt


# Map CLI flags -> metadata
# Each entry: packet type, field name, legend label, category
SERIES = {
    # ATTITUDE
    "attitude_pitch": ("ATTITUDE", "pitch", "Pitch (deg)", "ATTITUDE"),
    "attitude_roll": ("ATTITUDE", "roll", "Roll (deg)", "ATTITUDE"),
    "attitude_yaw": ("ATTITUDE", "yaw", "Yaw (deg)", "ATTITUDE"),
    # VARIO
    "vario_vspd": ("VARIO", "vspd", "Vertical speed (m/s)", "GPS"),
    # GPS
    "gps_lat": ("GPS", "lat", "Latitude (deg)", "GPS"),
    "gps_lon": ("GPS", "lon", "Longitude (deg)", "GPS"),
    "gps_gspd": ("GPS", "gspd", "Ground speed (m/s)", "GPS"),
    "gps_hdg": ("GPS", "hdg", "Heading (deg)", "GPS"),
    "gps_alt": ("GPS", "alt", "Altitude (m)", "GPS"),
    "gps_sats": ("GPS", "sats", "Satellites (count)", "GPS"),
    # BATTERY_SENSOR
    "battery_vbat": ("BATTERY_SENSOR", "vbat", "Voltage (V)", "BATTERY"),
    "battery_curr": ("BATTERY_SENSOR", "curr", "Current (A)", "BATTERY"),
    "battery_mah": ("BATTERY_SENSOR", "mah", "Consumed (mAh)", "BATTERY"),
    "battery_pct": ("BATTERY_SENSOR", "pct", "Battery (%)", "BATTERY"),
    # LINK_STATISTICS
    "link_rssi1": ("LINK_STATISTICS", "rssi1", "RSSI1 (dBm)", "LINK"),
    "link_rssi2": ("LINK_STATISTICS", "rssi2", "RSSI2 (dBm)", "LINK"),
    "link_lq": ("LINK_STATISTICS", "lq", "LQ", "LINK"),
    "link_snr": ("LINK_STATISTICS", "snr", "SNR (dB)", "LINK"),
    "link_antenna": ("LINK_STATISTICS", "antenna", "Antenna", "LINK"),
    "link_mode": ("LINK_STATISTICS", "mode", "Mode", "LINK"),
    "link_power": ("LINK_STATISTICS", "power", "Power", "LINK"),
    "link_downrssi": ("LINK_STATISTICS", "downrssi", "Downlink RSSI (dBm)", "LINK"),
    "link_downlq": ("LINK_STATISTICS", "downlq", "Downlink LQ", "LINK"),
    "link_downsnr": ("LINK_STATISTICS", "downsnr", "Downlink SNR (dB)", "LINK"),
}

# Order categories in the plot
CATEGORY_ORDER = ["ATTITUDE", "GPS", "BATTERY", "VARIO", "LINK"]


def parse_args():
    p = argparse.ArgumentParser(
        description="Read CRSF telemetry CSV logs and plot selected series vs elapsed time, grouped by category."
    )
    p.add_argument("logfile", help="Path to tele_log_*.csv produced by the recorder")
    p.add_argument(
        "--start_at_arm",
        action="store_true",
        help='Start at first FLIGHT_MODE where mode != "OK" (armed). Default: start at file beginning.',
    )
    # One flag per plottable series
    for flag in SERIES:
        p.add_argument(f"--{flag}", action="store_true", help=f"Plot {flag.replace('_', ' ')}")
    p.add_argument(
        "--all",
        action="store_true",
        help="Plot all numeric series (overrides individual flags).",
    )
    return p.parse_args()


def load_rows(path: Path):
    rows = []
    with path.open("r", newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for i, row in enumerate(r, 1):
            try:
                t = float(row["time"])
                ptype = row["type"].strip()
                data = json.loads(row["data"]) if row.get("data") else {}
                rows.append({"time": t, "type": ptype, "data": data})
            except Exception:
                # Skip malformed rows
                continue
    return rows


def trim_trailing_radio(rows):
    """Return rows up to the last non-RADIO_ID entry; discard RADIO_ID rows."""
    last_idx = None
    for i in range(len(rows) - 1, -1, -1):
        if rows[i]["type"] != "RADIO_ID":
            last_idx = i
            break
    if last_idx is None:
        return []
    cut = rows[: last_idx + 1]
    return [r for r in cut if r["type"] != "RADIO_ID"]


def find_arm_time(rows):
    """Return the time of the first FLIGHT_MODE where mode != 'OK'. None if not found."""
    for r in rows:
        if r["type"] == "FLIGHT_MODE":
            mode = r.get("data", {}).get("mode")
            if isinstance(mode, str) and mode != "OK":
                return r["time"]
    return None


def pick_series(args):
    if args.all:
        return list(SERIES.keys())
    chosen = [k for k in SERIES if getattr(args, k)]
    if not chosen:
        # Default set if nothing chosen
        chosen = [
            "attitude_pitch", "attitude_roll",
            "gps_gspd", "gps_alt",
            "battery_vbat",
            "vario_vspd",
            "link_rssi1", "link_rssi2", "link_lq",
        ]
    return chosen


def collect_series(rows, selected_flags, t0):
    """
    Build:
      per_flag: {flag: {"t": [..], "y": [..], "label": str, "cat": str}}
    """
    per_flag = {flag: {"t": [], "y": [], "label": SERIES[flag][2], "cat": SERIES[flag][3]} for flag in selected_flags}
    for r in rows:
        if r["time"] < t0:
            continue
        elapsed = r["time"] - t0
        ptype = r["type"]
        data = r.get("data", {})
        for flag in selected_flags:
            pt, field, _, _ = SERIES[flag]
            if ptype == pt and field in data:
                val = data[field]
                if isinstance(val, (int, float)):
                    if pt == "VARIO" and field == "vspd":
                        val *= 0.1  # old recorder divided by 10 instead of 100

                    if pt == "ATTITUDE" and field in ("pitch", "roll", "yaw"):
                        val = math.degrees(val)

                    per_flag[flag]["t"].append(elapsed)
                    per_flag[flag]["y"].append(val)
    # Drop empty series
    per_flag = {k: v for k, v in per_flag.items() if len(v["t"]) > 0}
    return per_flag


def group_by_category(per_flag, selected_flags):
    """
    Return list of (category, [flags in order]) filtered to those with data.
    Category order follows CATEGORY_ORDER, with any extras appended.
    """
    cats = {}
    for flag in selected_flags:
        if flag in per_flag:
            cat = per_flag[flag]["cat"]
            cats.setdefault(cat, []).append(flag)

    ordered_cats = []
    seen = set()
    for c in CATEGORY_ORDER:
        if c in cats:
            ordered_cats.append((c, cats[c]))
            seen.add(c)
    # Append any categories not in CATEGORY_ORDER
    for c, flags in cats.items():
        if c not in seen:
            ordered_cats.append((c, flags))
    return ordered_cats


def plot_grouped(per_flag, selected_flags, title):
    if not per_flag:
        raise SystemExit("No data matched the selected series and time window.")

    cat_groups = group_by_category(per_flag, selected_flags)
    n = len(cat_groups)
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(11, max(3, 2.5 * n)))
    if n == 1:
        axes = [axes]

    for ax, (cat, flags) in zip(axes, cat_groups):
        for flag in flags:
            t = per_flag[flag]["t"]
            y = per_flag[flag]["y"]
            label = per_flag[flag]["label"]
            ax.plot(t, y, lw=1.4, label=label)
        ax.set_ylabel(cat.title())
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize="small")
        ax.axhline(0.0, color="k", lw=0.8, alpha=0.3)

    axes[-1].set_xlabel("Time (s, elapsed)")
    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


def main():
    args = parse_args()
    path = Path(args.logfile)
    if not path.exists():
        raise SystemExit(f"File not found: {path}")

    rows_all = load_rows(path)
    if not rows_all:
        raise SystemExit("No rows loaded. Is this a valid recorder CSV?")

    rows = trim_trailing_radio(rows_all)
    if not rows:
        raise SystemExit("No non-RADIO_ID data to plot after trimming trailing RADIO_ID packets.")

    if args.start_at_arm:
        t_arm = find_arm_time(rows)
        if t_arm is None:
            t0 = rows[0]["time"]
            title_prefix = "Start=begin (no arm found)"
        else:
            t0 = t_arm
            title_prefix = "Start=arm"
    else:
        t0 = rows[0]["time"]
        title_prefix = "Start=begin"

    selected = pick_series(args)
    per_flag = collect_series(rows, selected, t0)
    title = f"{title_prefix} | {path.name}"
    plot_grouped(per_flag, selected, title)


if __name__ == "__main__":
    main()
