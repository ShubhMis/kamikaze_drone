#!/usr/bin/env python3
"""
Plot PID tuning data from kamikaze flight logs.

Usage:
    python3 plot_flight_log.py flight_log_YYYYMMDD_HHMMSS.csv
"""

import sys
import csv
import matplotlib.pyplot as plt
import numpy as np


def load_csv(path):
    with open(path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return rows


def to_float(rows, key):
    out = []
    for r in rows:
        v = r.get(key, "")
        try:
            out.append(float(v))
        except (ValueError, TypeError):
            out.append(float("nan"))
    return np.array(out)


def main():
    if len(sys.argv) < 2:
        # Auto-find latest log
        import glob, os
        logs = sorted(glob.glob(os.path.join(os.path.dirname(__file__), "flight_log_*.csv")))
        if not logs:
            print("Usage: python3 plot_flight_log.py <flight_log.csv>")
            sys.exit(1)
        path = logs[-1]
        print(f"Using latest log: {path}")
    else:
        path = sys.argv[1]

    rows = load_csv(path)
    if not rows:
        print("Empty log file.")
        return

    t = to_float(rows, "t")
    t = t - t[0]  # Relative time

    yaw_err     = to_float(rows, "yaw_err_deg")
    pitch_err   = to_float(rows, "pitch_err_deg")
    yaw_err_db  = to_float(rows, "yaw_err_db")
    pitch_err_db= to_float(rows, "pitch_err_db")
    yaw_P       = to_float(rows, "yaw_P")
    yaw_D       = to_float(rows, "yaw_D")
    yaw_cmd     = to_float(rows, "yaw_cmd")
    pitch_P     = to_float(rows, "pitch_P")
    pitch_D     = to_float(rows, "pitch_D")
    pitch_cmd   = to_float(rows, "pitch_cmd")
    az          = to_float(rows, "az")
    vz          = to_float(rows, "vz")
    alt         = to_float(rows, "alt")
    drone_x     = to_float(rows, "drone_x")
    drone_y     = to_float(rows, "drone_y")
    tgt_X       = to_float(rows, "tgt_X")
    tgt_Y       = to_float(rows, "tgt_Y")

    fig, axes = plt.subplots(4, 2, figsize=(16, 14), sharex=True)
    fig.suptitle(f"PID Tuning — {path}", fontsize=14, fontweight="bold")

    # ── Yaw Error ──
    ax = axes[0, 0]
    ax.plot(t, yaw_err, "b-", alpha=0.4, label="raw error")
    ax.plot(t, yaw_err_db, "b-", linewidth=1.5, label="after deadband")
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.axhline(2, color="green", linewidth=0.5, linestyle="--", label="deadband")
    ax.axhline(-2, color="green", linewidth=0.5, linestyle="--")
    ax.set_ylabel("Yaw Error (°)")
    ax.legend(fontsize=8)
    ax.set_title("Yaw Error")
    ax.grid(True, alpha=0.3)

    # ── Pitch Error ──
    ax = axes[0, 1]
    ax.plot(t, pitch_err, "r-", alpha=0.4, label="raw error")
    ax.plot(t, pitch_err_db, "r-", linewidth=1.5, label="after deadband")
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.axhline(2, color="green", linewidth=0.5, linestyle="--", label="deadband")
    ax.axhline(-2, color="green", linewidth=0.5, linestyle="--")
    ax.set_ylabel("Pitch Error (°)")
    ax.legend(fontsize=8)
    ax.set_title("Pitch Error")
    ax.grid(True, alpha=0.3)

    # ── Yaw PID Components ──
    ax = axes[1, 0]
    ax.plot(t, yaw_P, label="P", color="blue", alpha=0.7)
    ax.plot(t, yaw_D, label="D", color="orange", alpha=0.7)
    ax.plot(t, yaw_cmd, label="cmd (P+D)", color="black", linewidth=1.5)
    ax.set_ylabel("Yaw PID Output")
    ax.legend(fontsize=8)
    ax.set_title("Yaw PID Components")
    ax.grid(True, alpha=0.3)

    # ── Pitch PID Components ──
    ax = axes[1, 1]
    ax.plot(t, pitch_P, label="P", color="red", alpha=0.7)
    ax.plot(t, pitch_D, label="D", color="orange", alpha=0.7)
    ax.plot(t, pitch_cmd, label="cmd (P+D)", color="black", linewidth=1.5)
    ax.set_ylabel("Pitch PID Output")
    ax.legend(fontsize=8)
    ax.set_title("Pitch PID Components")
    ax.grid(True, alpha=0.3)

    # ── Commands ──
    ax = axes[2, 0]
    ax.plot(t, az, "b-", linewidth=1.5, label="az (yaw rate)")
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.set_ylabel("az (rad/s)")
    ax.legend(fontsize=8)
    ax.set_title("Yaw Rate Command (az)")
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    ax.plot(t, vz, "r-", linewidth=1.5, label="vz (climb rate)")
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.set_ylabel("vz (m/s)")
    ax.legend(fontsize=8)
    ax.set_title("Vertical Rate Command (vz)")
    ax.grid(True, alpha=0.3)

    # ── Altitude ──
    ax = axes[3, 0]
    ax.plot(t, alt, "g-", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude")
    ax.grid(True, alpha=0.3)

    # ── Ground Track ──
    ax = axes[3, 1]
    ax.plot(drone_x, drone_y, "b-", linewidth=1, alpha=0.6, label="drone path")
    valid = ~(np.isnan(tgt_X) | np.isnan(tgt_Y))
    if valid.any():
        ax.scatter(tgt_X[valid], tgt_Y[valid], c="red", s=10, alpha=0.5, label="target est")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Ground Track (top-down)")
    ax.legend(fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = path.replace(".csv", ".png")
    plt.savefig(out_path, dpi=150)
    print(f"Saved plot → {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
