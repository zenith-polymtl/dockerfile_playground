#!/usr/bin/env python3
"""
Reworked plots:
- Figure 1 (saved): radial & tangential pairs (2x2)
  [Row 1] Radial speed vs setpoint | Radial PID effort (+acc_cmd_r)
  [Row 2] Tangential speed vs setpoint | Tangential PID effort (+acc_cmd_theta)
- Figure 2 (saved): Vertical pair (1x2)
  [Col 1] Vertical speed vs setpoint | [Col 2] Vertical PID effort (+acc_cmd_z)
- Figure 3 (saved): Yaw (1x2)
  [Col 1] Yaw vs target (+error) | [Col 2] Yaw rate
- Figure 4 (saved): Soft pos-hold latched positions & PID (2x2)
  [Row 1] Radius vs latched position | Radial PID components (P/I/D)
  [Row 2] Drone z vs target z | Vertical PID components (P/I/D)

Usage:
  python graph_analysis.py input.csv [output_prefix]
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Columns we’ll try to use (script tolerates missing ones)
KEEP = [
    "radius",
    "set_speed_r","set_speed_r_filtered","meas_speed_r","acc_cmd_r",
    "set_speed_theta","meas_speed_theta","acc_cmd_theta",
    "set_speed_z","meas_speed_z","acc_cmd_z",
    "acc_x","acc_y","acc_z",
    "pub_vel_x","pub_vel_y","pub_vel_z",
    "pid_r_P","pid_r_I","pid_r_D",
    "pid_vtheta_P","pid_vtheta_I","pid_vtheta_D",
    "pid_z_P","pid_z_I","pid_z_D",
    "yaw_enu","yaw_target","error_yaw","total_yaw_err","yaw_rate",
    # soft_pos_hold / latched positions & PID components
    "drone_z","target_z",
    "r_lock_position","r_lock_active","r_pid_P","r_pid_I","r_pid_D",
    "z_lock_position","z_lock_active","z_pid_P","z_pid_I","z_pid_D",
    # theta hold
    "theta_actual","theta_target","theta_lock_position","theta_lock_active",
    "theta_pid_P","theta_pid_I","theta_pid_D",
    "dt",
]

def col_ok(df, name): return name in df.columns

def t_axis(df):
    if "dt" in df.columns:
        t = df["dt"].to_numpy()
        t = np.cumsum(t) - t[0]
        return t, "time [s]"
    else:
        return np.arange(len(df)), "sample"

def plot_if(ax, t, df, name, style="-", label=None, lw=1.4):
    if name in df.columns:
        y = df[name].to_numpy()
        m = np.isfinite(y)
        if m.any():
            ax.plot(t[m], y[m], style, label=(label or name), linewidth=lw)
            return True
    return False

def main():
    import argparse
    p = argparse.ArgumentParser(description="PID / approach log analysis")
    p.add_argument("src", help="input CSV")
    p.add_argument("out_prefix", nargs="?", default="plots", help="output prefix for saved figures")
    p.add_argument("--tmin", type=float, default=None, help="start time (s) for plotting window")
    p.add_argument("--tmax", type=float, default=None, help="end time (s) for plotting window")
    p.add_argument("--last", type=float, default=None, help="use last N seconds as plotting window")
    args = p.parse_args()

    src = args.src
    out_prefix = args.out_prefix

    df = pd.read_csv(src)
    # Trim to known columns, keep order if present
    df = df[[c for c in KEEP if c in df.columns]].copy()

    # Time axis
    t, tlabel = t_axis(df)

    # apply user-specified time window (if any)
    t = np.asarray(t, dtype=float)
    t_min_all = float(np.nanmin(t))
    t_max_all = float(np.nanmax(t))
    if args.last is not None:
        tmax = t_max_all
        tmin = max(t_min_all, tmax - float(args.last))
    else:
        tmin = t_min_all if args.tmin is None else float(args.tmin)
        tmax = t_max_all if args.tmax is None else float(args.tmax)

    if not (tmin < tmax):
        print(f"Invalid time window: tmin={tmin} >= tmax={tmax}")
        sys.exit(1)

    mask = (t >= tmin) & (t <= tmax)
    if not np.any(mask):
        print(f"No samples in requested time window {tmin}..{tmax}")
        sys.exit(1)

    # subset dataframe and time vector to the requested window
    df = df.iloc[np.nonzero(mask)[0]].reset_index(drop=True)
    t = t[mask]
    # shift t to start at 0 (keeps previous behaviour where dt-based axis starts at zero)
    t = t - t[0]
    # now plotting will use df and t restricted to the chosen window

    # ---------- Figure 1: Radial & Tangential (2x2) ----------
    fig1, axs = plt.subplots(2, 2, figsize=(14, 8))
    fig1.suptitle("Radial & Tangential: Speed vs Setpoint and PID Effort")

    # (1,1) Radial speed
    ax = axs[0, 0]
    plot_if(ax, t, df, "set_speed_r", "-", "set_speed_r")
    plot_if(ax, t, df, "set_speed_r_filtered", "--", "set_speed_r_filtered")
    plot_if(ax, t, df, "meas_speed_r", "-", "meas_speed_r")
    ax.set_ylabel("v_r")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # (1,2) Radial effort
    ax = axs[0, 1]
    plot_if(ax, t, df, "acc_cmd_r", "-", "acc_cmd_r")
    plot_if(ax, t, df, "pid_r_P", "--", "pid_r_P")
    plot_if(ax, t, df, "pid_r_I", "--", "pid_r_I")
    plot_if(ax, t, df, "pid_r_D", "--", "pid_r_D")
    ax.set_ylabel("effort (r)")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # (2,1) Tangential speed
    ax = axs[1, 0]
    plot_if(ax, t, df, "set_speed_theta", "-", "set_speed_theta")
    plot_if(ax, t, df, "meas_speed_theta", "-", "meas_speed_theta")
    ax.set_ylabel("v_θ")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # (2,2) Tangential effort
    ax = axs[1, 1]
    plot_if(ax, t, df, "acc_cmd_theta", "-", "acc_cmd_theta")
    plot_if(ax, t, df, "pid_vtheta_P", "--", "pid_vtheta_P")
    plot_if(ax, t, df, "pid_vtheta_I", "--", "pid_vtheta_I")
    plot_if(ax, t, df, "pid_vtheta_D", "--", "pid_vtheta_D")
    ax.set_ylabel("effort (θ)")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    fig1.tight_layout(rect=[0, 0.02, 1, 0.95])
    fig1.savefig(f"{out_prefix}_radial_tangential.png", dpi=150)

    # ---------- Figure 2: Vertical (1x2) ----------
    fig2, axs2 = plt.subplots(1, 2, figsize=(14, 4.8))
    fig2.suptitle("Vertical: Speed vs Setpoint and PID Effort")

    # Speed
    ax = axs2[0]
    plot_if(ax, t, df, "set_speed_z", "-", "set_speed_z")
    plot_if(ax, t, df, "meas_speed_z", "-", "meas_speed_z")
    ax.set_ylabel("v_z")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # Effort
    ax = axs2[1]
    plot_if(ax, t, df, "acc_cmd_z", "-", "acc_cmd_z")
    plot_if(ax, t, df, "pid_z_P", "--", "pid_z_P")
    plot_if(ax, t, df, "pid_z_I", "--", "pid_z_I")
    plot_if(ax, t, df, "pid_z_D", "--", "pid_z_D")
    ax.set_ylabel("effort (z)")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    fig2.tight_layout(rect=[0, 0.02, 1, 0.95])
    fig2.savefig(f"{out_prefix}_vertical.png", dpi=150)

    # ---------- Figure 3: Yaw (1x2) ----------
    fig3, axs3 = plt.subplots(1, 2, figsize=(14, 4.8))
    fig3.suptitle("Yaw")

    # Yaw vs target (with error if present)
    ax = axs3[0]
    plot_if(ax, t, df, "yaw_target", "-", "yaw_target")
    plot_if(ax, t, df, "yaw_enu", "-", "yaw_enu")
    plot_if(ax, t, df, "error_yaw", ":", "error_yaw")
    ax.set_ylabel("yaw [rad]")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # Yaw rate
    ax = axs3[1]
    plot_if(ax, t, df, "yaw_rate", "-", "yaw_rate")
    ax.set_ylabel("yaw rate [rad/s]")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    fig3.tight_layout(rect=[0, 0.02, 1, 0.95])
    fig3.savefig(f"{out_prefix}_yaw.png", dpi=150)

    # ---------- Figure 4: Soft pos-hold latched positions & PID (2x2) ----------
    fig4, axs4 = plt.subplots(2, 2, figsize=(14, 8))
    fig4.suptitle("Soft pos-hold: Latched Position vs Actual and PID components")

    # Radial: radius vs r_lock_position
    ax = axs4[0, 0]
    plot_if(ax, t, df, "radius", "-", "radius")
    plot_if(ax, t, df, "r_lock_position", "-", "r_lock_position")
    plot_if(ax, t, df, "r_lock_active", ":", "r_lock_active")
    ax.set_ylabel("radial [m]")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # Radial PID components (latched controller)
    ax = axs4[0, 1]
    plot_if(ax, t, df, "r_pid_P", "-", "r_pid_P")
    plot_if(ax, t, df, "r_pid_I", "--", "r_pid_I")
    plot_if(ax, t, df, "r_pid_D", ":", "r_pid_D")
    ax.set_ylabel("PID (r)")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # Vertical: drone z, target z and z_lock_position
    ax = axs4[1, 0]
    plot_if(ax, t, df, "drone_z", "-", "drone_z")
    plot_if(ax, t, df, "target_z", "--", "target_z")
    plot_if(ax, t, df, "z_lock_position", "-", "z_lock_position")
    plot_if(ax, t, df, "z_lock_active", ":", "z_lock_active")
    ax.set_ylabel("z [m]")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # Vertical PID components (latched controller)
    ax = axs4[1, 1]
    plot_if(ax, t, df, "z_pid_P", "-", "z_pid_P")
    plot_if(ax, t, df, "z_pid_I", "--", "z_pid_I")
    plot_if(ax, t, df, "z_pid_D", ":", "z_pid_D")
    ax.set_ylabel("PID (z)")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    fig4.tight_layout(rect=[0, 0.02, 1, 0.95])
    fig4.savefig(f"{out_prefix}_poshold.png", dpi=150)

    # ---------- Figure 5: Theta pos-hold (1x2) ----------
    fig5, axs5 = plt.subplots(1, 2, figsize=(14, 4.8))
    fig5.suptitle("Theta pos-hold: Actual vs Latched/Target and PID components")

    # Theta actual vs latched/target
    ax = axs5[0]
    plot_if(ax, t, df, "theta_actual", "-", "theta_actual")
    plot_if(ax, t, df, "theta_target", "--", "theta_target")
    plot_if(ax, t, df, "theta_lock_position", "-", "theta_lock_position")
    plot_if(ax, t, df, "theta_lock_active", ":", "theta_lock_active")
    ax.set_ylabel("theta [rad]")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # Theta PID components
    ax = axs5[1]
    plot_if(ax, t, df, "theta_pid_P", "-", "theta_pid_P")
    plot_if(ax, t, df, "theta_pid_I", "--", "theta_pid_I")
    plot_if(ax, t, df, "theta_pid_D", ":", "theta_pid_D")
    ax.set_ylabel("PID (theta)")
    ax.set_xlabel(tlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    fig5.tight_layout(rect=[0, 0.02, 1, 0.95])
    fig5.savefig(f"{out_prefix}_theta.png", dpi=150)

    # Close figures to avoid GUI usage
    plt.close(fig1); plt.close(fig2); plt.close(fig3); plt.close(fig4); plt.close(fig5)
    print("[ok] saved:",
          f"{out_prefix}_radial_tangential.png,",
          f"{out_prefix}_vertical.png,",
          f"{out_prefix}_yaw.png,",
          f"{out_prefix}_poshold.png,",
          f"{out_prefix}_theta.png")

if __name__ == "__main__":
    main()
