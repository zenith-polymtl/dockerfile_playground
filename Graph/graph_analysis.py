# plot_cmd_resp_fixed_paths.py
# ==== USER PARAMETERS (edit here) ============================================
CSV_PATH = "long_flight.csv"          # path to your logged CSV
SAVE_PATH = "cmd_vs_resp_long_flight.png"   # set to None to show instead of save
SMOOTH_WINDOW = 1                      # >1 applies rolling mean smoothing

# Time window (seconds). If LAST_SECONDS is set (e.g., 30), it overrides TMIN/TMAX.
TMIN = 120        # e.g., 120.0
TMAX = 300      # e.g., 180.0
LAST_SECONDS = None  # e.g., 30.0
# ============================================================================

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys, os

def rmse(a, b):
    m = np.isfinite(a) & np.isfinite(b)
    if not np.any(m):
        return np.nan
    return np.sqrt(np.mean((a[m] - b[m])**2))

def main():
    if not os.path.isfile(CSV_PATH):
        print(f"CSV not found: {CSV_PATH}")
        sys.exit(1)

    df = pd.read_csv(CSV_PATH)
    if "t" not in df.columns:
        print("CSV must contain a 't' column.")
        sys.exit(1)

    df = df.dropna(subset=["t"]).sort_values("t").reset_index(drop=True)

    def col(name):
        return df[name].to_numpy() if name in df.columns else None

    # Raw series
    t = col("t")
    r = col("r")
    r_hold = col("r_hold")
    vel_r_cmd = col("v_r")
    vel_r_measured = col("vel_r_measured")
    vel_r_transf = col("v_r_cmd")
    v_theta_cmd = col("v_theta")
    v_cmd = col("cmd_v")
    vel_theta_meas = col("vel_theta")
    v_z_cmd = col("v_z")
    vel_z_meas = col("vel_z")
    yaw = col("yaw")
    yaw_target = col("yaw_target")

    # ---- Time window selection (TOP-OF-FILE PARAMS) -------------------------
    if t is None or len(t) == 0:
        print("No time data.")
        sys.exit(1)

    t_min_all = float(np.nanmin(t))
    t_max_all = float(np.nanmax(t))

    if LAST_SECONDS is not None:
        tmin = max(t_max_all - float(LAST_SECONDS), t_min_all)
        tmax = t_max_all
    else:
        tmin = t_min_all if TMIN is None else float(TMIN)
        tmax = t_max_all if TMAX is None else float(TMAX)

    if not (tmin < tmax):
        print(f"Invalid time window: tmin={tmin} >= tmax={tmax}")
        sys.exit(1)

    mask = (t >= tmin) & (t <= tmax)

    def m(x):  # apply mask
        return x[mask] if x is not None else None

    # Apply mask to all series
    t = t[mask]
    r, r_hold = m(r), m(r_hold)
    vel_r_cmd, vel_r_measured, vel_r_transf = m(vel_r_cmd), m(vel_r_measured), m(vel_r_transf)
    v_theta_cmd, v_cmd, vel_theta_meas = m(v_theta_cmd), m(v_cmd), m(vel_theta_meas)
    v_z_cmd, vel_z_meas = m(v_z_cmd), m(vel_z_meas)
    yaw, yaw_target = m(yaw), m(yaw_target)

    if len(t) < 2:
        print("Selected time window contains too few samples.")
        sys.exit(1)

    # Derived responses (compute after windowing)
    v_r_resp = None
    if r is not None and len(r) > 1:
        dr_dt = np.gradient(r, t)
        v_r_resp = -dr_dt  # positive when moving inward

    omega_resp = None
    if vel_theta_meas is not None and r is not None:
        r_safe = np.where(r == 0, np.nan, r)
        omega_resp = vel_theta_meas / r_safe  # rad/s

    # Optional smoothing
    if SMOOTH_WINDOW and SMOOTH_WINDOW > 1:
        def smooth(x):
            if x is None: return None
            return pd.Series(x).rolling(SMOOTH_WINDOW, center=True, min_periods=1).mean().to_numpy()
        r = smooth(r)
        r_hold = smooth(r_hold)
        vel_r_cmd = smooth(vel_r_cmd)
        vel_r_measured = smooth(vel_r_measured)
        vel_r_transf = smooth(vel_r_transf)
        v_theta_cmd = smooth(v_theta_cmd)
        v_cmd = smooth(v_cmd)
        vel_theta_meas = smooth(vel_theta_meas)
        v_z_cmd = smooth(v_z_cmd)
        vel_z_meas = smooth(vel_z_meas)
        v_r_resp = smooth(v_r_resp)
        omega_resp = smooth(omega_resp)

    # Build plots
    fig, axs = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
    ax_idx = 0

    # Radial
    if vel_r_cmd is not None and v_r_resp is not None:
        axs[ax_idx].plot(t, vel_r_cmd, label="Pilot cmd [m/s]")
        if vel_r_measured is not None:
            axs[ax_idx].plot(t, -vel_r_measured, label="Measured [m/s]")
        if vel_r_transf is not None:
            axs[ax_idx].plot(t, -vel_r_transf, label="Processed cmd [m/s]", linewidth=0.5, color='green')
        axs[ax_idx].set_ylabel("v_r [m/s]")
        axs[ax_idx].grid(True)
        axs[ax_idx].legend()
        ax_idx += 1

    # Yaw (angles)
    if yaw_target is not None and yaw is not None:
        axs[ax_idx].plot(t, yaw_target, label="Yaw cmd [rad]")
        axs[ax_idx].plot(t, yaw, label="Measured yaw [rad]")
        axs[ax_idx].set_ylabel("yaw [rad]")
        axs[ax_idx].grid(True)
        axs[ax_idx].legend()
        ax_idx += 1

    # Tangential
    if v_theta_cmd is not None and (vel_theta_meas is not None or v_cmd is not None):
        if v_cmd is not None:
            axs[ax_idx].plot(t, v_cmd, label="Pilot cmd [m/s]")
        if vel_theta_meas is not None:
            axs[ax_idx].plot(t, vel_theta_meas, label="Measured [m/s]")
        axs[ax_idx].plot(t, v_theta_cmd, label="Processed cmd [m/s]", linewidth=0.5, color='green')
        axs[ax_idx].set_ylabel("v_theta [m/s]")
        axs[ax_idx].grid(True)
        axs[ax_idx].legend()
        ax_idx += 1

    # Vertical
    if v_z_cmd is not None and vel_z_meas is not None:
        axs[ax_idx].plot(t, v_z_cmd, label="vertical cmd (v_z) [m/s]")
        axs[ax_idx].plot(t, vel_z_meas, label="vertical resp (vel_z) [m/s]")
        axs[ax_idx].set_ylabel("v_z [m/s]")
        axs[ax_idx].grid(True)
        axs[ax_idx].legend()
        axs[ax_idx].set_title(f"Vertical: RMSE={rmse(v_z_cmd, vel_z_meas):.3f} m/s")
        ax_idx += 1

    # Range tracking
    if r is not None and r_hold is not None:
        axs[ax_idx].plot(t, r, label="r (distance) [m]")
        axs[ax_idx].plot(t, r_hold, label="r_hold [m]")
        axs[ax_idx].set_ylabel("r [m]")
        axs[ax_idx].grid(True)
        axs[ax_idx].legend()
        axs[ax_idx].set_title("Range tracking")
        ax_idx += 1

    for k in range(ax_idx, 5):
        axs[k].set_visible(False)

    # Lock x-limits to chosen window
    for k in range(ax_idx):
        axs[k].set_xlim(tmin, tmax)

    axs[max(ax_idx-1, 0)].set_xlabel("time [s]")
    fig.suptitle(f"Time window: {tmin:.2f}–{tmax:.2f} s", y=0.995)
    fig.tight_layout()

    if SAVE_PATH:
        fig.savefig(SAVE_PATH, dpi=150, bbox_inches="tight")
        print(f"Saved figure -> {SAVE_PATH}")
    else:
        plt.show()

if __name__ == "__main__":
    main()
