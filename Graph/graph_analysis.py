# plot_cmd_resp_fixed_paths.py
# ==== USER PARAMETERS (edit here) ============================================
CSV_PATH = "approach_log_polar.csv"          # path to your logged CSV
SAVE_PATH = "approach_log_polar.png"   # set to None to show instead of save
SMOOTH_WINDOW = 1                      # >1 applies rolling mean smoothing

# Time window (seconds). If LAST_SECONDS is set (e.g., 30), it overrides TMIN/TMAX.
TMIN = None      # e.g., 120.0
TMAX = None  # e.g., 180.0
LAST_SECONDS = None  # e.g., 30.0

# NEW: acceleration plotting config
PLOT_ACCEL = True
SAVE_ACC_PATH = "cmd_vs_acc_long_flight.png"   # set to None to show instead of save
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
    # use measured column names as logged by position.py
    vel_r_measured = col("vel_r_measured")
    vel_r_transf = col("v_r_cmd")
    v_theta_cmd = col("v_theta")
    v_cmd = col("cmd_v")
    # vel_theta_meas = col("vel_theta")
    vel_theta_meas = col("vel_theta_measured")
    v_z_cmd = col("v_z")
    # vel_z_meas = col("vel_z")
    vel_z_meas = col("vel_z_measured")
    yaw = col("yaw")
    yaw_target = col("yaw_target")

    # --- NEW: read logged setpoints / filtered setpoint --------------------
    set_v_r = col("set_v_r")
    set_v_theta = col("set_v_theta")
    set_v_z = col("set_v_z")
    filtered_v_r = col("filtered_v_r")
    # -----------------------------------------------------------------------

    # --- NEW: acceleration command / component columns (optional) ----------
    cmd_acc_r = col("cmd_acc_r")
    cmd_acc_theta = col("cmd_acc_theta")
    cmd_acc_z = col("cmd_acc_z")
    acc_x = col("acc_x"); acc_y = col("acc_y"); acc_z = col("acc_z")
    pub_vel_x = col("pub_vel_x"); pub_vel_y = col("pub_vel_y"); pub_vel_z = col("pub_vel_z")
    # -----------------------------------------------------------------------

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

    # --- APPLY MASK TO NEW SETPOINT COLUMNS -------------------------------
    set_v_r = m(set_v_r); set_v_theta = m(set_v_theta); set_v_z = m(set_v_z)
    filtered_v_r = m(filtered_v_r)
    # -----------------------------------------------------------------------

    # --- APPLY MASK TO NEW ACCEL COLUMNS ----------------------------------
    cmd_acc_r = m(cmd_acc_r); cmd_acc_theta = m(cmd_acc_theta); cmd_acc_z = m(cmd_acc_z)
    acc_x = m(acc_x); acc_y = m(acc_y); acc_z = m(acc_z)
    pub_vel_x = m(pub_vel_x); pub_vel_y = m(pub_vel_y); pub_vel_z = m(pub_vel_z)
    # -----------------------------------------------------------------------

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
        # --- smooth setpoint series too ---
        set_v_r = smooth(set_v_r)
        set_v_theta = smooth(set_v_theta)
        set_v_z = smooth(set_v_z)
        filtered_v_r = smooth(filtered_v_r)
        # -------------------------------------------------------------------

    # Build plots
    fig, axs = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
    ax_idx = 0

    # Radial (speed) with acceleration overlay
    if vel_r_cmd is not None and v_r_resp is not None:
        ax = axs[ax_idx]
        ax.plot(t, vel_r_cmd, label="Pilot cmd [m/s]")
        # plot measured radial speed with same sign as logged (removed previous negation)
        if vel_r_measured is not None:
            ax.plot(t, vel_r_measured, label="Measured [m/s]")
        if vel_r_transf is not None:
            ax.plot(t, vel_r_transf, label="Processed cmd [m/s]", linewidth=0.5, color='green')
        ax.set_ylabel("v_r [m/s]")
        ax.grid(True)
        # secondary axis for acceleration command overlay
        ax_a = ax.twinx()
        if cmd_acc_r is not None:
            ax_a.plot(t, cmd_acc_r, label="cmd_acc_r [m/s²]", color='C3', linewidth=0.8, alpha=0.9)
            ax_a.set_ylabel("a_r [m/s²]", color='C3')
            ax_a.tick_params(axis='y', labelcolor='C3')
        # merge legends
        lines, labels = ax.get_legend_handles_labels()
        if cmd_acc_r is not None:
            la, lb = ax_a.get_legend_handles_labels()
            lines += la; labels += lb
        ax.legend(lines, labels, loc='best')
        # annotate speed RMSE vs response (v_r_resp is derived from r)
        try:
            ax.set_title(f"Radial speed RMSE = {rmse(vel_r_cmd, v_r_resp):.3f} m/s")
        except Exception:
            pass
        ax_idx += 1

    # Yaw (angles)
    if yaw_target is not None and yaw is not None:
        axs[ax_idx].plot(t, yaw_target, label="Yaw cmd [rad]")
        axs[ax_idx].plot(t, yaw, label="Measured yaw [rad]")
        axs[ax_idx].set_ylabel("yaw [rad]")
        axs[ax_idx].grid(True)
        axs[ax_idx].legend()
        ax_idx += 1

    # Tangential (speed) with acceleration overlay
    if v_theta_cmd is not None and (vel_theta_meas is not None or v_cmd is not None):
        ax = axs[ax_idx]
        if v_cmd is not None:
            ax.plot(t, v_cmd, label="Pilot cmd [m/s]")
        if vel_theta_meas is not None:
            ax.plot(t, vel_theta_meas, label="Measured [m/s]")
        ax.plot(t, v_theta_cmd, label="Processed cmd [m/s]", linewidth=0.5, color='green')
        ax.set_ylabel("v_theta [m/s]")
        ax.grid(True)
        # accel overlay
        ax_a = ax.twinx()
        if cmd_acc_theta is not None:
            ax_a.plot(t, cmd_acc_theta, label="cmd_acc_theta [m/s²]", color='C3', linewidth=0.8, alpha=0.9)
            ax_a.set_ylabel("a_theta [m/s²]", color='C3')
            ax_a.tick_params(axis='y', labelcolor='C3')
        lines, labels = ax.get_legend_handles_labels()
        if cmd_acc_theta is not None:
            la, lb = ax_a.get_legend_handles_labels()
            lines += la; labels += lb
        ax.legend(lines, labels, loc='best')
        # speed RMSE vs measured tangential speed if available
        try:
            if v_theta_cmd is not None and vel_theta_meas is not None:
                axs[ax_idx].set_title(f"Tangential speed RMSE = {rmse(v_theta_cmd, vel_theta_meas):.3f} m/s")
        except Exception:
            pass
        ax_idx += 1

    # Vertical (speed) with acceleration overlay
    if v_z_cmd is not None and vel_z_meas is not None:
        ax = axs[ax_idx]
        ax.plot(t, v_z_cmd, label="vertical cmd (v_z) [m/s]")
        ax.plot(t, vel_z_meas, label="vertical resp (vel_z) [m/s]")
        ax.set_ylabel("v_z [m/s]")
        ax.grid(True)
        # accel overlay
        ax_a = ax.twinx()
        if cmd_acc_z is not None:
            ax_a.plot(t, cmd_acc_z, label="cmd_acc_z [m/s²]", color='C3', linewidth=0.8, alpha=0.9)
            ax_a.set_ylabel("a_z [m/s²]", color='C3')
            ax_a.tick_params(axis='y', labelcolor='C3')
        lines, labels = ax.get_legend_handles_labels()
        if cmd_acc_z is not None:
            la, lb = ax_a.get_legend_handles_labels()
            lines += la; labels += lb
        ax.legend(lines, labels, loc='best')
        try:
            axs[ax_idx].set_title(f"Vertical speed RMSE = {rmse(v_z_cmd, vel_z_meas):.3f} m/s")
        except Exception:
            pass
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

    # --- NEW: Speed tracking plots (setpoint vs achieved) -----------------
    # We'll plot time series + scatter (achieved vs setpoint) for v_r, v_theta, v_z.
    def plot_speed_vs_set(ax_ts, ax_scatter, time, setpoint, achieved, label_set, label_achieve):
        # time-series overlay
        if setpoint is not None:
            ax_ts.plot(time, setpoint, label=label_set, linewidth=1.2)
        if achieved is not None:
            ax_ts.plot(time, achieved, label=label_achieve, linewidth=1.0, alpha=0.9)
        ax_ts.grid(True); ax_ts.legend(loc="upper left")
        # scatter + identity
        if setpoint is not None and achieved is not None:
            # align finite indices
            m = np.isfinite(setpoint) & np.isfinite(achieved)
            if np.any(m):
                ax_scatter.scatter(setpoint[m], achieved[m], s=6, alpha=0.6)
                mn = np.nanmin(np.concatenate([setpoint[m], achieved[m]]))
                mx = np.nanmax(np.concatenate([setpoint[m], achieved[m]]))
                ax_scatter.plot([mn, mx], [mn, mx], 'k--', linewidth=0.8)
                ax_scatter.set_xlim(mn, mx); ax_scatter.set_ylim(mn, mx)
                ax_scatter.set_xlabel("setpoint")
                ax_scatter.set_ylabel("achieved")
                ax_scatter.set_title(f"RMSE = {rmse(setpoint[m], achieved[m]):.3f}")
        else:
            ax_scatter.set_visible(False)

    # Prepare achieved radial speed: prefer measured radial projection, else use derived v_r_resp
    achieved_vr = None
    # use measured radial speed as-is (sign convention now follows logged value)
    if vel_r_measured is not None:
        achieved_vr = vel_r_measured
    elif v_r_resp is not None:
        achieved_vr = v_r_resp

    # create a figure with 3 rows, each has a time series (left) and a small scatter to the right
    fig_s, axs_s = plt.subplots(3, 2, figsize=(12, 9), gridspec_kw={"width_ratios":[3,1]})
    times = t

    # Radial: set_v_r or filtered_v_r vs achieved_vr
    setpoint_vr = set_v_r if set_v_r is not None else filtered_v_r
    plot_speed_vs_set(axs_s[0,0], axs_s[0,1], times, setpoint_vr, achieved_vr, "set v_r [m/s]", "achieved v_r [m/s]")
    axs_s[0,0].set_ylabel("v_r [m/s]")

    # Tangential: set_v_theta vs measured tangential speed
    plot_speed_vs_set(axs_s[1,0], axs_s[1,1], times, set_v_theta, vel_theta_meas, "set v_theta [m/s]", "achieved v_theta [m/s]")
    axs_s[1,0].set_ylabel("v_theta [m/s]")

    # Vertical: set_v_z vs measured vertical speed
    plot_speed_vs_set(axs_s[2,0], axs_s[2,1], times, set_v_z, vel_z_meas, "set v_z [m/s]", "achieved v_z [m/s]")
    axs_s[2,0].set_ylabel("v_z [m/s]")

    fig_s.suptitle(f"Setpoint vs Achieved speeds ({tmin:.2f}–{tmax:.2f} s)", y=0.995)
    fig_s.tight_layout(rect=[0,0,1,0.97])

    # save or show
    spath = SAVE_ACC_PATH if SAVE_ACC_PATH else None
    if spath:
        # name it sensibly
        base = os.path.splitext(spath)[0]
        out = base + "_speeds.png"
        fig_s.savefig(out, dpi=150, bbox_inches="tight")
        print(f"Saved speed comparison -> {out}")
    else:
        plt.show()
    # -----------------------------------------------------------------------

    # --- NEW: Acceleration plots (command vs measured) -----------------------
    if PLOT_ACCEL:
        # read columns related to commanded accelerations / acceleration components
        acc_x_cmd = col("acc_x")
        acc_y_cmd = col("acc_y")
        acc_z_cmd = col("acc_z")
        cmd_acc_r = col("cmd_acc_r")
        cmd_acc_theta = col("cmd_acc_theta")
        cmd_acc_z = col("cmd_acc_z")

        # measured velocity columns (may be needed for accel computation)
        vel_r_measured = col("vel_r_measured")
        vel_theta_meas = col("vel_theta")
        vel_z_meas = col("vel_z")
        pub_vel_x = col("pub_vel_x")
        pub_vel_y = col("pub_vel_y")
        pub_vel_z = col("pub_vel_z")
        r = col("r")

        # Apply same mask/time-window
        def m(x): return x[mask] if x is not None else None
        acc_x_cmd = m(acc_x_cmd); acc_y_cmd = m(acc_y_cmd); acc_z_cmd = m(acc_z_cmd)
        cmd_acc_r = m(cmd_acc_r); cmd_acc_theta = m(cmd_acc_theta); cmd_acc_z = m(cmd_acc_z)
        vel_r_measured = m(vel_r_measured); vel_theta_meas = m(vel_theta_meas); vel_z_meas = m(vel_z_meas)
        pub_vel_x = m(pub_vel_x); pub_vel_y = m(pub_vel_y); pub_vel_z = m(pub_vel_z)
        r = m(r)

        # optionally smooth same as other series
        if SMOOTH_WINDOW and SMOOTH_WINDOW > 1:
            def smooth(x):
                if x is None: return None
                return pd.Series(x).rolling(SMOOTH_WINDOW, center=True, min_periods=1).mean().to_numpy()
            acc_x_cmd = smooth(acc_x_cmd); acc_y_cmd = smooth(acc_y_cmd); acc_z_cmd = smooth(acc_z_cmd)
            cmd_acc_r = smooth(cmd_acc_r); cmd_acc_theta = smooth(cmd_acc_theta); cmd_acc_z = smooth(cmd_acc_z)
            vel_r_measured = smooth(vel_r_measured); vel_theta_meas = smooth(vel_theta_meas); vel_z_meas = smooth(vel_z_meas)
            pub_vel_x = smooth(pub_vel_x); pub_vel_y = smooth(pub_vel_y); pub_vel_z = smooth(pub_vel_z)
            r = smooth(r)

        # Compute measured accelerations where possible
        a_x_meas = np.gradient(pub_vel_x, t) if pub_vel_x is not None else None
        a_y_meas = np.gradient(pub_vel_y, t) if pub_vel_y is not None else None
        a_z_meas_from_pub = np.gradient(pub_vel_z, t) if pub_vel_z is not None else None

        # Polar accelerations
        a_r_meas_total = None
        if vel_r_measured is not None and vel_theta_meas is not None and r is not None:
            dvelr_dt = np.gradient(vel_r_measured, t)
            centripetal = np.where(r == 0, np.nan, (vel_theta_meas**2) / r)
            a_r_meas_total = -dvelr_dt + centripetal

        a_theta_meas = np.gradient(vel_theta_meas, t) if vel_theta_meas is not None else None
        a_z_meas = np.gradient(vel_z_meas, t) if vel_z_meas is not None else None

        # Build 6-axis acceleration figure: ENU (x,y,z) + polar (radial, tangential, vertical)
        fig2, axs2 = plt.subplots(6, 1, figsize=(11, 14), sharex=True)
        ia = 0

        # Helper to plot and compute RMSE if both present
        def plot_and_rmse(ax, t, cmd, meas, cmd_label, meas_label, ylabel):
            plotted = False
            if cmd is not None:
                ax.plot(t, cmd, label=cmd_label)
                plotted = True
            if meas is not None:
                ax.plot(t, meas, label=meas_label)
                plotted = True
            ax.set_ylabel(ylabel)
            ax.grid(True)
            if cmd is not None and meas is not None:
                ax.set_title(f"RMSE = {rmse(cmd, meas):.3f}")
            return plotted

        # ENU X
        if (acc_x_cmd is not None) or (a_x_meas is not None):
            plot_and_rmse(axs2[ia], t, acc_x_cmd, a_x_meas, "cmd_acc_x [m/s²]", "meas_acc_x [m/s²]", "acc_x [m/s²]")
            axs2[ia].legend()
            ia += 1

        # ENU Y
        if (acc_y_cmd is not None) or (a_y_meas is not None):
            plot_and_rmse(axs2[ia], t, acc_y_cmd, a_y_meas, "cmd_acc_y [m/s²]", "meas_acc_y [m/s²]", "acc_y [m/s²]")
            axs2[ia].legend()
            ia += 1

        # ENU Z
        if (acc_z_cmd is not None) or (a_z_meas_from_pub is not None):
            plot_and_rmse(axs2[ia], t, acc_z_cmd, a_z_meas_from_pub, "cmd_acc_z [m/s²]", "meas_acc_z [m/s²]", "acc_z [m/s²]")
            axs2[ia].legend()
            ia += 1

        # Radial (polar)
        if (cmd_acc_r is not None) or (a_r_meas_total is not None):
            plot_and_rmse(axs2[ia], t, cmd_acc_r, a_r_meas_total, "cmd_acc_r [m/s²]", "meas_radial_acc [m/s²]", "a_r [m/s²]")
            axs2[ia].legend()
            ia += 1

        # Tangential (polar)
        if (cmd_acc_theta is not None) or (a_theta_meas is not None):
            plot_and_rmse(axs2[ia], t, cmd_acc_theta, a_theta_meas, "cmd_acc_theta [m/s²]", "meas_tangential_acc [m/s²]", "a_theta [m/s²]")
            axs2[ia].legend()
            ia += 1

        # Vertical (polar)
        if (cmd_acc_z is not None) or (a_z_meas is not None):
            plot_and_rmse(axs2[ia], t, cmd_acc_z, a_z_meas, "cmd_acc_z [m/s²]", "meas_vertical_acc [m/s²]", "a_z [m/s²]")
            axs2[ia].legend()
            ia += 1

        # Hide unused axes
        for k in range(ia, 6):
            axs2[k].set_visible(False)

        # Lock x-limits to chosen window
        for k in range(ia):
            axs2[k].set_xlim(tmin, tmax)

        axs2[max(ia-1, 0)].set_xlabel("time [s]")
        fig2.suptitle(f"Accel: ENU & polar (cmd vs measured) {tmin:.2f}–{tmax:.2f} s", y=0.995)
        fig2.tight_layout()

        if SAVE_ACC_PATH:
            fig2.savefig(SAVE_ACC_PATH, dpi=150, bbox_inches="tight")
            print(f"Saved acceleration figure -> {SAVE_ACC_PATH}")
        else:
            plt.show()

if __name__ == "__main__":
    main()
