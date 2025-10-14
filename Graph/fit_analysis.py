# plot_vr_cmd_vs_v2_over_r_filtered.py
# ===== USER PARAMS ===========================================================
CSV_PATH        = "long_flight.csv"          # path to your CSV
SAVE_PATH       = "vr_vs_v2_over_r_filtered.png"  # set to None to show instead of save

Y_COL           = "v_r_cmd"                  # dependent (y)
V_CMD_COL       = "vel_theta"                    # commanded tangential speed (for x = v^2/r)
R_COL           = "r"                        # radius (m)
TIME_COL        = "t"                        # time (s)

# Variation control — we exclude points when VAR_REF_COL is "too wiggly"
VAR_REF_COL     = "vel_theta"                # use this for variability checks; fallback to V_CMD_COL if missing
R_MIN           = 0.15                       # discard when r < R_MIN (avoid blow-ups)

# Threshold on instantaneous variation (|dv/dt|) of VAR_REF_COL (m/s^2)
DVDT_MAX        = 2.0                        # set None to disable

# Rolling window STD filter on VAR_REF_COL (m/s)
ROLL_STD_WIN_S  = 0.80                       # window (seconds) for rolling std; set None to disable
ROLL_STD_MAX    = 0.4                        # keep samples where rolling std <= this

# Optional simple smoothing (for plotting only; fit uses filtered raw values)
SMOOTH_WIN_S    = None                       # e.g., 0.20 seconds; None to disable
# ============================================================================

import os, sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def finite(*arrs):
    m = np.ones_like(arrs[0], dtype=bool)
    for a in arrs:
        m &= np.isfinite(a)
    return m

def rolling_std(x, win_n):
    s = pd.Series(x)
    return s.rolling(win_n, center=True, min_periods=max(2, win_n//3)).std().to_numpy()

def main():
    if not os.path.isfile(CSV_PATH):
        print(f"CSV not found: {CSV_PATH}"); sys.exit(1)
    df = pd.read_csv(CSV_PATH)

    # Column sanity
    for c in [Y_COL, R_COL]:
        if c not in df.columns:
            print(f"Missing column '{c}'. Available: {list(df.columns)}"); sys.exit(1)
    v_col = V_CMD_COL if V_CMD_COL in df.columns else None
    if v_col is None:
        print(f"Missing '{V_CMD_COL}' (V_CMD_COL). Available: {list(df.columns)}"); sys.exit(1)

    # Pick reference for variability checks
    var_col = VAR_REF_COL if VAR_REF_COL in df.columns else v_col
    if VAR_REF_COL not in df.columns:
        print(f"Note: VAR_REF_COL '{VAR_REF_COL}' not found; using '{v_col}' for variability checks.")

    # Pull arrays
    y = pd.to_numeric(df[Y_COL], errors="coerce").to_numpy()
    v = pd.to_numeric(df[v_col],  errors="coerce").to_numpy()
    r = pd.to_numeric(df[R_COL],  errors="coerce").to_numpy()
    t = pd.to_numeric(df[TIME_COL], errors="coerce").to_numpy() if TIME_COL in df.columns else np.arange(len(v))
    vref = pd.to_numeric(df[var_col], errors="coerce").to_numpy()

    base_mask = finite(y, v, r, t) & (np.abs(r) >= R_MIN)
    y, v, r, t, vref = y[base_mask], v[base_mask], r[base_mask], t[base_mask], vref[base_mask]

    if len(t) < 3:
        print("Not enough valid samples after basic filtering."); sys.exit(1)

    # Estimate median dt for window conversions
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0)]
    dt_med = np.median(dt) if len(dt) else 0.02  # fallback 50 Hz

    # Build variability mask
    keep = np.ones_like(y, dtype=bool)

    # 1) Instantaneous derivative filter (|dv/dt| <= DVDT_MAX)
    if DVDT_MAX is not None:
        # np.gradient handles uneven t spacing
        dvdt = np.gradient(vref, t)
        keep &= np.abs(dvdt) <= float(DVDT_MAX)

    # 2) Rolling STD filter over a time window
    if ROLL_STD_WIN_S is not None and ROLL_STD_WIN_S > 0:
        win_n = max(3, int(round(ROLL_STD_WIN_S / dt_med)))
        rs = rolling_std(vref, win_n)
        keep &= (rs <= float(ROLL_STD_MAX)) | ~np.isfinite(rs)

    # Final filtered arrays
    y_f = y[keep]
    v_f = v[keep]
    r_f = r[keep]

    # Compute x = v^2 / r (centripetal accel); guard against r≈0
    r_safe = np.where(np.abs(r_f) < 1e-9, np.nan, r_f)
    x_f = (v_f**2) / r_safe
    m = finite(x_f, y_f)
    x_f, y_f = x_f[m], y_f[m]

    # Stats
    print(f"Total samples          : {len(base_mask)} (after basic filtering: {np.sum(base_mask)})")
    print(f"Kept after variability : {len(y_f)}")
    if len(y_f) < 2 or np.allclose(x_f, x_f.mean()):
        print("Not enough usable variation to fit a line."); sys.exit(1)

    # Linear fit: y = a*x + b
    a, b = np.polyfit(x_f, y_f, 1)
    y_pred = a*x_f + b
    ss_res = np.sum((y_f - y_pred)**2)
    ss_tot = np.sum((y_f - np.mean(y_f))**2)
    r2 = 1.0 - ss_res/ss_tot if ss_tot > 0 else np.nan

    print(f"Fit: {Y_COL} = a * ({v_col}^2 / {R_COL}) + b   using {len(y_f)} pts")
    print(f"  a (slope)     : {a:.6f}  [{Y_COL}/(m/s^2)]")
    print(f"  b (intercept) : {b:.6f}  [{Y_COL}]")
    print(f"  R^2           : {r2:.6f}")

    # Optional smoothing for plotting (doesn't affect fit)
    if SMOOTH_WIN_S and SMOOTH_WIN_S > 0:
        win_n = max(3, int(round(SMOOTH_WIN_S / dt_med)))
        y_plot = pd.Series(y_f).rolling(win_n, center=True, min_periods=1).mean().to_numpy()
        x_plot = pd.Series(x_f).rolling(win_n, center=True, min_periods=1).mean().to_numpy()
    else:
        x_plot, y_plot = x_f, y_f

    # Plot
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(x_plot, y_plot, s=12, alpha=0.75, label=f"Kept data ({Y_COL} vs {v_col}^2/{R_COL})")

    # Show rejected points faintly (for debugging)
    rejected = (~keep)[m]  # align to m mask
    if np.any(rejected):
        # Recover rejected x/y for context
        rej_idx = np.where(~keep)[0]
        y_r = y[rej_idx]
        v_r = v[rej_idx]
        r_r = r[rej_idx]
        r_rsafe = np.where(np.abs(r_r) < 1e-9, np.nan, r_r)
        x_r = (v_r**2) / r_rsafe
        mr = finite(x_r, y_r)
        if np.any(mr):
            ax.scatter(x_r[mr], y_r[mr], s=8, alpha=0.25, label="Rejected by variability", marker='x')

    # Trend line over observed x-range
    xx = np.linspace(np.nanmin(x_f), np.nanmax(x_f), 200)
    ax.plot(xx, a*xx + b, linewidth=2, label=f"Fit: y={a:.3f}x+{b:.3f} (R²={r2:.3f})")

    ax.set_xlabel(f"{v_col}^2 / {R_COL}  [m/s²]")
    ax.set_ylabel(f"{Y_COL}  [m/s]")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend()
    fig.tight_layout()

    if SAVE_PATH:
        fig.savefig(SAVE_PATH, dpi=150, bbox_inches="tight")
        print(f"Saved figure -> {SAVE_PATH}")
    else:
        plt.show()

if __name__ == "__main__":
    main()
