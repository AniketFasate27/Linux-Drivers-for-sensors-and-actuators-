#!/usr/bin/env python3
import argparse, math
from pathlib import Path
import numpy as np, pandas as pd, matplotlib.pyplot as plt
import allantools as at

def allan_from_csv(csv_path, out_png):
    df = pd.read_csv(csv_path, usecols=["time_s","gx_rad_s","gy_rad_s","gz_rad_s"])
    # Estimate fs from median dt
    dt = np.diff(df["time_s"].values)
    dt = dt[(dt>0) & (dt<10)]
    fs = 1.0/np.median(dt)
    print(f"Estimated sampling rate fs ≈ {fs:.3f} Hz")

    chans = {
        "gx": df["gx_rad_s"].values,
        "gy": df["gy_rad_s"].values,
        "gz": df["gz_rad_s"].values,
    }

    plt.figure()
    results = {}

    for i,(label,data) in enumerate(chans.items()):
        # overlapping Allan deviation for rate data (units: rad/s)
        # convert to "phase" by integrating? For gyro rate data, AllanTools
        # accepts 'data_type="freq"' meaning rate-like; use rate=True flag
        # Simpler: just use oadev on rate with rate=True (recent versions support).
        taus, adev, _, _ = at.oadev(data, rate=fs, data_type="freq", taus="all")

        plt.loglog(taus, adev, label=f"{label}")

        # --- Angle Random Walk N (slope -1/2) at tau=1s:
        # sigma(τ) ≈ N / sqrt(τ)  => N ≈ sigma(1s)
        # find closest tau to 1
        i1 = np.argmin(np.abs(taus-1.0))
        N = adev[i1]
        # --- Bias Instability B: minimum of sigma(τ) * sqrt(2/π)
        imin = np.argmin(adev)
        sigma_min = adev[imin]
        B = sigma_min*math.sqrt(2/math.pi)
        # --- Rate Random Walk K (slope +1/2): sigma(τ) ≈ (K / sqrt(3)) * sqrt(τ)
        # estimate K by slope fit on upper decade after the minimum
        # choose a span: taus >= 3*taus_min
        mask_rrw = taus >= (3.0*taus[imin])
        if np.sum(mask_rrw) >= 5:
            x = np.sqrt(taus[mask_rrw])
            y = adev[mask_rrw]
            # y ≈ (K/√3) * x  -> K ≈ √3 * slope
            slope = np.polyfit(x, y, 1)[0]
            K = math.sqrt(3.0)*slope
        else:
            K = float("nan")

        results[label] = dict(N=N, B=B, K=K)

    plt.grid(True, which="both")
    plt.xlabel("τ [s]")
    plt.ylabel("Allan deviation (rad/s)")
    plt.title("Fig 3: Gyro Allan Deviation")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()

    # Print the noise params in deg/s units too
    d2g = 180.0/math.pi
    print("\n--- Noise parameters ---")
    for ax,vals in results.items():
        print(f"{ax}:  N(ARW) = {vals['N']:.3e} rad/s/√Hz ({vals['N']*d2g:.3e} °/s/√Hz)")
        print(f"     B(bias) = {vals['B']:.3e} rad/s     ({vals['B']*d2g:.3e} °/s)")
        print(f"     K(RRW)  = {vals['K']:.3e} rad/s^2/√Hz ({vals['K']*d2g:.3e} °/s^2/√Hz)")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_path")
    ap.add_argument("--out", default="fig3_allan.png")
    args = ap.parse_args()
    allan_from_csv(args.csv_path, args.out)
