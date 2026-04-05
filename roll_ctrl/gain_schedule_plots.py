"""
Usage:
  cd [WORKING DIRECTORY] e.g "C:\\Users\\Conrad\\Rocket Team\\RT_Python_Lib\\roll_ctrl"
  python3 gain_schedule_plots.py FAR_Launch_Jan_2_wind_10mph.CSV
  python3 gain_schedule_plots.py FAR_Launch_Jan_2_wind_10mph.CSV --t 81
  python3 gain_schedule_plots.py FAR_Launch_Jan_2_wind_10mph.CSV --t 10:81
  python3 gain_schedule_plots.py FAR_Launch_Jan_2_wind_10mph.CSV --t
  python3 gain_schedule_plots.py FAR_Launch_Jan_2_wind_10mph.CSV --t :81
  python3 gain_schedule_plots.py FAR_Launch_Jan_2_wind_10mph.CSV --t 10:

Notes:
- Expects RASAero CSV columns: "Time (sec)", "Altitude (ft)", "Velocity (ft/sec)"
- Converts Altitude ft->m, Velocity ft/s->m/s
- Uses dynamic_gain_schedule_FULL_LAUNCH.py for atmosphere(), cl_alpha(), Jxx_of_t(), Gd(), Gd_star()
- Floors velocity to V_MIN (default 20 m/s) for Mach/Gd/q (and plotted velocity) to avoid huge Gd*/Gd at liftoff.
"""

import sys, csv
import matplotlib.pyplot as plt

import dynamic_gain_schedule_FULL_LAUNCH as dg

FT_TO_M = 0.3048
FTPS_TO_MPS = 0.3048

# Fallback defaults (used only if your module doesn’t define them globally)
DEFAULT_Jxx0 = 0.27
DEFAULT_Jxxf = 0.20
DEFAULT_t_b  = 9.25
DEFAULT_T_MAX = 81.0
DEFAULT_V_MIN = 20.0   # m/s floor

def parse_time_window(s):
    # Returns (t_start, t_end) where either can be None
    # s can be "81" or "10:81" or ":81" or "10:"
    if s is None:
        return (None, None)

    s = s.strip()
    if ":" in s:
        a, b = s.split(":", 1)
        t0 = float(a) if a.strip() else None
        t1 = float(b) if b.strip() else None
        return (t0, t1)
    else:
        # cutoff
        return (None, float(s))

def read_rasaero_csv(path, t0=None, t1=None):
    t = []
    h_m = []
    v_mps = []

    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                ti = float(row["Time (sec)"])
            except (KeyError, ValueError):
                continue

            # time window logic (assumes time is monotonic increasing)
            if t0 is not None and ti < t0:
                continue
            if t1 is not None and ti > t1:
                break

            try:
                hi = float(row["Altitude (ft)"]) * FT_TO_M
                vi = float(row["Velocity (ft/sec)"]) * FTPS_TO_MPS
            except (KeyError, ValueError):
                continue

            t.append(ti)
            h_m.append(hi)
            v_mps.append(vi)

    return t, h_m, v_mps

def compute_signals(t, h_m, v_mps, Jxx0, Jxxf, t_b, V_MIN):
    # Use your module’s Gd_star() function once (constant)
    Gd_star_val, v_star, rho_star, T_star = dg.Gd_star(Jxx0, Jxxf, t_b)

    gain = []
    rho = []
    q = []
    Jxx = []
    cla = []
    v_used = []   # velocity after floor (for plotting)

    for i in range(len(t)):
        T, rhoi, a = dg.atmosphere(h_m[i])

        v_eff = v_mps[i]
        if v_eff < V_MIN:
            v_eff = V_MIN

        mach = v_eff / a if a > 0.0 else 0.0
        clai = dg.cl_alpha(mach)
        Jxxi = dg.Jxx_of_t(t[i], Jxx0, Jxxf, t_b)

        gd = dg.Gd(rhoi, v_eff, clai, Jxxi)

        gain.append(Gd_star_val / gd if gd != 0.0 else 1e300)
        rho.append(rhoi)
        q.append(0.5 * rhoi * v_eff * v_eff)
        Jxx.append(Jxxi)
        cla.append(clai)
        v_used.append(v_eff)

    return Gd_star_val, gain, rho, v_used, q, Jxx, cla

def plot_all(t, gain, rho, v_mps, q, Jxx, cla):
    def one(y, ylabel, title):
        plt.figure()
        plt.plot(t, y)
        plt.xlabel("Time (s)")
        plt.ylabel(ylabel)
        plt.title(title)
        plt.grid(True)

    one(gain, "Gd*/Gd (-)", "Dynamic Gain Ratio")
    one(rho, "Density (kg/m^3)", "Atmospheric Density")
    one(v_mps, "Velocity (m/s)", "Velocity (floored)")
    one(q, "Dynamic Pressure (Pa)", "Dynamic Pressure (with floored v)")
    one(Jxx, "Jxx (kg*m^2)", "Roll Moment of Inertia (Jxx)")
    one(cla, "CL_alpha", "CL_alpha (Step Table)")

    plt.show()

def main():
    # CLI:
    #   argv[1] = csv name
    #   optional: --t <cutoff or start:end>  OR --t (uses module T_MAX or default)
    if len(sys.argv) < 2:
        print("Usage: python3 gain_schedule_plots.py your_run.CSV [--t 81 | --t 10:81 | --t]")
        sys.exit(2)

    csv_path = sys.argv[1]

    # Pull globals from your module.
    # Support BOTH naming styles:
    #   New (yours): Jxx0, Jxxf, t_b
    #   Old (mine):  JXX0, JXXF, T_B
    Jxx0 = getattr(dg, "Jxx0", getattr(dg, "JXX0", DEFAULT_Jxx0))
    Jxxf = getattr(dg, "Jxxf", getattr(dg, "JXXF", DEFAULT_Jxxf))
    t_b  = getattr(dg, "t_b",  getattr(dg, "T_B",  DEFAULT_t_b))

    # Velocity floor (use dg.V_MIN if you define it; else default 20 m/s)
    V_MIN = getattr(dg, "V_MIN", DEFAULT_V_MIN)

    # Default plot cutoff if user types --t with no value
    module_tmax = getattr(dg, "T_MAX", DEFAULT_T_MAX)

    t_arg = None
    if len(sys.argv) >= 3:
        # forms:
        #   --t 81
        #   --t 10:81
        #   --t
        #   --t=81
        #   --t=10:81
        if sys.argv[2] == "--t":
            if len(sys.argv) >= 4:
                t_arg = sys.argv[3]
            else:
                # user typed "--t" only -> use module cutoff
                t_arg = str(module_tmax)
        elif sys.argv[2].startswith("--t="):
            t_arg = sys.argv[2].split("=", 1)[1]

    t0, t1 = parse_time_window(t_arg)

    t, h_m, v_mps = read_rasaero_csv(csv_path, t0=t0, t1=t1)

    Gd_star_val, gain, rho, v_used, q, Jxx, cla = compute_signals(
        t, h_m, v_mps, Jxx0, Jxxf, t_b, V_MIN
    )

    print("Using:")
    print("  Jxx0 =", Jxx0, " Jxxf =", Jxxf, " t_b =", t_b)
    print("  V_MIN =", V_MIN, "m/s")
    print("  time window:", ("start=" + str(t0) if t0 is not None else "start=None"),
          (", end=" + str(t1) if t1 is not None else ", end=None"))
    print("  Gd*  =", Gd_star_val)

    plot_all(t, gain, rho, v_used, q, Jxx, cla)

if __name__ == "__main__":
    main()

