# Minimal dynamic gain factor model (hopefully flight-computer friendly)
# Dynamic gain: G_d = rho * v^2 * CL_alpha / (2 * Jxx)
# Dynamic gain factor = G_d* / G_d
# " * " quantities are those at max dynamic pressure (max Q) which is where the control performance is designed
# The only thing that should change between the FULL LAUNCH and TEST LAUNCH codes are " * " quantities.
# Based on the latest simulations for FULL LAUNCH from 5 FEB 2026:
#   M* = 3.2
#   h* = 5000 m  (sets rho* and T* -> a* -> V* = M* a*)
#   t* = t_b = 9.25 s    (sets Jxx*)
#   G_d* computed from the " * " values.
#
# Units:
#   altitude input: meters (AGL expected by caller; model adds LAUNCH_ALT)
#   velocity input: m/s
#   rho: kg/m^3, T: K, a: m/s

import math

FT_TO_M = 0.3048
FTPS_TO_MPS = 0.3048

# Vehicle parameters updated 5 FEB 2026
Jxx0 = 0.27 # kg * m^2 (Roll MOI initial)
Jxxf = 0.2  # kg * m^2 (Roll MOI final)
t_b  = 9.25 # s (Burnout time)

# Velocity floor to prevent Gd -> 0 at liftoff
V_MIN = 20.0  # m/s

# Atmosphere Model (USSA 1976 model; layers valid up to 120k ft)
# INPUT: h_m (altitude AGL; function shifts by LAUNCH_ALT)
# OUTPUTS: T (temperature), rho (density), a (speed of sound)
LAUNCH_ALT = 622  # m at FAR Launch site
LAUNCH_TEMP = 305 # K at FAR Launch site (close enough for this model)

def atmosphere(h_m):
    h_m = h_m + LAUNCH_ALT
    # Clamp to model range (0–36.6 km)
    if h_m < 0.0:      h_m = 0.0
    if h_m > 36600.0: h_m = 36600.0

    g0 = 9.80665
    R  = 287.05287
    gamma = 1.4

    # Layers: (h_base, h_top, lapse_rate)
    layers = (
        (0.0,     11000.0, -0.0065),
        (11000.0, 20000.0,  0.0),
        (20000.0, 32000.0,  0.0010),
        (32000.0, 47000.0,  0.0028),
    )

    T = LAUNCH_TEMP  # K
    P = 101325.0     # Pa (model starts from sea-level)

    for h_base, h_top, L in layers:
        if h_m <= h_top:
            if L == 0.0:
                P *= math.exp(-g0 * (h_m - h_base) / (R * T))
            else:
                T_new = T + L * (h_m - h_base)
                P *= (T_new / T) ** (-g0 / (R * L))
                T = T_new
            rho = P / (R * T)
            a = math.sqrt(gamma * R * T)
            return T, rho, a

        # Advance to top of layer
        if L == 0.0:
            P *= math.exp(-g0 * (h_top - h_base) / (R * T))
        else:
            T_new = T + L * (h_top - h_base)
            P *= (T_new / T) ** (-g0 / (R * L))
            T = T_new

    rho = P / (R * T)
    a = math.sqrt(gamma * R * T)
    return T, rho, a


# CL_alpha (Lift coefficient slope: stepwise lookup table vs Mach)
M_EDGES = (0.0, 0.8, 1.2, 2.0, 2.5, 3.4)  # PLACEHOLDERS UNTIL CFD COMPLETE
CL_STEPS = (4.5, 4.0, 3.2, 2.6, 2.3, 2.0) # PLACEHOLDERS UNTIL CFD COMPLETE

def cl_alpha(mach):
    idx = 0
    for i in range(len(M_EDGES)):
        if mach >= M_EDGES[i]:
            idx = i
        else:
            break
    return CL_STEPS[idx]


# Jxx schedule (Linear Approximation of Roll MOI versus time)
def Jxx_of_t(t, Jxx0, Jxxf, t_b):
    if t <= 0.0:  return Jxx0
    if t >= t_b:  return Jxxf
    return Jxx0 + (t / t_b) * (Jxxf - Jxx0)


# Dynamic Gain calculations
def Gd(rho, v, cl_a, Jxx):
    if Jxx == 0.0:  # Catch for somehow passing in zero (should never happen)
        return 1
    return (rho * v * v * cl_a) / (2.0 * Jxx)


def Gd_star(Jxx0, Jxxf, t_b):
    M_star = 3.280581
    h_star = 16253.6 * FT_TO_M  # meters (AGL; atmosphere adds LAUNCH_ALT)

    T, rho, a = atmosphere(h_star)
    v_star = M_star * a
    cl_a = cl_alpha(M_star)
    Jxx_star = Jxx_of_t(t_b, Jxx0, Jxxf, t_b)

    return Gd(rho, v_star, cl_a, Jxx_star), v_star, rho, T


def dynamic_gain_factor(t, h, v, Jxx0, Jxxf, t_b):
    Gd_star_val, v_star, rho_star, T_star = Gd_star(Jxx0, Jxxf, t_b)

    n = len(t)
    gain = [0.0] * n

    for i in range(n):
        T, rho, a = atmosphere(h[i])

        v_eff = v[i]
        if v_eff < V_MIN:
            v_eff = V_MIN

        mach = v_eff / a if a > 0.0 else 0.0
        cla = cl_alpha(mach)
        Jxx = Jxx_of_t(t[i], Jxx0, Jxxf, t_b)
        gd = Gd(rho, v_eff, cla, Jxx)

        gain[i] = Gd_star_val / gd if gd != 0.0 else 0

    return gain, Gd_star_val, v_star, rho_star, T_star


# Example Flight Computer usage
if __name__ == "__main__":
    # Compute once at startup
    Gd_star_val, v_star, rho_star, T_star = Gd_star(Jxx0, Jxxf, t_b)

    # Each FC loop need to have these state variables:
    t = 7.5       # s (time)
    h = 12000.0   # m (altitude AGL)
    v = 5.0       # m/s (speed)

    v_eff = v if v >= V_MIN else V_MIN
    T, rho, a = atmosphere(h)
    mach = v_eff / a if a > 0.0 else 0.0
    cla = cl_alpha(mach)
    Jxx = Jxx_of_t(t, Jxx0, Jxxf, t_b)
    gd = Gd(rho, v_eff, cla, Jxx)

    dyn_gain_factor = Gd_star_val / gd if gd != 0.0 else 0

    print("Gd* =", Gd_star_val)
    print("Gd  =", gd)
    print("Dynamic gain factor =", dyn_gain_factor)