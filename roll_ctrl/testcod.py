# Minimal flight computer roll control model (hopefully flight-computer friendly)

# Realized Gain = sat [ K_0 * G_d*/G_d ] * 1 / k_servo
# K_0 = (sign) * Kp * e + (sign) * Kd * de/dt

# Dynamic gain: G_d = rho * v^2 * CL_alpha / (2 * Jxx)
# Dynamic gain factor = G_d* / G_d

import math


# Vehicle parameters for TEST LAUNCH updated 24 MAR 2026
Jxx0 = 0.267 # kg * m^2 (Roll MOI initial) 785.5 * 0.000292639653 * 95.7/82.3
Jxxf = 0.241  # kg * m^2 (Roll MOI final) 680.4 * 0.000292639653 * (95.7-18.5)/(82.3-18.5)
t_b  = 2.51 # s (Burnout time)
M_star = 1.24 # - (Max mach no)
h_star = 1557 * 0.3048 # m (Altitude AGL of predicted max q)
d_ref = 0.2207 # m (Distance from tabs to center of mass axis)
n_tabs = 2 # - Number of control tabs

# Velocity floor to prevent Gd -> 0 at liftoff
V_MIN = 20.0  # m/s

# Atmosphere Model (USSA 1976 model; layers valid up to 120k ft)
# INPUT: h_m (altitude AGL; function shifts by LAUNCH_ALT)
# OUTPUTS: T (temperature), rho (density), a (speed of sound)
# Units:
#   altitude input: meters (AGL expected by caller; model adds LAUNCH_ALT)
#   velocity input: m/s
#   rho: kg/m^3, T: K, a: m/s

LAUNCH_ALT = 271  # m at URRG Launch site
LAUNCH_TEMP = 290 # K at URRG Launch site (will vary day-to-day, close enough for this model)



def CMx_alpha(mach):
    """
    Input: 
        Mach no. []
    Output: 
        CMx_alpha [].   
    Notes: 
        Mx_alpha = CMx_alpha*q*S_tab*r_tab
    """
    if mach < 1:
        return 2.47539
    p1 = 2.34725224807287
    p2 = 1.04024379248907
    p3 = 1.49368646930894
    p4 = 0.779528950818373
    return p1*math.exp(-p2*(mach - p3)) + p4


def K_servo(rho, v, mach):
    """
    Inputs: 
        rho [kg/m^3]
        v [m/s]
        mach []
    Outputs:
        K_servo []
    Notes:
        K_servo represents fraction of DC command achieved in steady state
        under loaded conditions. Nondimensional quantity.
        I.e. lim t->inf (alpha) = K_servo*alpha_star
    """
    CMxa = CMx_alpha(mach)
    tau_alpha_proxy =  rho * v * v * CMxa
    deprate =  5.5830e-07
    return 1 - deprate * tau_alpha_proxy

### PD vals ###
## Reference State ("star") is 
# Jxx = .241 [kg*m^2]
# M = 1.24 []
# a = 338.52 [m/s]
# rho = 1.1329 [kg/m^3]
# CMx_alpha = 3.8356 []
# d_ref = 0.2207;  [m] (Distance from tabs to center of mass axis)
# S_ref = 0.00112161066; [m^2] (tab area)
## Corresponds to max q at test launch.

KP = 0.08444
KD = 0.02111

# Gain Marigin: 4.65
# Gain Crossover: 1.44 Hz
# Phase Marigin: 45.04°
# Phase Crossover: 5.54 Hz

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





# Jxx schedule (Linear Approximation of Roll MOI versus time)
def Jxx_of_t(t, Jxx0, Jxxf, t_b):
    if t <= 0.0:  return Jxx0
    if t >= t_b:  return Jxxf
    return Jxx0 + (t / t_b) * (Jxxf - Jxx0)



# Dynamic Gain calculations
def Gd(rho, v, CMx_alpha, Jxx):
    if Jxx == 0.0:  # Catch for somehow passing in zero (should never happen)
        return 1
    return (rho * v * v * CMx_alpha) / (2.0 * Jxx)



def Gd_star(Jxx0, Jxxf, t_b):
    global M_star,h_star
    #M_star = M_star
    #h_star = h_star  # meters (AGL; atmosphere adds LAUNCH_ALT)

    T, rho, a = atmosphere(h_star)
    v_star = M_star * a
    CMx_a = CMx_alpha(M_star)
    Jxx_star = Jxx_of_t(t_b, Jxx0, Jxxf, t_b)

    return Gd(rho, v_star, CMx_a, Jxx_star), v_star, rho, T



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
        CMx_alpha = CMx_alpha(mach)
        Jxx = Jxx_of_t(t[i], Jxx0, Jxxf, t_b)
        gd = Gd(rho, v_eff, CMx_alpha, Jxx)

        gain[i] = Gd_star_val / gd if gd != 0.0 else 0

    return gain, Gd_star_val, v_star, rho_star, T_star

# TODO: Implement servo extra gain factor for +/-10 deg authority at high loading (Connor)
def K_servo(mach):
    return None

# Example Flight Computer usage
if __name__ == "__main__":
    # COMPUTE ONCE AT STARTUP
    Kp = 1 # Connor
    Kd = 0.1 # Connor
    Gd_star_val, v_star, rho_star, T_star = Gd_star(Jxx0, Jxxf, t_b)
    d = 0.2207 # m (Distance from tabs to center of mass axis)

    # FC LOOP
    # PLACEHOLDER FOR JACK C++ CODE. TODO: Each FC loop need to have these state variables updated from sensors (Jack):
    t = 7.5       # s (time since start of flight)
    h = 12000.0   # m (altitude AGL)
    v = 5.0       # m/s (speed)

    v_eff = v if v >= V_MIN else V_MIN # Floor velocity to prevent Gd ratio -> INF at liftoff
    T, rho, a = atmosphere(h) # Inferred atmosphere parameters from known state variables (our sensors)
    mach = v_eff / a if a > 0.0 else 0.0 # Mach number calculation
    CMx_a = CMx_alpha(mach)#, d_ref)
    Jxx = Jxx_of_t(t, Jxx0, Jxxf, t_b) # Current roll moment of inertia from fuel burn
    Gd_val = Gd(rho, v_eff, CMx_a, Jxx)

    # Final control equation TODO: Pass in e, de/dt, make sure shit has correct sign and units (Jack)
    #K_0 = Kp * e + Kd * de/dt
    #pd_output = min(K_0 * Gd_star_val/Gd_val, 10) * 1 / K_servo(mach) # Additional gain factor (units degrees but check this)
