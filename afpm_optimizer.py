#!/usr/bin/env python3
"""
axial_flux_optimizer.py
Coreless, double-rotor, single-stator axial-flux PMSM optimizer for the ASME e-HPVC mid-drive.

Electromagnetics = the WiredWhite axial-flux tool (Kamper / Parviainen radial-slice model):
  concentrated or overlap winding, wye connection, sinusoidal FOC drive (id = 0),
  ideal SVPWM voltage limit.

Added on top (these parts are MINE, not the website's):
  * 500 W SHAFT-OUTPUT cap (inspector measures OUT) enforced over the whole speed range
  * 48 V voltage limit -> torque/speed envelope
  * mass model (magnets, back-iron, copper, potting, structure, reduction stage)
  * 0 -> v_target acceleration sim: 29" wheel, your cassette, belt/chain reduction to the chainring
  * launch heating check + continuous (rated-point) heating check
  * winding eddy-loss estimate (coreless windings sit in the magnet field)
  * differential-evolution optimizer

NOT modeled (same limits as the website): cogging, harmonics, saturation (beyond a back-iron
sizing rule), demagnetization, structural limits, bearing life, true 3-D fringing.
The website only validates at 400 mm. A 150-220 mm motor is an extrapolation: measure back-EMF
on the prototype and re-tune MAT["field_derate"] (see README notes in the chat).

pip install numpy scipy matplotlib
python axial_flux_optimizer.py
"""
import math
import numpy as np
from scipy.optimize import differential_evolution
import matplotlib.pyplot as plt

MU0 = 4e-7 * math.pi
RHO20 = 1.724e-8      # copper resistivity at 20 C [ohm m]
ALPHA_CU = 0.0038     # copper temperature coefficient [1/K]
RHO_CU = 8960.0       # copper density [kg/m3]
GRAV = 9.81
MPH = 0.44704

# =====================================================================
# 1) INPUTS - EDIT THESE
# =====================================================================
RULES = dict(
    P_cap_W=500.0,       # SHAFT OUTPUT cap. Inspector measures OUT, so this is shaft watts.
    Vdc=48.0,            # rated pack voltage (rule limit: 50 V rated). Use the RATED number.
    vdc_util=0.90,       # usable fraction of ideal SVPWM voltage (pack sag + dead time + headroom)
    eta_inv=0.97,        # inverter efficiency (only used for the battery-current estimate)
    batt_sag=0.90,       # pack volts under load / nominal (battery-current estimate)
    I_batt_max_A=20.0,   # BMS / pack discharge limit
)

VEHICLE = dict(
    m_base_kg=100.0,         # bike + rider + everything EXCEPT motor and reduction stage
    wheel_dia_m=0.737,       # 29" wheel + tire outer diameter (measure yours)
    wheel_inertia_kg=4.0,    # equivalent translating mass of wheel rotation
    crr=0.005,               # rolling resistance coefficient
    CdA=0.10,                # drag area [m2]
    rho_air=1.18,
    rider_power_W=0.0,       # extra rider power at the wheel (0 = motor-only worst case)
    rider_force_max_N=150.0,
    traction_limit_N=400.0,  # max drive force before the tire spins
    v_target_mph=20.0,       # acceleration target
    n_launches=1,            # back-to-back launches for the heating check
)

DRIVE = dict(
    design_cadence_rpm=85.0,   # chainring rpm at which the motor runs at its rated speed
                               # (sets the motor->chainring reduction R_mc = n_rated / cadence)
    chainring_T=46,
    cassette_T=[11, 13, 15, 17, 20, 23, 26, 30, 34],
    max_chainring_rpm=130.0,   # pedals turn with the chainring when the motor drives it
    chain_eff=0.95,
    red_eff=0.97,              # efficiency of the motor->chainring reduction stage
    R_mc_min=1.5,
    R_mc_max=10.0,             # biggest reduction you are willing to build (2 stages?)
    red_mass_fixed_kg=0.30,
    red_mass_per_ratio_kg=0.06,
)

THERMAL = dict(
    T_amb_C=35.0,
    T_cu_max_C=110.0,          # winding limit (epoxy and magnets are right next to it)
    T_cu_run_C=90.0,           # copper temperature used for resistance everywhere
    T_start_C=50.0,            # winding temperature at the start of a launch
    h_eff=25.0,                # W/m2K effective stator cooling (rotor-driven air). ROUGH WE HAVE TO TEST THIS.
    stator_heat_budget_W=None, # set a number to override the h_eff estimate
    c_cu=385.0, c_epoxy=1100.0,
    thermal_mass_fraction=0.8, # fraction of winding thermal mass that absorbs launch heat
)

MAT = dict(
    winding="concentrated",    # "concentrated" (3 coils/4 poles, poles % 4 == 0) or "overlap"
    a_paths=1,                 # parallel circuits; must divide coils per phase
    width_fill=1.0,            # fraction of max coil-side width (NOT copper fill)
    g_air_mm=1.2,              # mechanical clearance per side. A build tolerance, not optimized.
    Brem_T=1.2,                # magnet remanence AT OPERATING TEMPERATURE
    mu_rec=1.05,
    alpha_p=0.8,               # magnet arc / pole pitch
    field_derate=0.75,         # real fundamental / ideal 1-D estimate. Website's example implies ~0.74.
                               # >>> THE number to calibrate with a back-EMF test or FEM <<<
    rho_mag=7500.0, rho_steel=7850.0, rho_epoxy=1600.0,
    B_backiron_T=1.5, t_backiron_min_mm=3.0,
    struct_fixed_kg=0.50,      # shaft, bearings, housing, bolts
    struct_frac=0.20,          # extra structure as a fraction of active mass
    kL=0.42,                   # inductance estimate scale, calibrated to the website's 0.30 mH example
    n_slices=40,
    eddy_factor=1.0,           # 1 = include winding eddy loss estimate, 0 = ignore, 2 = pessimistic
)

LOSS = dict(n_ref_rpm=1000.0, P_bear_ref_W=4.0, P_wind_ref_W=2.0)   # replace with real estimates

LIM = dict(
    kfill_max=0.50,            # bare-copper fraction of the coil side you can really wind
    J_cont_A_mm2=6.0,          # continuous current density at the rated point
    J_peak_A_mm2=18.0,         # launch current density (seconds only)
    I_phase_max_A_rms=30.0,    # inverter limit (VESC "motor current max" amplitude / sqrt(2))
    eta_min=0.85,              # minimum efficiency at the rated point
    w_mag_min_mm=8.0,          # smallest magnet width you can source/glue (at the inner radius)
    w_side_min_mm=3.0,
    speed_margin=1.25,         # no-load speed / rated speed
    capability_ratio_max=2.5,  # unconstrained motor power capability / 500 W (rule-risk guard)
    mass_max_kg=6.0,           # motor + reduction
)

WEIGHTS = dict(
    time=1.0,                  # per second of 0 -> v_target time
    mass_s_per_kg=0.10,        # "I would trade 1 kg for this many seconds" (beyond the physics in the sim)
    eff_s_per_unit=2.0,        # seconds per 1.00 of efficiency (0.02 = 2 points of efficiency)
    penalty=50.0,              # per unit relative constraint violation
)

POLES = [12, 16, 20, 24, 28, 32, 36, 40, 44, 48]    # multiples of 4 work for both winding types
AWG = {16: 1.291, 17: 1.150, 18: 1.024, 19: 0.912, 20: 0.812, 21: 0.723, 22: 0.644,
       23: 0.573, 24: 0.511, 25: 0.455, 26: 0.405, 27: 0.361, 28: 0.321, 29: 0.286, 30: 0.255}
AWG_KEYS = sorted(AWG)

BOUNDS = [
    (110.0, 220.0),                  # 0 outer diameter [mm]  <- your packaging limit
    (0.55, 0.80),                    # 1 inner/outer ratio
    (0, len(POLES) - 1),             # 2 pole-count index
    (4, 80),                         # 3 turns per coil
    (0, len(AWG_KEYS) - 1),          # 4 AWG index (strand size)
    (1, 12),                         # 5 strands in hand
    (3.0, 12.0),                     # 6 coil thickness [mm]
    (2.0, 10.0),                     # 7 magnet thickness per rotor [mm]
    (DRIVE["R_mc_min"] * DRIVE["design_cadence_rpm"],
     DRIVE["R_mc_max"] * DRIVE["design_cadence_rpm"]),   # 8 rated motor speed [rpm]
]

# Evaluate your own design instead of optimizing: fill this dict (keys as below), else leave None
MANUAL_DESIGN = None
# MANUAL_DESIGN = dict(D_out_mm=200, ratio=0.7, poles=24, n_turn=19, d_wire_mm=0.912,
#                      n_strand=4, h_coil_mm=7.0, h_pm_mm=5.0, n_rated_rpm=600)


# =====================================================================
# 2) MOTOR MODEL
# =====================================================================
def vph_max():
    """Max phase RMS voltage the inverter can apply (ideal SVPWM: line-line RMS = Vdc/sqrt2)."""
    return RULES["vdc_util"] * RULES["Vdc"] / math.sqrt(6.0)


def build_motor(g, mat=None, field=None, L_override=None, T_run_C=None):
    M = MAT if mat is None else mat
    T_run = THERMAL["T_cu_run_C"] if T_run_C is None else T_run_C
    ro = 0.5 * g["D_out_mm"] * 1e-3
    ri = g["ratio"] * ro
    lact = ro - ri
    rmean = 0.5 * (ri + ro)
    hw = g["h_coil_mm"] * 1e-3
    hpm = g["h_pm_mm"] * 1e-3
    gs = M["g_air_mm"] * 1e-3
    lgap = hw + 2 * gs                              # total magnet-to-magnet nonmagnetic gap
    P = int(g["poles"])
    overlap = (M["winding"] == "overlap")
    Q = (3 * P) // 2 if overlap else (3 * P) // 4   # total coils
    q = Q // 3                                      # coils per phase
    a = int(M["a_paths"])
    Nt = int(g["n_turn"])
    Nser = Nt * q / a
    theta_m = math.pi if overlap else math.pi * P / Q
    wf = M["width_fill"]
    kappa = wf * ri / (ri + ro)
    th_ov = wf * (ri - lgap) * math.pi / (3 * rmean)
    th_side = th_ov if overlap else kappa * theta_m
    w_side = 2 * rmean * th_side / P                # coil-side circumferential width
    if overlap:
        l_end = 4 * math.pi * rmean / P + 4 * lgap
    else:
        l_end = (2 * theta_m / P) * (ro + ri) * (1 - 0.6 * kappa)
    l_turn = 2 * lact + l_end
    d = g["d_wire_mm"] * 1e-3
    ns = int(g["n_strand"])
    Acu = ns * math.pi * d * d / 4
    l_ph = q * Nt * l_turn
    R20 = RHO20 * l_ph / (a * a * Acu)
    R_run = R20 * (1 + ALPHA_CU * (T_run - 20.0))
    kfill = Nt * Acu / (hw * w_side)
    m_cu = 3 * RHO_CU * l_ph * Acu

    # ---- magnet field (ideal 1-D circuit, then derated) ----
    Bpl = M["Brem_T"] * 2 * hpm / (2 * hpm + M["mu_rec"] * lgap)
    Best = 4 / math.pi * Bpl * math.sin(math.pi * M["alpha_p"] / 2)
    if field is None:
        Bi = Bo = Best * M["field_derate"]
    else:
        Bi, Bo = field

    # ---- radial-slice flux integration (website method) ----
    n = int(M["n_slices"])
    dr = lact / n
    rs = ri + (np.arange(n) + 0.5) * dr
    Bs = Bi + (Bo - Bi) * (rs - ri) / lact
    th = P * w_side / (2 * rs)
    kw = np.sinc(th / (2 * np.pi))                  # sin(th/2)/(th/2)
    if not overlap:
        kw = kw * np.sin((theta_m - th) / 2)
    Ftot = float(np.sum(Bs * kw * rs) * dr)
    lam = 4.0 * Nser * Ftot / P                     # peak fundamental flux linkage per phase [Wb]
    Kt = 3 / math.sqrt(2) * (P / 2) * lam           # N m per A_rms
    ke = (P / 2) * lam / math.sqrt(2)               # V_phase_rms per (rad/s mech)

    # ---- inductance estimate (coreless: small, barely matters at these speeds) ----
    g_eff = lgap + 2 * hpm / M["mu_rec"]
    A_coil = (2 * math.pi / Q) * rmean * lact
    L = L_override if L_override is not None else (
        M["kL"] * q * MU0 * Nt ** 2 * A_coil / g_eff / (a * a))

    # ---- winding eddy loss: P = K_e * w^2 (round strands in a transverse field) ----
    Bpk = 0.5 * (Bi + Bo)
    rho_T = RHO20 * (1 + ALPHA_CU * (T_run - 20.0))
    fe_per_w = (P / 2) / (2 * math.pi)
    V_cu = m_cu / RHO_CU
    act_frac = 2 * lact / l_turn
    K_e = M["eddy_factor"] * math.pi ** 2 * fe_per_w ** 2 * Bpk ** 2 * d ** 2 / (16 * rho_T) * V_cu * act_frac

    # ---- masses ----
    A_ann = math.pi * (ro ** 2 - ri ** 2)
    m_mag = M["rho_mag"] * 2 * M["alpha_p"] * A_ann * hpm
    phi_pole = Bpl * M["alpha_p"] * A_ann / P
    t_by = max(M["t_backiron_min_mm"] * 1e-3, 1.1 * phi_pole / (2 * M["B_backiron_T"] * lact))
    m_by = M["rho_steel"] * 2 * A_ann * t_by
    m_epoxy = max(A_ann * hw - V_cu, 0.0) * M["rho_epoxy"]
    m_stator = m_cu + m_epoxy
    m_active = m_mag + m_by + m_stator
    m_struct = M["struct_fixed_kg"] + M["struct_frac"] * m_active
    m_motor = m_active + m_struct
    m_rot = m_mag + m_by + 0.5 * m_struct
    J_rot = m_rot * 0.5 * (ro ** 2 + ri ** 2)

    I_peak = min(LIM["I_phase_max_A_rms"], LIM["J_peak_A_mm2"] * a * Acu * 1e6)
    w_noload = vph_max() / ke
    return dict(
        g=g, ro=ro, ri=ri, poles=P, Q=Q, q=q, a=a, Nt=Nt, Nser=Nser, d=d, ns=ns,
        Acu_mm2=Acu * 1e6, l_turn=l_turn, w_side_mm=w_side * 1e3, kfill=kfill,
        m_cu=m_cu, R20=R20, R_run=R_run, L=L, lam=lam, Kt=Kt, ke=ke, Bi=Bi, Bo=Bo, Bpl=Bpl,
        K_e=K_e, hw=hw, hpm=hpm, t_by=t_by, m_mag=m_mag, m_by=m_by, m_epoxy=m_epoxy,
        m_stator=m_stator, m_struct=m_struct, m_motor=m_motor, J_rot=J_rot,
        F_ax=Bpl ** 2 / (2 * MU0) * M["alpha_p"] * A_ann,
        w_mag_min_mm=M["alpha_p"] * 2 * math.pi * ri / P * 1e3,
        w_mag_max_mm=M["alpha_p"] * 2 * math.pi * ro / P * 1e3,
        A_stator=2 * math.pi * (ro ** 2 - ri ** 2) + 2 * math.pi * (ro + ri) * hw,
        I_peak=I_peak, w_noload=w_noload, n_noload_rpm=w_noload * 60 / (2 * math.pi),
        Ftot=Ftot,
    )


def loss_torque(m, w):
    """Torque [N m] eaten by bearing + windage + winding eddy loss at mech speed w [rad/s]."""
    r = (w * 60 / (2 * math.pi)) / LOSS["n_ref_rpm"]
    p_mech = LOSS["P_bear_ref_W"] * r + LOSS["P_wind_ref_W"] * r ** 3 + m["K_e"] * w ** 2
    return p_mech / np.maximum(w, 1e-6)


def envelope(m, w):
    """Max shaft torque vs speed: limited by inverter/peak current, 48 V, and the 500 W OUTPUT cap."""
    Vph = vph_max()
    R, L, P = m["R_run"], m["L"], m["poles"]
    E = m["ke"] * w
    X = (P / 2) * w * L
    A = R * R + X * X
    disc = (E * R) ** 2 - A * (E * E - Vph ** 2)
    I_volt = np.where(E < Vph, (-E * R + np.sqrt(np.maximum(disc, 0.0))) / A, 0.0)
    Tl = loss_torque(m, w)
    I_cap = (RULES["P_cap_W"] / w + Tl) / m["Kt"]
    I = np.minimum(np.minimum(I_volt, I_cap), m["I_peak"])
    Tsh = np.maximum(m["Kt"] * I - Tl, 0.0)
    Pcu = 3 * I * I * R
    Pem = m["Kt"] * I * w
    return dict(I=I, I_volt=I_volt, T=Tsh, P=Tsh * w, Pcu=Pcu, Pin=Pem + Pcu)


def rated_point(m, n_rpm):
    """Continuous check: deliver the full 500 W at shaft at n_rpm."""
    w = n_rpm * 2 * math.pi / 60
    Tl = float(loss_torque(m, np.array([w]))[0])
    I = (RULES["P_cap_W"] / w + Tl) / m["Kt"]
    E = m["ke"] * w
    X = (m["poles"] / 2) * w * m["L"]
    R = m["R_run"]
    V_line = math.sqrt(3) * math.hypot(E + I * R, X * I)
    Pcu = 3 * I * I * R
    P_mech_loss = Tl * w
    P_eddy = m["K_e"] * w * w
    eta = RULES["P_cap_W"] / (RULES["P_cap_W"] + Pcu + P_mech_loss)
    return dict(I=I, V_line=V_line, V_line_max=RULES["vdc_util"] * RULES["Vdc"] / math.sqrt(2),
                Pcu=Pcu, P_mech_loss=P_mech_loss, P_eddy=P_eddy, stator_heat=Pcu + P_eddy,
                eta=eta, J=I / (m["a"] * m["Acu_mm2"]), T_shaft=RULES["P_cap_W"] / w)


def heat_budget(m):
    if THERMAL["stator_heat_budget_W"] is not None:
        return THERMAL["stator_heat_budget_W"]
    return THERMAL["h_eff"] * m["A_stator"] * (THERMAL["T_cu_max_C"] - THERMAL["T_amb_C"])


# =====================================================================
# 3) VEHICLE ACCELERATION (quasi-static, best gear at every speed)
# =====================================================================
def accel_run(m, R_mc, wg, env, m_red):
    V, D = VEHICLE, DRIVE
    rw = V["wheel_dia_m"] / 2.0
    vt = V["v_target_mph"] * MPH
    cass = np.array(D["cassette_T"], dtype=float)
    Gr = D["chainring_T"] / cass              # wheel rev per chainring rev
    k = R_mc / Gr                             # motor rev per wheel rev
    eta_d = D["chain_eff"] * D["red_eff"]
    m_weight = V["m_base_kg"] + m["m_motor"] + m_red
    m_eff0 = m_weight + V["wheel_inertia_kg"]
    edges = np.linspace(0.0, vt, 91)
    t, t10, E_cu, I_dc_max = 0.0, None, 0.0, 0.0
    tt, vv, gg, a0 = [0.0], [0.0], [], None
    for i in range(90):
        v = 0.5 * (edges[i] + edges[i + 1])
        dv = edges[i + 1] - edges[i]
        ww = v / rw
        wm = k * ww
        cad = ww / Gr * 60.0 / (2 * math.pi)
        ok = (wm < 0.98 * m["w_noload"]) & (cad <= D["max_chainring_rpm"])
        if not ok.any():
            return None
        T = np.interp(wm, wg, env["T"])
        Fw = np.minimum(T * k * eta_d / rw, V["traction_limit_N"])
        Fw = np.where(ok, Fw, -1.0)
        j = int(np.argmax(Fw))
        F_r = min(V["rider_power_W"] / max(v, 0.1), V["rider_force_max_N"])
        F_res = V["crr"] * m_weight * GRAV + 0.5 * V["rho_air"] * V["CdA"] * v * v
        m_eff = m_eff0 + m["J_rot"] * k[j] ** 2 / rw ** 2
        acc = (Fw[j] + F_r - F_res) / m_eff
        if acc < 0.02:
            return None
        if a0 is None:
            a0 = acc
        dt = dv / acc
        t += dt
        if t10 is None and edges[i + 1] >= 10 * MPH:
            t10 = t - dt * (edges[i + 1] - 10 * MPH) / dv
        I_here = float(np.interp(wm[j], wg, env["I"]))
        E_cu += 3 * I_here ** 2 * m["R_run"] * dt
        P_in = float(np.interp(wm[j], wg, env["Pin"])) / RULES["eta_inv"]
        I_dc_max = max(I_dc_max, P_in / (RULES["Vdc"] * RULES["batt_sag"]))
        tt.append(t)
        vv.append(edges[i + 1])
        gg.append(int(cass[j]))
    C_eff = THERMAL["thermal_mass_fraction"] * (m["m_cu"] * THERMAL["c_cu"] + m["m_epoxy"] * THERMAL["c_epoxy"])
    T_end = THERMAL["T_start_C"] + VEHICLE["n_launches"] * E_cu / C_eff
    return dict(t_target=t, t10=t10, a0=a0, T_end_C=T_end, I_dc_max_A=I_dc_max,
                trace=(np.array(tt), np.array(vv), np.array(gg)))


# =====================================================================
# 4) EVALUATION + OPTIMIZER
# =====================================================================
def _hi(v, lim):
    return max(0.0, (v - lim) / lim)


def _lo(v, lim):
    return max(0.0, (lim - v) / lim)


def evaluate(g):
    m = build_motor(g)
    n_r = g["n_rated_rpm"]
    R_mc = n_r / DRIVE["design_cadence_rpm"]
    m_red = DRIVE["red_mass_fixed_kg"] + DRIVE["red_mass_per_ratio_kg"] * R_mc
    rp = rated_point(m, n_r)
    wg = np.linspace(0.5, m["w_noload"], 300)
    env = envelope(m, wg)
    Tl = loss_torque(m, wg)
    P_free = float(np.max(np.maximum(m["Kt"] * np.minimum(env["I_volt"], m["I_peak"]) - Tl, 0.0) * wg))
    run = accel_run(m, R_mc, wg, env, m_red)
    budget = heat_budget(m)
    viol = dict(
        fill=_hi(m["kfill"], LIM["kfill_max"]),
        J_cont=_hi(rp["J"], LIM["J_cont_A_mm2"]),
        V_rated=_hi(rp["V_line"], rp["V_line_max"]),
        heat=_hi(rp["stator_heat"], budget),
        eta=_lo(rp["eta"], LIM["eta_min"]),
        magnet_width=_lo(m["w_mag_min_mm"], LIM["w_mag_min_mm"]),
        coil_width=_lo(m["w_side_mm"], LIM["w_side_min_mm"]),
        speed_margin=_lo(m["n_noload_rpm"], LIM["speed_margin"] * n_r),
        capability=_hi(P_free / RULES["P_cap_W"], LIM["capability_ratio_max"]),
        mass=_hi(m["m_motor"] + m_red, LIM["mass_max_kg"]),
    )
    if run is None:
        viol["cant_reach_target"] = 5.0
        J = 1000.0
    else:
        viol["launch_temp"] = _hi(run["T_end_C"], THERMAL["T_cu_max_C"])
        viol["battery_current"] = _hi(run["I_dc_max_A"], RULES["I_batt_max_A"])
        J = (WEIGHTS["time"] * run["t_target"]
             + WEIGHTS["mass_s_per_kg"] * (m["m_motor"] + m_red)
             + WEIGHTS["eff_s_per_unit"] * (1.0 - rp["eta"]))
    pen = WEIGHTS["penalty"] * sum(viol.values())
    return dict(J=J + pen, viol=viol, m=m, rp=rp, run=run, env=env, wg=wg, P_free=P_free,
                R_mc=R_mc, m_red=m_red, budget=budget)


def decode(x):
    return dict(
        D_out_mm=float(x[0]), ratio=float(x[1]),
        poles=POLES[int(round(x[2]))], n_turn=int(round(x[3])),
        d_wire_mm=AWG[AWG_KEYS[int(round(x[4]))]], n_strand=int(round(x[5])),
        h_coil_mm=float(x[6]), h_pm_mm=float(x[7]), n_rated_rpm=float(x[8]),
    )


def objective(x):
    try:
        return evaluate(decode(x))["J"]
    except Exception:
        return 1e6


def optimize(seed=1, popsize=24, maxiter=400):
    res = differential_evolution(objective, BOUNDS, seed=seed, popsize=popsize, maxiter=maxiter,
                                 tol=1e-7, mutation=(0.5, 1.0), recombination=0.8, polish=False,
                                 updating="deferred", workers=-1, disp=True)
    return decode(res.x)


# =====================================================================
# 5) REPORT / PLOT / SELF-TEST
# =====================================================================
def report(g, title="DESIGN"):
    r = evaluate(g)
    m, rp, run, env, wg = r["m"], r["rp"], r["run"], r["env"], r["wg"]
    idx = np.where(env["P"] >= 0.99 * RULES["P_cap_W"])[0]
    n_corner = wg[idx[0]] * 60 / (2 * math.pi) if len(idx) else float("nan")
    print("=" * 72)
    print(title)
    print("=" * 72)
    print("BUILD SHEET")
    print(f"  Outer / inner diameter : {g['D_out_mm']:.1f} mm / {2 * m['ri'] * 1e3:.1f} mm  (ratio {g['ratio']:.3f})")
    print(f"  Poles / coils / per ph : {m['poles']} / {m['Q']} / {m['q']}   ({MAT['winding']})")
    print(f"  Magnets                : {m['poles']} per rotor, {m['w_mag_min_mm']:.1f}-{m['w_mag_max_mm']:.1f} mm wide "
          f"x {(m['ro'] - m['ri']) * 1e3:.1f} mm radial x {g['h_pm_mm']:.1f} mm thick, Br {MAT['Brem_T']} T")
    print(f"  Back-iron (each rotor) : {m['t_by'] * 1e3:.1f} mm mild steel")
    print(f"  Coil                   : {m['Nt']} turns, {m['ns']} x {g['d_wire_mm']:.3f} mm strands "
          f"(AWG {AWG_KEYS[[AWG[k_] for k_ in AWG_KEYS].index(g['d_wire_mm'])]}), "
          f"{m['w_side_mm']:.1f} mm side width, {g['h_coil_mm']:.1f} mm thick, fill {m['kfill'] * 100:.0f} %")
    print(f"  Air gap per side       : {MAT['g_air_mm']} mm   (mag-to-mag {(m['hw'] + 2 * MAT['g_air_mm'] * 1e-3) * 1e3:.1f} mm)")
    print(f"  Rotor-to-rotor pull    : ~{m['F_ax']:.0f} N  (design the assembly jig for this!)")
    print("ELECTRICAL (wye, FOC)")
    print(f"  Fundamental field used : {m['Bi']:.3f} T peak (derate {MAT['field_derate']})")
    print(f"  Kt {m['Kt']:.3f} N m/A_rms   ke {m['ke']:.4f} V_ph,rms/(rad/s)   R_hot {m['R_run'] * 1e3:.1f} mOhm   L {m['L'] * 1e3:.3f} mH")
    print(f"  No-load speed on {RULES['Vdc']:.0f} V : {m['n_noload_rpm']:.0f} rpm")
    print("MASS")
    print(f"  magnets {m['m_mag']:.2f} | back-iron {m['m_by']:.2f} | copper {m['m_cu']:.2f} | potting {m['m_epoxy']:.2f} | "
          f"structure {m['m_struct']:.2f} -> MOTOR {m['m_motor']:.2f} kg  (+ reduction {r['m_red']:.2f} kg)")
    print(f"RATED POINT ({RULES['P_cap_W']:.0f} W out @ {g['n_rated_rpm']:.0f} rpm, {rp['T_shaft']:.2f} N m)")
    print(f"  I {rp['I']:.1f} A_rms | J {rp['J']:.1f} A/mm2 | line V {rp['V_line']:.1f} of {rp['V_line_max']:.1f} V | "
          f"eta {rp['eta'] * 100:.1f} %")
    print(f"  copper loss {rp['Pcu']:.0f} W + winding eddy {rp['P_eddy']:.1f} W = stator heat {rp['stator_heat']:.0f} W "
          f"(budget ~{r['budget']:.0f} W)")
    print("TORQUE-SPEED ENVELOPE (500 W OUTPUT cap applied)")
    print(f"  stall shaft torque {env['T'][0]:.1f} N m @ {env['I'][0]:.0f} A_rms | hits 500 W at {n_corner:.0f} rpm")
    print(f"  unconstrained capability {r['P_free']:.0f} W ({r['P_free'] / RULES['P_cap_W']:.2f}x the cap) "
          f"-> your controller current limits are what keep it legal")
    print("VEHICLE")
    print(f"  motor->chainring reduction R_mc = {r['R_mc']:.2f}:1  (motor at {g['n_rated_rpm']:.0f} rpm when pedals at {DRIVE['design_cadence_rpm']:.0f} rpm)")
    if run is None:
        print("  !! cannot reach the target speed with this design/gearing")
    else:
        t10 = f"{run['t10']:.1f} s" if run["t10"] else "n/a"
        print(f"  launch accel {run['a0']:.2f} m/s2 | 0-10 mph {t10} | 0-{VEHICLE['v_target_mph']:.0f} mph {run['t_target']:.1f} s")
        print(f"  winding temp after launch {run['T_end_C']:.0f} C | peak battery current {run['I_dc_max_A']:.1f} A")
    print("CONTROLLER SETTINGS (starting point)")
    print(f"  phase current limit {math.sqrt(2) * env['I'][0]:.0f} A amplitude (VESC 'motor current max'), "
          f"battery current limit ~{(run['I_dc_max_A'] if run else 0):.0f} A")
    bad = {k_: v for k_, v in r["viol"].items() if v > 1e-3}
    print("CONSTRAINTS:", "all satisfied" if not bad else "VIOLATED -> " + ", ".join(f"{k_} ({v * 100:.0f} % over)" for k_, v in bad.items()))
    print("  (J score incl. penalties: %.2f)" % r["J"])
    return r


def plot(g, fname="afpm_design.png"):
    r = evaluate(g)
    wg, env, run = r["wg"], r["env"], r["run"]
    n = wg * 60 / (2 * math.pi)
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
    ax[0].plot(n, env["T"], label="shaft torque [N m]")
    ax[0].set_xlabel("motor speed [rpm]")
    ax[0].set_ylabel("N m")
    ax[0].grid(alpha=0.3)
    ax0b = ax[0].twinx()
    ax0b.plot(n, env["P"], "r--", label="shaft power [W]")
    ax0b.set_ylabel("W")
    ax[0].set_title("Torque-speed envelope (48 V, 500 W out cap)")
    if run is not None:
        tt, vv, gg = run["trace"]
        ax[1].plot(tt, vv / MPH)
        ax[1].set_xlabel("time [s]")
        ax[1].set_ylabel("mph")
        ax[1].grid(alpha=0.3)
        ax[1].set_title(f"0-{VEHICLE['v_target_mph']:.0f} mph in {run['t_target']:.1f} s")
    fig.tight_layout()
    fig.savefig(fname, dpi=150)
    print("saved", fname)
    plt.show()


def self_test():
    """Reproduce the WiredWhite numbers. If these fail, stop and fix before trusting anything."""
    mat = dict(MAT, g_air_mm=1.8, winding="concentrated", a_paths=1, width_fill=1.0,
               n_slices=40, eddy_factor=0.0)
    g = dict(D_out_mm=400, ratio=0.7, poles=24, n_turn=25, d_wire_mm=0.85, n_strand=6,
             h_coil_mm=8.4, h_pm_mm=6.0)
    m = build_motor(g, mat=mat, field=(0.527, 0.527), L_override=0.3e-3, T_run_C=80.0)
    E = m["ke"] * 300 * 2 * math.pi / 60
    m20 = build_motor(g, mat=mat, field=(0.527, 0.527), L_override=0.3e-3, T_run_C=20.0)
    T_kamper = m20["Kt"] * math.sqrt(120.0 / (3 * m20["R_run"]))
    checks = [
        ("flux linkage [Wb]", m["lam"], 0.1111),
        ("phase EMF @300 rpm [V]", E, 29.628),
        ("Kt [N m/A]", m["Kt"], 2.829),
        ("R hot @80C [ohm]", m["R_run"], 0.1953),
        ("copper mass [kg]", m["m_cu"], 2.874),
        ("copper fill", m["kfill"], 0.4147),
        ("Kamper conc. torque @120 W Cu, 20C [N m]", T_kamper, 44.872),
    ]
    ok = True
    print("SELF-TEST vs wiredwhite.com example")
    for name, got, exp in checks:
        good = abs(got - exp) / exp < 0.005
        ok &= good
        print(f"  {'OK ' if good else 'BAD'} {name:44s} model {got:9.4f}   site {exp:9.4f}")
    if not ok:
        print("  !!! self-test failed - do not trust the optimizer output until this is fixed")
    return ok


if __name__ == "__main__":
    self_test()
    if MANUAL_DESIGN:
        report(MANUAL_DESIGN, "MANUAL DESIGN")
        plot(MANUAL_DESIGN)
    else:
        best = optimize()
        report(best, "OPTIMIZED DESIGN")
        plot(best)
