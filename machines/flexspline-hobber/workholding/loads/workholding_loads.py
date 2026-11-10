#!/usr/bin/env python3
"""Workholding loads, pressure window, accuracy budget and automation sizing for the
flexspline expanding mandrel.  Run: python3 workholding_loads.py
Pure stdlib; prints every number quoted in README.md.  Inputs are at the top."""
import math

# ---- inputs: model + reference-force evidence ---------------------------------------
m, z, b = 0.3, 200, 8.0            # GearSpec(0.3, 200, 8) in FlexsplineHobber.hx
d = m * z                          # 60 mm reference diameter
Fc, Fr = 46.678736258836665, 16.989670571379055   # N, build/.../flexspline-reference-force-fast/loads.json
T_cut = 1.4902342072175818         # N m, maxTableTorqueBound = hypot(Fc,Fr)*30 mm
Fa_cut = 0.3 * Fc                  # N, ASSUMPTION (model has no axial force; hob lead angle ~0.55 deg, feed 0.5 mm/rev)
SF = 2.0
D = 58.05                          # bore, H5 (+0.013/0)
IT5 = 0.013
L = max(16.0, b + 8.0)             # clamped length from seat
a = D / 2
t_cup = 0.6                        # wall under tooth root (bodyOutside 59.25)
E, nu = 210e3, 0.3                 # MPa
sig_y = 1000.0                     # MPa, 40CrNiMoA 38-42 HRC, conservative
A = math.pi * D * L
print(f"d={d:.1f} mm  b={b} mm  clamp length L={L} mm  bore area A={A:.0f} mm2")

# ---- 1. holding torque and axial force ------------------------------------------------
T_req, Fa_req = SF * T_cut, SF * Fa_cut
print(f"\n[1] Fc={Fc:.1f} N  Fr={Fr:.1f} N  Fa(assumed 0.3 Fc)={Fa_cut:.1f} N  T_bound={T_cut:.2f} N m")
print(f"    required torque {T_req:.2f} N m, axial pull-off {Fa_req:.1f} N (SF {SF})")
def p_hold(mu, T=T_req, Fa=Fa_req):
    return math.hypot(T / (a / 1000), Fa) / (mu * A)     # MPa
for mu in (0.08, 0.10, 0.15):
    print(f"    mu={mu:.2f}: p_hold={p_hold(mu):.2f} MPa; if tooth peaks are 2x window means: {p_hold(mu,2*T_req,2*Fa_req):.2f} MPa; "
          f"torque capacity {mu*A*a/1000:.2f} N m/MPa")
p_min = p_hold(0.08)

# ---- 2. cup compliance and stress -----------------------------------------------------
bo = a + t_cup
lame_u = lambda p: p * a / E * ((bo**2 + a**2) / (bo**2 - a**2) + nu) * 1000      # um radial
lame_sig = lambda p: p * (bo**2 + a**2) / (bo**2 - a**2)                          # inner hoop, MPa
k_c = lame_u(1.0)                                                                  # um/MPa
print(f"\n[2] cup (t={t_cup} mm): radial expansion {k_c:.2f} um/MPa (Lame; thin-ring {((a+t_cup/2)**2/(E*t_cup)*1000):.2f}); "
      f"inner hoop {lame_sig(1):.1f} MPa/MPa")
for p in (p_min, 1.0, 2.0, 10.0):
    print(f"    p={p:5.2f} MPa: dr={lame_u(p):6.2f} um (dD={2*lame_u(p):6.2f}), hoop={lame_sig(p):6.1f} MPa, yield SF={sig_y/lame_sig(p):5.1f}")
p_yield = sig_y / 2 / lame_sig(1)
print(f"    p for hoop = yield/2: {p_yield:.1f} MPa (dr={lame_u(p_yield):.0f} um)")
print(f"    model limit 2 um radial would be p={2/k_c:.2f} MPa < p_hold(mu=0.08)={p_min:.2f} MPa: the limit contradicts holding")
# release strain (uniform expansion is a scale error of the toothed ring)
for p in (p_min, 2.0):
    eps = lame_u(p) / 1000 / a
    print(f"    p={p:.2f}: strain {eps:.2e} -> tooth thickness change {eps*0.47*1000:.3f} um, circular pitch {eps*math.pi*m*1000:.3f} um, "
          f"pitch dia {eps*d*1000:.2f} um")

# sleeve + gap: series model.  sleeve of wall ts, mean radius rs, oil P inside, contact p outside
def sleeve_k(ts, rs=a - 0.0125): return rs**2 / (E * ts) * 1000                    # um/MPa, free thin tube
def contact(P, g, ts):                                                             # g = radial clearance, um
    ks = sleeve_k(ts); return max(0.0, (P * ks - g) / (ks + k_c))
def oil_for(p, g, ts):
    ks = sleeve_k(ts); return (g + p * (ks + k_c)) / ks
print("\n    sleeve+gap model: p = (P*ks - g)/(ks+kc);  P = (g + p(ks+kc))/ks")
for ts in (1.0, 1.5, 2.0):
    ks = sleeve_k(ts)
    row = [f"ts={ts}: ks={ks:.2f} um/MPa, dp/dP={ks/(ks+k_c):.2f}"]
    for cd in (0.010, 0.028, 0.050):    # diametral collapsed clearance, mm
        g = cd / 2 * 1000
        row.append(f"clr {cd*1000:.0f}um: P(p=1)={oil_for(1.0,g,ts):.1f} P(p={p_min:.2f})={oil_for(p_min,g,ts):.1f} MPa")
    print("    " + " | ".join(row))
# window with fixed regulated P, clearance 10..28 um diametral (OD -0.005 tol, H5 bore), ts 1.5
ts = 1.5; g_lo, g_hi = 5.0, 14.0
P_set = oil_for(1.0, (g_lo + g_hi) / 2, ts)
print(f"    fixed P={P_set:.1f} MPa, ts=1.5: p from {contact(P_set,g_hi,ts):.2f} (loosest) to {contact(P_set,g_lo,ts):.2f} MPa (tightest); "
      f"cup expansion {lame_u(contact(P_set,g_hi,ts)):.1f}..{lame_u(contact(P_set,g_lo,ts)):.1f} um radial")
P_model = 10.0
print(f"    model default: clearance 50 um dia, ts 1.5, P=10 MPa -> p={contact(P_model,25.0,1.5):.2f} MPa (< p_hold {p_min:.2f}: would NOT hold); "
      f"P=15 MPa -> p={contact(15.0,25.0,1.5):.2f} MPa; dry-fire 15 MPa sleeve expansion {15*sleeve_k(1.5):.0f} um radial, hoop {15*(a)/1.5:.0f} MPa")

# ---- 3. ISO 1328-1:2013 tolerances (formulae, mn=0.3 is below the 0.5 table range) ----
def iso(Q):
    f = 2 ** ((Q - 5) / 2)
    sm, sd, sb = math.sqrt(m), math.sqrt(d), math.sqrt(b)
    fp = (0.001 * d + 0.4 * m + 5) * f
    Fp = (0.002 * d + 0.55 * sd + 7) * f
    Fa = (3.2 * sm + 0.22 * sd + 0.7) * f
    Fb = (0.1 * sd + 0.63 * sb + 4.2) * f
    Fr_ = 0.9 * Fp
    return dict(fp=fp, Fp=Fp, Fr=Fr_, Falpha=Fa, Fbeta=Fb)
print("\n[3] ISO 1328-1:2013 (um), d=60, mn=0.3, b=8, unrounded")
tol = {Q: iso(Q) for Q in (5, 6)}
for Q in (5, 6):
    print(f"    class {Q}: " + "  ".join(f"{k}={v:.1f}" for k, v in tol[Q].items()))
# workholding allocation: each item gets a share of the tolerance, combined in RSS <= 50% of tolerance
SHARE = 0.5
print(f"    workholding budget = {SHARE:.0%} of tolerance (25% of the variance), RSS of its parts")
for Q in (5, 6):
    t = tol[Q]
    runout_b = SHARE * t["Fr"]
    parts = {"mandrel static TIR": 3.0 if Q == 5 else 4.0, "re-clamp centring repeat TIR": 1.5 if Q == 5 else 2.0,
             "clamp lobing 2a (bore roundness 3 um conforms)": 3.0}
    rss = math.sqrt(sum(v * v for v in parts.values()))
    print(f"    class {Q}: Fr budget {runout_b:.1f} um; parts {parts} -> RSS {rss:.1f} um")
    tilt_um = 2.0 if Q == 5 else 3.0       # axis misalignment effect over b
    print(f"       Fbeta budget {SHARE*t['Fbeta']:.1f} um: tilt of cup axis vs table axis over b: need <= {tilt_um} um/{b:.0f} mm "
          f"= {tilt_um/b/1000*1e3:.2f} mrad ({tilt_um/b*L:.1f} um over L={L:.0f})")
    lobe = 1.5
    print(f"       fp budget {SHARE*t['fp']:.1f} um, Fp budget {SHARE*t['Fp']:.1f} um: lobing tangential ~ a/n = {lobe/2:.2f} um (n=2) -> ok")
# cocking on loading before clamp
print(f"    cocking before clamp: clearance 10..28 um / L=16 -> up to {0.028/L*1e3:.2f} mrad = {0.028/L*b*1000:.1f} um over b; "
      f"clamp must pull axis true (seat land + cylinder both locate).")

# ---- 4. automation sizing -------------------------------------------------------------
print("\n[4] automation")
p_air, eta = 0.5, 0.9     # MPa gauge usable after rotary union, booster efficiency
P_need = oil_for(1.0, 9.5, 1.5)
for P in (P_need, 10.0, 15.0):
    print(f"    oil {P:.1f} MPa needs area ratio {P/(p_air*eta):.1f}:1 at 5 bar (6 bar supply less union drop), "
          f"{P/(0.6*eta):.1f}:1 at 6 bar")
dV = 2 * math.pi * a * L * (25.0 / 1000) + (0.5 * A) * (10.0 / 1500)
print(f"    oil volume to expand sleeve 25 um + compress 0.5 mm cavity at 10 MPa: {dV:.0f} mm3 -> booster stroke tiny (piston 20 mm dia: {dV/(math.pi*100):.2f} mm on oil side)")
print(f"    trapped-oil thermal: ~0.036 MPa/K of sleeve pressure (0.5 mm cavity 1457 mm3 * 7e-4/K / (26.8 mm3/MPa))")
N = lambda p: p * math.pi * D * L
for p in (1.0, 2.0):
    for alpha in (3.0, 7.0):
        rho = math.degrees(math.atan(0.1))
        Fd = N(p) * math.tan(math.radians(alpha + rho))
        print(f"    drawbar cone p={p:.1f} MPa N={N(p):.0f} N alpha={alpha} deg mu_cone=0.1: pull {Fd:.0f} N "
              f"(air cylinder dia for 5 bar, eta .9: {math.sqrt(4*Fd/(math.pi*0.5*0.9)):.0f} mm)")
# robot lead-in
rob = 0.1
print(f"    robot lead-in: robot+gripper position error {rob} mm + bore/clearance {0.028/2:.3f} mm radial -> nose lead-in radial >= 0.5 mm, "
      f"15 deg half-angle x 2 mm axial; cup seat must stay a 0.3 mm land")
