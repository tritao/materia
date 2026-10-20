# Prepared weld reaction qualification

The fatigue duty is **100 million full-load reversals**, approved on
2026-10-20. Do not substitute the earlier provisional ten-million-cycle duty.
The load envelope includes 3 m/s² acceleration reversals and concurrent
300 N cutting allowances in X, Y and Z. Gravity is included in strength, not
in the alternating stress range. Strength uses G=1.35 and Q=1.5.

## Reaction model and limits

`fem/weld_model.py` replaces the merged frame-centreline joints with 68 separate
manufactured stock pieces. Only selected physical weld runs join those pieces.
Each run contributes its actual position, length and three-dimensional line
inertia. Six positive spring quadrature points reproduce each contact's rigid
joint stiffness; offset kinematics use exact work-conjugate MPC equations.
There is no assumed arbitrary rotational weld spring and no proximity tie
bypassing the prepared connections. Beam-foot seams replace the old lumped
foot-weld springs too.

Point stiffness uses G a L / leg, with steel E=210 GPa, Poisson ratio 0.3 and
throat a=leg/√2. Native spring forces recover each individual prepared
piece-pair's force and moment. The elastic line-group solution gives throat
traction at every effective weld endpoint. Collinear groups receive no credit
for a moment outside their resisting subspace. Global force balance, artificial
coupling energy and modal mass/strain-energy checks guard every solver packet.
A native two-body wrench fixture checks both force and moment recovery.

Open runs lose one throat at each end, unless the drawing explicitly requires
full-size run-on/off tabs and inspection of the finished ends. Connected short
returns belong to their physical run. Isolated runs shorter than
max(30 mm, 6 throats) carry no structural weld load. Flush butt edges do not
become imaginary fillets. These effective-length rules follow
[the JRC Eurocode worked examples](https://publications.jrc.ec.europa.eu/repository/bitstream/JRC96658/jrc_steel_report_2015_07_22.pdf).

The S235 lower-bound simplified throat-strength limit is
360/(√3 × 0.8 × 1.25) = **207.85 MPa**; see
[SCI P358](https://www.steelconstruction.info/images/a/a9/SCI_P358.pdf).
Fatigue adopts EN 1993-1-9:2005's **FAT36** weld-throat root category and
**FAT80** longitudinal shear category, described in
[SCI guidance 2.12](https://www.steelconstruction.info/images/1/10/GN-2-12.pdf).
The assessment conservatively applies FAT36 to the full traction-vector range,
without the static √3 multiplier and without crediting the higher shear limit.
The component interval encloses every pair of signed load reversals.

With γMf=1.15, the variable-amplitude cut-off is
36 × (2/5)^(1/3) × (5/100)^(1/5) / 1.15 = **12.669 MPa**.
The m=3 branch ends at five million cycles; m=5 continues to 100 million.
Use this cut-off even though a strictly constant-amplitude spectrum could
justify the higher five-million-cycle endurance limit. This retains the
approved life without claiming a better detail category for a larger bead.

Static reactions cover all nine full-Z poses on each bed. Dynamic qualification
uses the governing modal pose, 20 and 40 modes, 0.5% and 1% damping, and native
jerk-limited profiles at 3 m/s², 20 m/s³, 20 m/min contour and 40 m/min rapid.
The contour, rapid-to-cut and rapid-stop histories include omitted-mode static
residuals and an analytical bound on the entire subsequent vibration tail.
Dynamic qualification does not claim a modal transient at every static pose.
The tool-motion gates remain 0.05 mm contour deviation and 0.05 mm residual.

## Detail corrections

- Remove accidental grazing contacts: table crossmembers and plan braces have
  1 mm non-weld cope clearance to posts; end braces have the same clearance to
  the upper side rails. Loads use the intended continuous stringers. The native
  prepared-solid and weld connectivity checks verify the remaining load path.
- Use full accessible cope runs at post/stringer joints. The two middle
  post01/post11-to-tableSide joints require **6 mm legs** for fatigue; other
  base groups retain nominal **5 mm legs**. On the **2600 bed**, use
  sacrificial run-on/off tabs on their four 100 mm runs per joint, with
  full-size finished-end inspection. The 3150 bed needs no such end credit.
- Each welded beam foot requires **two 92 mm fillets with 8 mm legs** and
  sacrificial run-on/off tabs, removed and blended after cooling. Inspect full
  throat through both finished ends. The 5.657 mm throat is below the RHS's
  6.3 mm wall. The narrow beam-end overhang cannot accommodate an all-around
  fillet, so none is credited. See the unit machining checklist in
  [DIY_ROUTE.md](DIY_ROUTE.md).

The old 5 kN/arbitrary-moment screen selected stitch extents; it does not decide
release once actual reactions are available. Its failed group numbers must be
mapped by stock names because removing grazing contacts renumbers the groups.
Per-group reaction reports distinguish a passing structural weld from an
uncredited short incidental run; an uncredited run is never labelled a pass.

## Scope of shop release

These are reduced elastic member/weld-group checks, not local weld-toe or RHS
wall-distortion finite elements. The per-group table records the weld-throat
category and range; it does not assert a certified parent-metal hot-spot
assessment. The signed shop review must accept the actual cope geometry,
material identity, welding procedure/access and tabbed-end witness detail.
Keep fabrication release gated until that review, the relief route, epoxy
application qualification and full-unit machining capability are recorded.
Measured relief/rest, rail alignment and SR6-C commissioning remain build
witnesses and cannot be signed off by this model.
