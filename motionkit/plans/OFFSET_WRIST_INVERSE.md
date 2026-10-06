# Offset-wrist algebraic inverse reduction

Research derivation for `OffsetWristGeometry`; no production solver or completeness claim yet.
Use canonical angles after applying extracted signs/reference offsets and the existing
base/tool/external-cell transforms. Dimensions are the extracted OPW dimensions and
middle-axis displacement d. Target flange position/rotation are p,R.

Let alpha=q1, beta=q2+q3, gamma=q6. Define

```
x = (cos(alpha), sin(alpha), 0)
y = (-sin(alpha), cos(alpha), 0)
z = (0, 0, 1)
m = R * (sin(gamma), cos(gamma), 0)
w = p - c4 * R*z - d*m
n = sin(beta)*x + cos(beta)*z
f = (a2*cos(beta) + c3*sin(beta))*x
    + (-a2*sin(beta) + c3*cos(beta))*z
v = w - a1*x - c1*z - f
```

The three scalar equations are

```
w dot y = b
m dot n = 0
v dot v = c2*c2 + b*b
```

The lateral equation puts the wrist centre in the arm plane at its actual lateral
offset. The orientation equation makes the middle wrist axis perpendicular to
the forearm axis. The length equation enforces the upper arm length after removing
the forearm displacement. No OPW branch-validity interval or residual-angle wrapping
appears in this formulation.

For a real solution, reconstruct q2=atan2(v dot x, v dot z), q3=beta-q2.
Let A=Rz(alpha)*Ry(beta), h=A.transpose()*m. Then h.z=0 and
q4=atan2(-h.x,h.y). The rotation B=Rz(-q4)*A.transpose()*R*Rz(-gamma)
fixes Y, so it is Ry(q5); recover q5=atan2(B[0,2],B[0,0]). q6=gamma.
Full FK verification remains required after floating-point reconstruction. Apply
signs/offsets, enumerate legal periodic lifts and verify real joint limits separately.
The existing native parameter checks require c2>0, so the q2 reconstruction vector
has nonzero length. This does not dispose of positive-dimensional solution sets.

For algebraic isolation, express each angle using homogeneous half-angle coordinates
(S,T): sin=2*S*T/(S*S+T*T), cos=(T*T-S*S)/(S*S+T*T).
Clear denominators in these three equations to obtain a polynomial system in three
projective angle pairs. Every real projective pair is nonzero and its denominator
is positive; clearing these denominators introduces no real denominator zeros.
A single affine tan(angle/2) chart misses angle pi. Isolation must include the
complementary charts or explicitly solve their boundary systems, deduplicate shared
roots, and handle positive-dimensional/singular systems. Complex denominator zeros
can introduce elimination factors and must not be accepted as physical roots.

The finite-angle polynomial system is a candidate replacement for the sampled OPW
consistency scan, not a completed isolator. Next implementation work: derive and
inspect its coefficients/elimination degree, isolate every real root including
multiple roots and chart boundaries, diagnose continua, then compare reconstructed
solutions with the existing authored/fixture FK proofs before native integration.

Validation experiment: deterministic seed 712019, 1,000 six-angle configurations
uniformly distributed over [-pi,pi], dimensions a1=.17,a2=-.09,b=.08,c1=.4,
c2=.6,c3=.5,c4=.12,d=.035. Canonical FK uses
Rz(q1)*Ry(q2+q3)*Rz(q4)*Ry(q5)*Rz(q6) and the vendor OPW wrist-centre
formula plus d*m. Maximum absolute residuals: lateral 1.8041124150158794e-16,
orientation 2.220446049250313e-16, squared length 3.885780586188048e-16.
The experiment exited zero. This verifies the reduction on generated configurations;
it neither enumerates target roots nor proves runtime performance or whole-plan acceptance.

## Eliminate beta without inverse trigonometric branch scans

Under the lateral constraint, let X=w dot x-a1, Z=w.z-c1,
M=m dot x, N=m.z and

```
L = w dot w - b*b - 2*a1*(w dot x) - 2*c1*w.z
    + a1*a1 + c1*c1 + a2*a2 + c3*c3 - c2*c2
A = a2*X + c3*Z
B = c3*X - a2*Z
K = A*M - B*N
```

The remaining equations become
`M*sin(beta)+N*cos(beta)=0` and
`L=2*(A*cos(beta)+B*sin(beta))`. If H=sqrt(M*M+N*N)>0,
then `(cos(beta),sin(beta))=sigma*(M,-N)/H`, sigma in {-1,+1}.
Eliminating beta gives

```
L*L*(M*M+N*N) - 4*K*K = 0
```

Recover and check both signs against the unsquared length equation. Squaring
retains both physical signs but numerical candidates still require reconstruction
and full FK verification. If H=0, this elimination becomes identically zero and
is insufficient: solve the original length equation in beta separately. This is
an explicit degeneracy, not permission to divide by H or discard the target.

Exact-rational elimination experiment: the dimensions above, p=(.6,.2,.7),
and rotation

```
[  2/15  -2/3  11/15 ]
[ 14/15   1/3   2/15 ]
[  -1/3   2/3    2/3 ]
```

were verified orthogonal exactly in SymPy. Substitute u=tan(alpha/2),
t=tan(gamma/2), cancel rational denominators, and take primitive numerators.
The lateral polynomial has bidegree (2,2), nine terms. The eliminated length
polynomial has bidegree (8,8), 81 terms. Their resultant eliminating u has
degree 32 and factors into degrees/multiplicities (2,4), (4,2), (16,1).
The resultant step took 0.019 seconds; symbolic rational cancellation was much
slower. The experiment exited zero. These are exact results for this rational
instance, not a generic degree proof, performance measurement for compiled IK,
or proof that every factor except degree 16 is extraneous. Factor classification,
root recovery and projective/degenerate systems remain to implement and verify.

## Reproducible exact-rational prototype

Run `python3 motionkit/scripts/research/offset-wrist-polynomial.py` with SymPy
available. It constructs common-denominator polynomial numerators directly,
computes and factors the resultant, isolates its real roots with rational
intervals of width at most 1e-20, back-substitutes the lateral quadratic,
reconstructs both beta signs and verifies canonical FK. This is a research tool;
SymPy is not a production dependency. Affine base-boundary and projected-axis
degeneracies explicitly raise rather than silently skip. Final-wrist angle pi
is outside this prototype's affine chart and remains unhandled.

For the rational instance above, the degree-two factor is t^2+1 and has no
real roots. The degree-four factor also has zero real roots (exact root count).
The degree-16 factor has eight real roots. Reconstruction returns eight target
solutions; maximum matrix-entry/position FK error is 3.885780586188048e-15.
The final experiment exited zero and took 0.439 seconds including coefficient
construction, exact isolation and reconstruction. Output is retained in
`/home/joao/dev/materia-cache/claude-scratch/process-path-offset-polynomial.json`.
This finite-instance proof does not classify extraneous factors generically,
cover projective boundaries/continua, apply actual joint limits, or meet native
bulk performance and authored track gates. No production family changed.

## Complementary projective charts

The prototype now constructs direct and reciprocal half-angle charts for both
base and final-wrist angles, solving all four combinations. Numerator formulas
switch cos from (1-u^2)/(1+u^2) to (u^2-1)/(1+u^2) in the reciprocal chart;
sin retains 2u/(1+u^2). Coordinate zero in that chart represents angle pi.
Only |coordinate|<=1 is retained in each chart to avoid large-coordinate
reconstruction; overlaps are deduplicated modulo rotary turns. Lateral
back-substitution supports linear degree drops. Identically zero resultants,
vanishing lateral polynomials and projected-middle-axis degeneracy still produce
explicit diagnostics rather than a completeness claim.

`--check-boundaries` generates exact-rational FK targets for base pi, final wrist
pi and both pi. All three recover their generating six-joint configurations
modulo turns; each returns four FK-verified solutions, maximum error
2.220446049250313e-16. The original target still returns eight solutions after
four-chart deduplication. Actual script exit zero, 6.844 seconds including all
four targets (`process-path-offset-boundaries.json`); original-target-only run
exit zero, 1.467 seconds (`process-path-offset-charts.json`). Earlier single-chart
limitations above describe the preceding prototype revision.

This improves finite-chart coverage, not a certified inverse solver. Floating
coefficient degree/drop/discriminant thresholds during back-substitution can
still lose close or repeated roots. The exact univariate root isolation does
not certify the subsequent floating reconstruction. Generic factor classification,
projected-axis special systems and positive-dimensional families remain open,
as do native runtime performance, real model limits and authored acceptance.

## Projected-axis degeneracy reconstruction

When M=N=0, isolate beta using A*cos(beta)+B*sin(beta)=L/2 directly.
For amplitude hypot(A,B)>0, beta=atan2(B,A) +/- acos(L/(2*amplitude));
then apply the original length and full FK checks. If amplitude=0 and L!=0
there is no solution; if amplitude=L=0, beta is a continuous family and the
prototype raises an explicit parameterized-output diagnostic. Floating thresholds
remain experimental, with no certification near these degeneracies.

The boundary experiment additionally generates exact-rational targets with q4=0
and q4=pi. Both recover their generating configurations; each returns four
solutions, two through the special projected-axis reconstruction. Maximum FK
errors are 1.12e-16 and 6.67e-16 respectively. A separate constructed target
with a2=.3,c3=.4,c2=.5 and zero X,Z checks that the continuous-beta case raises
the intended diagnostic instead of returning a finite root list. The final full
research experiment exits zero; evidence is
`process-path-offset-degenerate-final.json`. These checks support the special
reconstruction formula but do not implement parameterized continuum output or
certify the elimination/back-substitution in other singular systems.

## Deterministic reachable-target sweep

`--check-random=12` generates exact-rational reachable targets from deterministic
six-joint half-angle tangents (seed 712019, each numerator in [-9,9], denominator
in [1,9]). FK generation is shared with the chart-boundary fixtures. The combined
`--check-random=12 --check-boundaries` experiment exits zero: all twelve generating
configurations recovered modulo turns, with four to eight solutions per target.
All existing five finite boundary/projected-axis fixtures and the continuum
diagnostic also pass. Maximum sweep FK error is 2.502442697505103e-13; combined
elapsed time is 32.803 seconds. Evidence: `process-path-offset-random-final.json`
and per-sample progress `process-path-offset-random-final.log`.

The initial refactor experiment exited one because the extracted FK helper still
referenced its former local `wrist` variable. It was corrected to `tangents[5]`
before the successful combined run. The failure is not inverse-coverage evidence.
This finite sweep does not prove all-root completeness; it verifies original-root
recovery and checks every returned candidate against FK. Certified coupled-root
back-substitution and continuum representation remain necessary before default
family integration.

## Exact coupled polynomial recovery certificate

`--certify` computes the subresultant sequence in the base coordinate. For each
irreducible resultant factor having real roots, the penultimate subresultant must
be linear a(t)*u+b(t). The prototype inverts a in QQ[t]/factor and constructs
u=-b/a in that quotient field. It verifies exactly, by modular Horner substitution,
that both lateral and eliminated-length polynomials vanish at this coordinate.
It also checks that their leading base coefficients do not vanish modulo the
factor, avoiding a degree-drop claim based on unspecialized subresultants.
Nonlinear/vanishing recovery or leading-degree drops produce explicit diagnostics.

Certified factors use this exact coordinate map for back-substitution rather than
the floating quadratic's discriminant/degree thresholds. The isolated t interval
midpoint is rational; coordinate evaluation uses exact rational arithmetic before
conversion to Float. This is not an interval enclosure for the resulting u or a
bound on midpoint reconstruction error; certified adaptive conversion remains open.

Final `--certify` experiment exits zero on the original rational target. All four
charts certify the degree-16 factor, each with eight real roots; the other factors
have no real roots. Deduplicated reconstruction retains eight FK-verified solutions,
maximum error 1.3322676295501878e-15, elapsed 4.377 seconds. Evidence:
`process-path-offset-certificate-final.json`. This establishes exact coupled
polynomial recovery for these factors, not a general all-target inverse certificate.
The option currently applies to the original target only; boundary/random fixtures
retain their earlier floating quadratic mode. Projected-axis/continuum and other
singular factors still need specialized recovery, and production remains unchanged.

## Rational coordinate enclosures

Certified mode now evaluates the recovered coordinate polynomial by exact rational
interval Horner arithmetic over the isolated final-angle interval. Root intervals
are refined until both chart coordinates have decided membership in [-1,1] and
the base-coordinate enclosure width is at most 1e-14. Outside-chart intervals are
rejected by exact bounds; no floating membership tolerance is used for that step.
The midpoint of the enclosed coordinate is then converted to Float. Output retains
both coordinate intervals and refinement counts. This replaces the previous
uncertified coordinate-midpoint conversion, while atan/trigonometric reconstruction,
singular thresholds and FK acceptance remain floating and uncertified.

Final `--certify` experiment exits zero: eight solutions/eight retained chart
coordinate enclosures, maximum base width 4.829e-17, FK error at most 1.34e-15.
A separate high-cancellation polynomial enclosure around sqrt(1/2) requires seven
refinements and verifies its true coordinate using rational squared inequalities.
An exact chart-boundary coordinate is retained and an outside-chart coordinate
rejected. Evidence: `process-path-offset-intervals-final.json`, total 4.436 s.
These helper checks validate refinement, not all-target/singular-family completeness.

## Exact recovery across the reachable-target sweep

`--certify --check-random=12` now applies exact coupled recovery and rational
coordinate enclosures to each deterministic reachable target, rather than only
the original fixed target. The actual run exits zero and recovers all twelve
generating configurations modulo turns. It returns the same per-target solution
counts (four to eight) as the previous floating sweep, with maximum FK error
3.219646771412954e-14. The sweep includes q4=0 (sample 5), q5=0 (sample 4),
and base half-angle coordinate 1 (samples 10/11); these factor/reconstruction cases
also pass. Evidence: `process-path-offset-certified-sweep.json` and progress log
of the same stem. Combined elapsed time is 138.363 seconds including the fixed
reference target and interval helper checks.

This verifies the certificate path across twelve exact-rational reachable targets;
it is not a generic all-root/singular-system proof. The continuum diagnostic remains
an unsupported family representation, and trigonometric reconstruction remains
floating. Symbolic runtime is unsuitable for bulk planning: this reference must
inform native polynomial generation/isolation and actual compiled model acceptance,
not become the production candidate sampler. No authored track or full phase gate
was rerun or accepted from this evidence.

## Native coefficient prototype

`motionkit/native/src/offset_wrist_polynomial.h` constructs the two bivariate
polynomial numerators in fixed 9-by-9 long-double storage for each projective
chart. Polynomial operations diagnose storage overflow; no ABI, inverse family,
root search or production candidate path is added. This is an internal prototype.
The builder currently assumes valid finite dimensions and a proper rotation;
input validation belongs in the eventual native solver boundary.

Reproduce the exact-reference comparison with:

```
c++ -std=c++17 -O2 -Wall -Wextra -Werror motionkit/scripts/research/offset-wrist-coefficients.cpp -o /tmp/offset-wrist-coefficients
python3 motionkit/scripts/research/offset-wrist-polynomial.py --check-native=/tmp/offset-wrist-coefficients
```

The retained scratch executable was built and the committed checker actually run;
both exit zero. Every one of 648 coefficient slots across all four charts is
checked, including zeros and the exact expected key set. Maximum absolute error
against rational coefficients is 1.204271456790123456790123456790123456790123e-18.
Evidence: `process-path-native-offset-coefficients-final.json`. This proves the
coefficient translation for one rational target, not generic conditioning,
root isolation/recovery, a runtime-family integration or authored performance.
Full native/runtime phase gates remain pending until implementation integration.

## Native coefficient sweep and validation

The native prototype now rejects nonfinite target/dimension values, nonpositive
c2/c3, negative c4, nonorthonormal/improper rotations and nonfinite arithmetic
results. Rotation orthogonality/determinant tolerance is 1e-10. This is internal
validation, not an exported ABI boundary.

The committed emitter accepts target packets on stdin. `--check-native=...` now
checks eighteen targets: the original rational target, twelve deterministic
reachable targets and five chart/projected-axis boundary targets. Strict standalone
compilation and final checker exit zero. All 11,664 coefficient slots match their
exact-rational references; maximum absolute difference 2.68701421224109e-17.
Eight invalid-input cases (nonfinite rotation/position/offset, invalid lengths,
nonorthogonal rotation and reflection) are rejected. Evidence:
`process-path-native-offset-sweep-final.json`.

The first expanded sweep exited one because a coefficient error 1.186e-17
exceeded the former fixed 1e-17 envelope. The checker now uses the explicitly
scaled envelope 1e-16*max(1,abs(exact coefficient)); this tests coefficient
translation rather than imposing the first target's measured absolute precision
on differently sized coefficients. The preceding failure is not reported as green.
This finite sweep does not bound conditioning for all inputs, validate the arithmetic
overflow branch itself, isolate roots, register a native solver, or satisfy runtime
phase/track acceptance. Those original requirements remain open.
