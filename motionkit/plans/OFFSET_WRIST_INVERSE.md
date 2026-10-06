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
