# Supported-rack rerun preparation — 2026-10-25

2026-10-26: the [executed rerun](J6_EXECUTED_RERUN.md) now records the completed
36-case frozen reference and 186-history replay. The preparation text below is
historical scope, not a claim that the real mount has been qualified.

No frozen CAD change, structural solve, modal replay or new acceptance claim is
made here. Root has about 5.3 GB free; the minimum for replay/solves remains 8 GB.
DATA1 is full and receives no writes.

## Executed motion handoff

[Preflight](rack-results/executed-handoff.json) admits all 186 histories from
`/home/joao/dev/materia-worktrees/jerk-timing/build/om-r1/ROUTER_EXPORT_HANDOFF.md`:
90 pose-grid, 64 phase-grid and 32 full-feed-resume histories. It checks the
producer manifests, hashes, execution identity, clock certificates, mapped
limits and pose bindings. Contour jerk and the higher assumed physical-drive
jerk ceiling are retained separately. These generic executor limits still need
qualification against the selected router drives.

Use nominal, executed hold/resume and ≤100% clock overrides. Above-100% time
scaling is rejected; raised-feed next-window replans remain outside this grid.
Old offline stop/resume suffixes are excluded. Publication-reserve timing is
inflated: cycle times and comparisons are deliberately suppressed. Producer
errors belong to its frozen structure and are not copied into our model.

`executed_rerun.py` defaults to preflight only. Its optional modal replay requires
two corrected structural meshes per pose, at least 40 modes and an explicit
corrected-mount provenance marker. It checks history/index hashes again during
replay, so a re-export requires a new preflight. Replay is prepared but has not
been exercised on corrected structural results. It reports geometric contour,
handover, physical-rest residual and braking-following separately; the latter
is a diagnostic, not an unapproved change to SR-D3. Modal history starts at the
rapid lead-in. Frequency/damping sensitivity and SR6-C remain required.

## Mount placement and service clearance

[Envelope screen](rack-results/mount-clearance-plan.json) checks 18 poses per bed,
56,448 body-pair envelopes in total. Pinion axial centres come from their mesh
poses plus the explicit 20 mm axial width. The local fixture had used the axial
face origin as the centre, producing a 10 mm lateral reference discrepancy.
The proposed corrected receiver uses local rail-width coordinate −28.9 mm,
versus −38.9 mm in that fixture. This is a prototype correction; frozen rail
locations are unchanged.

There are 19 distinct overlap candidates per bed involving the preload mount,
fixed chain bracket and switch brackets. Bounding-box overlaps request native
solid/access checks; they do not prove collisions. Local nut capture and key
access passed the earlier fixture checks, but full-machine wrench corridors,
cassette withdrawal, section joints, X host and articulated chains remain open.
Do not claim a complete mount or feed this candidate into release acceptance.

## Geometry-derived compliance and provisional drive ranking

[Local compliance](rack-results/local-mount-compliance.json) uses the verified
native fixture, actual bolt grips/washer areas and hole stations, coupled
receiver section inertia, bar bending, bolt deformation and an assumed 4 GPa
replication layer. Reaction weights conserve force and moment and their squares
enter flexibility. Contact compression and bolt-supported uplift are separate;
there is no grout tension or unqualified friction credit. Washer-annulus and
tributary-foot contact areas are sensitivities, not confidence bounds. Joint
rotation, thread/bearing deformation, weld flexibility, preload and creep remain
unqualified. This is a local linear screen, not an automatic full-machine mount
compliance export.

At the corrected placement the existing three-anchor section gives 8.56–8.97
MN/m equivalent seat stiffness. Adding anchors at ±230 mm on this 500 mm rack
section gives 20.49–20.92 MN/m, adding 0.0243 kg of actual local hardware. These
anchors are a study proposal requiring native clearance/detail checks. Local
fixture mass is 11.585 kg including rail/rack, or 9.001 kg excluding them;
different fixture lengths prohibit extrapolating these totals per metre or bed.

[Drive preparation](rack-results/y-drive-preparation.json), using the conservative
end-anchored contact case, ranks idealized changes as follows. All values are
per pinion, including seat compliance; dual-Y stiffness is twice these values.

| Stiffness-only option | MN/m |
|---|---:|
| Existing drive plus proposed seat | 6.02 |
| Reducer stiffness ×2 | 6.43 |
| Motor/preload mount stiffness ×2 | 7.81 |
| Mount ×2 and reducer ×2 | 8.52 |
| Mount ×4 | 9.18 |
| Mount ×4 and reducer ×2 | 10.16 |

The frozen drive without a seat spring was 8.53 MN/m per pinion. No replacement
hardware, moving-mass delta, frequency benefit, error reduction or cycle-time
benefit is established by these multipliers. Module 2 cannot be substituted into
this foot unchanged: its pitch height leaves 5.08 mm under a 6 mm counterbore.

Next: correct the prototype datum and resolve native mount/preload/service
clearances, complete X/section/end details and full mount masses, then propose
the corrected geometry. After approval and >8 GB free, rerun both beds/meshes
with the executed histories, weld fatigue at 100 million reversals and options
(a) shaping, (b) scheduled limits, (c) actual stiffer-drive candidates. Cycle-time
cost waits for corrected publication timing. The motor mount and receiver end
anchoring are the first provisional stiffness candidates.

## Lightweight validation

15 focused tests cover rejected histories/certificates, pose binding, disk-floor
blocking, reaction equilibrium, section tensors, SI projection, positive
compliance and hardware mass. Run with:

```sh
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s machinekit/examples/sheet-router/fem/tests -p test_rerun_preparation.py -v
```

The scripts read retained native and frozen-input archives in memory without
bulk extraction. Reports contain input hashes. The prior native fixture evidence
remains linked from `mounting/geometry-validation.json`; no new native CAD build
was performed for this checkpoint.
