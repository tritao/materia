# EAIK spike, 2026-10-06

Decision: adopt EAIK for supported 6R families. Correctness and browser gates
pass; the worst median ratio is **1.27263**, below the specified limit of 2.
Production adoption is complete: the compiled-chain/base adapter, normalized
labels/turns, OPW/handwritten UR deletion and adoption phase gate pass. The
100,000-target production timing report is non-gating.

## Historical spike correctness

- Published IRB2400, KR6, R2000 and TX40: 1,000 fixed-seed targets each;
  all valid OPW solutions contained within 1e-9 rad after wrapping.
- Compiled RobotArm700/900/1300 and Cobot500/850/900/1300: 1,000 regular
  targets plus five independent wrist-singularity targets each. H/P vectors
  come from the compiled joint chain, including fixed mounts and flange
  rotation; targets come from KinematicGroup FK. No industrial dimensions
  are duplicated in the native test. All exact solutions round-trip within
  1e-9 m and 1e-9 rad; repeated calls produce identical solution sets.
- Industrial compiled models contain every valid OPW solution. Singularities
  require an exact FK-valid solution and deterministic output; their continuous
  solution families do not require identical arbitrary wrist representatives.
- EAIK-marked least-squares alternatives are excluded from exact comparisons.
- Unknown generic 6R decompositions reject explicitly. This revision offers
  closed-form decompositions, not a general 1-D search fallback. Search-case
  latency is unavailable; arbitrary future 6R mechanisms are not certified.
- Native Release build and full CTest: 17/17.
- Emscripten 6.0.9: C++ core/spike builds and four-fixture runtime check passes
  under Node, including caught unsupported-family rejection. C++ exception
  handling is enabled for these targets. This is not an interactive browser
  application or a browser performance measurement.

## Historical accepted spike timing

Intel Core i5-13600K, GCC 13.3.0, Release, one thread on CPU 16 (no SMT
sibling). Fixed RNG seed `0x50503161`; 100,000 reachable targets per arm;
1,000 warm-up calls per backend. FK, coordinate conversion and target
allocation are outside the timed calls. Calls alternate OPW/EAIK order,
consume all-solutions results and use steady-clock microseconds.

| Arm | OPW median µs | OPW p95 µs | EAIK median µs | EAIK p95 µs | Median ratio |
|---|---:|---:|---:|---:|---:|
| Compiled RobotArm900 | 2.861 | 3.095 | 3.641 | 3.795 | 1.27263 |
| IRB2400 | 2.987 | 3.237 | 3.606 | 3.786 | 1.20723 |

The accepted run completed with exit zero, 1.677677 s elapsed and 1.677440 s
child CPU. CPU 16 recorded 168 busy ticks at 100 Hz, accounted for by the
benchmark within tick resolution. Other cores were active; the benchmark core
had no measurable competing execution. Later contended reruns are excluded.
The accepted timing preceded the additional independent singularity records;
the 100,000-target sampler and both timed kernels were unchanged.

## Reproduction

EAIK is pinned at `3d973f6b7e7c87ca32928ec5b7eb5e50d956a7f5`, IK-Geo at
`fea1d2e7e3fa4c5e3777c37b0af0e0612b3a13e9`, using MotionKit's Eigen.
Initialize EAIK and its `CPP/external/ik-geo` submodule, not its Eigen copy.
From the worktree root, build `machinekit/tests/haxeon.eaik-model-export.json`
with Haxeon and run its `EaikModelExport` entrypoint with
`EAIK_MODEL_FIXTURES` set to the desired output file. Runtime library paths
must include the existing CadKit, CadBridge and kinematics native builds.

```sh
cmake -S motionkit/native -B build/pp1a-eaik -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DMK_BUILD_EAIK_SPIKE=ON
cmake --build build/pp1a-eaik --parallel 4
ctest --test-dir build/pp1a-eaik --output-on-failure
build/pp1a-eaik/motionkit_eaik_spike "$EAIK_MODEL_FIXTURES"
taskset -c 16 build/pp1a-eaik/motionkit_eaik_spike \
  "$EAIK_MODEL_FIXTURES" --benchmark
emcmake cmake -S motionkit/native -B build/pp1a-eaik-wasm -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DMK_BUILD_SHARED=OFF -DRK_BUILD_SHARED=OFF
cmake --build build/pp1a-eaik-wasm --parallel 4 \
  --target motionkit_eaik_tests motionkit_candidate_sampling_tests
node build/pp1a-eaik-wasm/motionkit_eaik_tests.js
node build/pp1a-eaik-wasm/motionkit_candidate_sampling_tests.js
```

External evidence is under `/home/joao/dev/materia-cache/claude-scratch/`:
`process-path-pp1a-model-parity-final.log`, `process-path-pp1a-native-ctest.log`,
`process-path-pp1a-wasm-exceptions-runtime.log`, and
`process-path-pp1a-timing-quiet.log` with its metadata JSON. Rejected timing
runs retain their separate logs and contention metadata.


## Production adoption follow-through

The production adapter now exports the compiled 6R axes/link displacements and
terminal flange rotation, composes the current upstream arm base and workpiece
frame, and enumerates legal physical turns. Configuration labels are computed
geometrically and normalized to shoulder/elbow/wrist bits 2/1/4; they do not
use EAIK's returned solution order. The industrial mapping was cross-checked
against the retired solver on 3,000 authored targets before its deletion.
The OPW implementation/vendor and the handwritten UR inverse are removed.
The point inverse adapter retains seed-relative periodic-lift ordering.

At an exact spherical wrist pole, the native adapter preserves the seed's
fourth physical joint and analytically recovers the coupled sixth angle.
It does this before the final exact-FK filter: the pinned core can return a
singular representative with an unusable wrist rotation, even when its
shoulder/elbow solution is valid. Every emitted result must still satisfy
1e-9 m/rad FK. No numeric IK polishing or hidden numeric fallback is added.

The 100,000-target benchmark is an **optional, non-gating report**. Historical
OPW/EAIK ratios above remain evidence for the original decision, rather than a
new production acceptance threshold. After removing OPW, `--benchmark` reports
raw EAIK and the production exact-filter adapter separately for RobotArm900 and
IRB2400. The original comparison source/results remain in repository history
and the cited scratch logs, rather than retaining a second production solver.

An exploratory TX40 target from a different RNG/range missed the original
joint vector by about 1.2e-9 rad near an elbow singularity, while passing the
1e-9 m/rad task-space filter. The native test keeps that exact deterministic
case as a non-gating joint-accuracy observation; it does not claim universal
1e-9 joint-coordinate accuracy at every singularity. The regular published
fixture gate uses the original PP1a `mt19937_64(0x50503161)` sequence over
[-2.8,2.8], and retains its strict 1e-9 wrapped-joint/FK checks. All seven
compiled-model gates retain their original 1,000 regular / five singular
sample set and exact-FK tolerances. Adoption phase-gate results and the
per-step G17 profile are recorded in PROCESS_PATH_PLANNING.md.


Production phase gate: native CTest 15/15, four-platform FFI audit, two
Emscripten exact-inverse/candidate runtimes, all seven compiled-model fixture
sets, full MachineKit script, compiler-only app/downstream builds and full
MotionKit/ProcessKit/RobotKit/CadBridge/Toolpath motion runtimes pass. Details
and the per-step G17 profile are recorded in PROCESS_PATH_PLANNING.md.

The non-gating production report (100,000 targets, 1,000 warmups, GCC 13.3
Release, one thread pinned to CPU 16) measures raw core / exact-filter adapter
median (p95) microseconds: RobotArm900 3.502 (3.788) / 13.145 (13.689); IRB2400
3.397 (3.776) / 13.112 (13.615). These are unlabelled calls. Other system
activity was not excluded; no uncontended speed acceptance is inferred.
Log: external scratch process-path-pp1-production-timing.log.
