# EAIK spike, 2026-10-06

Decision: adopt EAIK for supported 6R families. Correctness and browser gates
pass; the worst median ratio is **1.27263**, below the specified limit of 2.
The production adapter and deletion of OPW/handwritten UR remain pending.

## Correctness

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

## Accepted timing

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
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DMK_BUILD_SHARED=OFF -DMK_BUILD_EAIK_SPIKE=ON
cmake --build build/pp1a-eaik-wasm --target motionkit_eaik_spike --parallel 4
node build/pp1a-eaik-wasm/motionkit_eaik_spike.js
```

External evidence is under `/home/joao/dev/materia-cache/claude-scratch/`:
`process-path-pp1a-model-parity-final.log`, `process-path-pp1a-native-ctest.log`,
`process-path-pp1a-wasm-exceptions-runtime.log`, and
`process-path-pp1a-timing-quiet.log` with its metadata JSON. Rejected timing
runs retain their separate logs and contention metadata.
