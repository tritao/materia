# Third-party native code

## Ruckig Community

- Upstream: <https://github.com/pantor/ruckig>
- Git submodule: `vendor/ruckig`, release tag `v0.19.4` at
  `a8db97a4e9c55e5160a3855f739fa3b270df8e4c`.
- License: MIT; full text at `vendor/ruckig/LICENSE`.
- Language: C++20, linked statically into `motionkit_core`.

The build disables `BUILD_CLOUD_CLIENT`, examples, Python bindings, tests and
benchmarks before adding the submodule. The audited compiled sources are
`brake.cpp`, `position_first_step{1,2}.cpp`,
`position_second_step{1,2}.cpp`, `position_third_step{1,2}.cpp`,
`velocity_second_step{1,2}.cpp`, and `velocity_third_step{1,2}.cpp`, all under
`src/ruckig/`. `src/ruckig/cloud_client.cpp` and the bundled HTTP/JSON
dependencies are not compiled or linked; a CMake check rejects that source if
upstream build defaults change. MotionKit uses only offline `calculate` with
no intermediate waypoints.

## TOPP-RA C++

- Upstream: <https://github.com/hungpham2511/toppra>
- Git submodule: `vendor/toppra`, version 0.6.10 at
  `8121e9af72f5aa589b515aa15e0dc408344c3220`.
- License: MIT; full text at `vendor/toppra/LICENSE`.
- Dependency: Eigen 3, MPL-2.0, provided through `Eigen3::Eigen`.

The audited compiled C++ sources are `constraint.cpp`,
`constraint/linear_joint_velocity.cpp`,
`constraint/linear_joint_acceleration.cpp`, `solver.cpp`,
`solver/seidel.cpp`, `geometric_path.cpp`,
`geometric_path/piecewise_poly_path.cpp`, `algorithm.cpp` and
`algorithm/toppra.cpp`, all below `cpp/src/toppra/`. The CMake source
filter excludes qpOASES, GLPK, Python bindings, Pinocchio, torque and
Cartesian constraints, and parametrizer implementations. Only the Seidel
LP solver is constructed by MotionKit.
