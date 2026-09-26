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
