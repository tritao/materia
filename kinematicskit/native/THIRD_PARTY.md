# Third-party code in kinematicskit/native

## proxsuite (ProxQP)

- Source: https://github.com/Simple-Robotics/proxsuite, pinned submodule at
  `vendor/proxsuite`, tag v0.7.3 (commit b93d7778ffc3299d84b5cb0851022a29bf24a596).
- Licence: BSD 2-Clause, Copyright (c) 2022-2026 Inria (`vendor/proxsuite/LICENSE`).
- Use: header-only. Only `include/proxsuite/proxqp/dense` (and the
  `linalg`/`helpers` headers it includes) is compiled, from `src/qp.cpp`.
  proxsuite's own CMake is not used; `src/proxsuite_config/proxsuite/config.hpp`
  supplies the four version macros its build would generate.
- Not used: the sparse and parallel solvers, SIMD vectorisation (simde,
  `PROXSUITE_VECTORIZE` is not defined), serialization (cereal), Python
  bindings (nanobind), and the upstream submodules
  (`cmake-module`, `external/cereal`, `bindings/python/external/nanobind`),
  which stay uninitialized.
- Dependency: Eigen 3 (MPL-2.0), found with `find_package(Eigen3 3.3)` as in
  MotionKit.

Updating: bump the submodule to a new tag, update the version macros in
`src/proxsuite_config/proxsuite/config.hpp` and this file, then rerun
`native/tests` (the QP parity and optimality tests).
