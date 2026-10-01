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

## coal (collision and distance)

- Source: our fork https://github.com/tritao/coal, branch `materia`, pinned
  submodule at `vendor/coal`. Based on upstream coal-library/coal tag v3.0.4
  (commit f0fecd0a); the branch adds one commit (f92f288b).
- Licence: BSD 3-Clause (upstream `vendor/coal/LICENSE`; coal descends from
  FCL and hpp-fcl).
- Use: the collision and distance core (shapes, GJK/EPA, bounding-volume
  trees, broad phase, height fields) built as a static library by
  `vendor/coal/materia/CMakeLists.txt` (target `coal::core`), Eigen only.
  The branch removes Boost from the core headers (pi from `EIGEN_PI`, an
  unused `boost/function` include dropped); `materia/include` stands in for
  the `config.hh`/`deprecated.hh`/`warning.hh` coal's CMake would generate.
- Not used: mesh-file loading (`mesh_loader`: assimp, Boost.Filesystem),
  serialization (Boost.Serialization), octrees (octomap), logging
  (Boost.Log), the Python bindings, coal's own CMake and its `cmake`
  submodule (jrl-cmakemodules), which stays uninitialized. Meshes are built
  from triangle arrays.
- Linked privately into `kinematicskit_core` (the collision world,
  `src/collision.cpp`); nothing in the C ABI names it. Every build of the
  native kit therefore needs the submodule initialized (non-recursively: its
  nested `cmake` submodule is not used). The first build compiles about 70
  coal sources (about a minute).
- Gaps worked around in `src/collision.cpp`: coal answers no distance
  queries on height fields, so the world compares a field's cells as convex
  prisms; without qhull, convex shapes are built from points alone
  (`PointConvex`, no neighbour lists, so support queries scan every point).
- Checked: a standalone build of `native/` runs `tests/cpp/coal_smoke.cpp`
  and `tests/cpp/collision_world.cpp`; its dependency records list no Boost
  or assimp header.

Updating: rebase the `materia` branch onto a new upstream tag, rerun the
Boost check (`grep -rn boost include src` outside mesh_loader, serialization
and python), update the version macros in `materia/include/coal/config.hh`,
bump the submodule and this file, and rerun the standalone C++ tests and
`native/tests`.
