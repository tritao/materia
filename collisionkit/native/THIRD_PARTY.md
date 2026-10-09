# Third-party code in collisionkit/native

## coal (collision and distance)

Moved here from `kinematicskit/native/vendor/coal` with COLLISION.md CL2.

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
- Linked privately into `collisionkit_core` (the collision world,
  `src/world.cpp`); nothing in the C ABI names it. Only packages that use
  collisionkit-native compile it (about 70 sources, a minute on a first
  build). The submodule is initialized non-recursively: its nested `cmake`
  submodule is not used.
- Gaps worked around in `src/world.cpp`: coal answers no distance queries
  on height fields, so the world compares a field's cells as convex prisms;
  without qhull, convex shapes are built from points alone (`PointConvex`,
  no neighbour lists, so support queries scan every point); coal clamps
  heights at a field's minimum, so the world refuses heights below it.
- Checked: a standalone build of `native/` runs `tests/cpp/coal_smoke.cpp`
  and `tests/cpp/collision_world.cpp`; its dependency records list no Boost
  or assimp header.

Updating: rebase the `materia` branch onto a new upstream tag, rerun the
Boost check (`grep -rn boost include src` outside mesh_loader, serialization
and python), update the version macros in `materia/include/coal/config.hh`,
bump the submodule and this file, and rerun the standalone C++ tests and
`native/tests`.
