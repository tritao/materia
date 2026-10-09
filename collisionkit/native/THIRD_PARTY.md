# Third-party code in collisionkit/native

## coal (collision and distance)

Moved here from `kinematicskit/native/vendor/coal` with COLLISION.md CL2.

- Source: our fork https://github.com/tritao/coal, branch `materia`, pinned
  submodule at `vendor/coal`. Based on upstream coal-library/coal tag v3.0.4
  (commit f0fecd0a); the branch adds two commits: f92f288b (the Materia
  build) and 85cb6397 (height-field distance, COLLISION.md CL-D13; meant
  for upstream, not yet pushed or proposed).
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
- Gaps: upstream coal answers no distance queries on height fields; our
  branch implements them (85cb6397, with `test/hfield_distance.cpp`).
  Worked around in `src/world.cpp`: without qhull, convex shapes are built from points alone (`PointConvex`,
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

## V-HACD 4 (convex decomposition)

- Source: upstream https://github.com/kmammou/v-hacd, pinned submodule at
  `vendor/v-hacd`, tag v4.1.0 (commit 22ec20a7). No fork (COLLISION.md
  CL-D11); upstream is archived, which only means frozen.
- Licence: BSD 3-Clause, Copyright (c) 2011 Khaled Mamou
  (`vendor/v-hacd/LICENSE`).
- Use: the single header `include/VHACD.h`, compiled once in
  `src/decompose.cpp` (`ENABLE_VHACD_IMPLEMENTATION`), synchronously
  (`m_asyncACD` off). It needs only the C++ standard library and threads
  (`Threads::Threads`). The `app/` and `doc/` directories are not used.
- Enclosure: V-HACD does not promise that its pieces enclose the input;
  `src/decompose.cpp` measures how far the mesh sticks out of the pieces'
  union (exact point-to-polytope distances at samples no farther apart
  than a spacing) and reports the inflation that encloses it.

Updating: bump the submodule to a new tag, update this file, and rerun the
standalone C++ tests (`ck_decomposition`) and the cell tests
(`robotkit/cadbridge/tests/collision`).
