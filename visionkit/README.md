# VisionKit

Classical camera geometry for Materia, implemented with a private OpenCV build.
The public C ABI exposes plain values only. Coordinates returned by future
geometry calls use Materia's right-handed frame (+X forward, +Z up), metres,
radians, and `x,y,z,w` quaternions.

Configure the native kit with `cmake -S visionkit/native -B <build-dir>
-DVISIONKIT_OPENCV_DIR=<install-prefix>`, then build and run `ctest` there.
The OpenCV install is shared between worktrees; see `native/THIRD_PARTY.md`.
`VK_OPENCV_THREADS` caps OpenCV internal worker threads and defaults to 1.

The Haxe smoke test can be compiled with
`./haxeon/scripts/haxeon build --compiler-only --project visionkit/tests/haxeon.json`.
Run its `main.hl` with the repository's `haxeon/.tools/hashlink/hl`, with the
native build directory and `haxeon/out` on `LD_LIBRARY_PATH`.
