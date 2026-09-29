# Third-party native code

## OpenCV 4.14.0

- Source: <https://github.com/opencv/opencv/archive/refs/tags/4.14.0.tar.gz>
- SHA-256: `ee8fb9b30eb60850431b4656447080e3737b56e45719c92b67f245950609f86e`
- License: Apache-2.0 (OpenCV 4.5 and later).
- Enabled modules: `core`, `imgproc`, `calib3d`, `features2d`, `flann`,
  `objdetect`.
- Excluded modules: `dnn`, `highgui`, `videoio`, `imgcodecs`, `gapi`, `ml`,
  `photo`, `stitching`, Python, Java, and contrib modules.

The native build uses static OpenCV archives. It sets `BUILD_LIST` to the six
enabled modules, disables shared libraries, tests, examples, applications and
language bindings, and disables IPP, ADE, ITT, OpenCL, CUDA, TBB, OpenMP,
GUI and video backends, and image codec dependencies. The OpenCV build is an
`ExternalProject` so its CMake settings stay isolated from Materia.

The shared installation on this host is
`/home/joao/dev/materia-deps/opencv/4.14.0-trimmed-v2`. Later worktrees should
set `VISIONKIT_OPENCV_DIR` to that path. The OpenCV build configuration is
Release with position independent code and the flags in `cmake/OpenCV.cmake`.
The initial configure-to-install build took about 114 seconds on this host.
The final configuration has `WITH_VTK=OFF` and `WITH_OBSENSOR=OFF`; the
installed `lib/` contains exactly the six static module archives listed above.
