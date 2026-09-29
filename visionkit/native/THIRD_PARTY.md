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
VisionKit links system zlib alongside the static OpenCV archives.

## Reproducible build and bundled licenses

A new build installs OpenCV below its own CMake binary directory. A shared
install is used only when `VISIONKIT_OPENCV_DIR` is set. The build sets
`CMAKE_BUILD_TYPE=Release`, `CMAKE_POSITION_INDEPENDENT_CODE=ON`,
`CMAKE_INSTALL_LIBDIR=lib`, `BUILD_SHARED_LIBS=OFF`, and
`BUILD_LIST=core,imgproc,calib3d,features2d,flann,objdetect`.

These options are explicitly `OFF`: `BUILD_TESTS`, `BUILD_PERF_TESTS`,
`BUILD_EXAMPLES`, `BUILD_DOCS`, `BUILD_opencv_apps`, `BUILD_JAVA`,
`BUILD_opencv_python2`, `BUILD_opencv_python3`, `WITH_IPP`, `WITH_ADE`,
`WITH_ITT`, `WITH_OPENCL`, `WITH_CUDA`, `WITH_TBB`, `WITH_OPENMP`,
`WITH_GTK`, `WITH_QT`, `WITH_VTK`, `WITH_V4L`, `WITH_FFMPEG`,
`WITH_GSTREAMER`, `WITH_OBSENSOR`, `WITH_JPEG`, `WITH_PNG`, `WITH_WEBP`,
`WITH_TIFF`, `WITH_OPENEXR`, `WITH_PROTOBUF`, `WITH_FLATBUFFERS`,
`WITH_EIGEN`, `WITH_LAPACK`, `WITH_CAROTENE`, and `WITH_KLEIDICV`.
The excluded-module check runs after installation for fresh builds.

The existing shared `4.14.0-trimmed-v2` prefix was built before the final
Eigen/LAPACK opt-outs: its cache has both enabled and its exported targets
reference Eigen. It remains usable on this host, but it is not the reference
for a new portable build. The earlier `trimmed-v1` build enabled VTK and is
obsolete. Future shared prefixes should be identified by a digest of the
complete configure flags, rather than a manually incremented suffix.

OpenCV also contains Berkeley SoftFloat (BSD 3-Clause; `modules/core/3rdparty/SoftFloat/COPYING.txt`)
and KAZE and AKAZE (BSD-style licenses; `modules/features2d/src/kaze/LICENSE.KAZE`
and `LICENSE.AKAZE`). The existing shared prefix uses Eigen (MPL 2.0,
with some files offered under LGPL 2.1 or MPL 2.0;
`/usr/share/doc/libeigen3-dev/copyright` on this host). The CLI uses the
vendored stb image decoder from `animkit/native/vendor/stb` under its MIT
option (`LICENSE`).
