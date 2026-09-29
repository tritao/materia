# VisionKit (classical computer vision on OpenCV): plan

Status: plan, not started. Audience: the agent implementing it.

## Purpose

VisionKit is Materia's classical computer vision layer: camera models,
calibration, undistortion, pose from points and fiducial markers. OpenCV does
the work inside. None of its types or concepts cross the kit's boundary.

It complements the other layers without overlapping them:
- **SensorKit** owns what a sensor is and how it's simulated.
- **RobotKit `inference`** runs learned models through ONNX Runtime.
- **RobotKit `perception`** turns frames into semantic observations.

VisionKit supplies the geometry that perception needs to turn pixels into
metric poses.

## Decisions (do not re-litigate)

- **A new top-level kit,** `visionkit/`, shaped like `motionkit/`:
  - `native/{include,src,bindings,tests,tools}`;
  - a `THIRD_PARTY.md`;
  - `haxe/` and `haxeon.json`.

  RobotKit consumes it the way it consumes MotionKit: CMake `add_subdirectory`,
  plus a haxeon dependency with `.hxi` FFI bindings.
- **Dependency direction.** RobotKit depends on VisionKit, never the reverse.
  VisionKit's Haxe API uses its own plain types (image views, camera models,
  poses as arrays or small classes). Adapters that produce RobotKit types, such
  as `FiducialDetector` implementations, live in RobotKit.
- **The C ABI hides OpenCV.** No `cv::Mat` or other OpenCV type or header
  appears in the public headers.
  - Images cross the ABI as an image view: width, height, stride in bytes,
    pixel format, and a byte pointer.
  - Results are plain structs with `struct_size` fields and opaque handles, in
    the style of `robotkit/policy/include/robotkit_policy.h`.
- **Coordinate conventions.** OpenCV's camera frame is +Z forward, +X right,
  +Y down. Materia's (`robotkit/ARCHITECTURE.md`, around lines 71-80) is
  right-handed, +X forward, +Z up.
  - Convert exactly once, inside VisionKit, at the ABI boundary.
  - Everything VisionKit returns uses Materia's convention, with quaternions
    ordered `x,y,z,w` and transforms named `A_T_B`.
  - Units are metres and radians.
- **A trimmed OpenCV build.** Pin the latest stable OpenCV 4.x release (4.5+ is
  Apache-2.0) and record the version and hash.
  - Include only `core`, `imgproc`, `calib3d`, `features2d`, `flann` (if
    `calib3d` or `features2d` requires it) and `objdetect` (ArUco and AprilTag
    dictionaries).
  - Exclude `dnn` (ONNX Runtime covers inference), `highgui`, `videoio`,
    `imgcodecs`, `gapi`, `ml`, `photo`, `stitching`, Python and Java, and
    `opencv_contrib`.
  - Build with `WITH_IPP`, `WITH_ADE` and anything else that downloads during
    configure turned off. Also turn off every GUI and video backend, and
    OpenCL/CUDA.
  - Link it statically into `visionkit_core`.
- **One shared OpenCV build.** Building OpenCV is slow, like OCCT, and must not
  be repeated per worktree or agent.
  - `VISIONKIT_OPENCV_DIR` (a CMake variable or environment variable) points to
    an installed build to use.
  - If it's unset, the build fetches the pinned source tarball (with SHA-256)
    and builds it through `ExternalProject` in an isolated tree, so OpenCV's
    CMake globals don't leak into Materia's.
  - Build it once into a shared location outside any worktree, for example
    `/home/joao/dev/materia-deps/opencv/<version>-<config-hash>`. Document it
    so later worktrees and agents set `VISIONKIT_OPENCV_DIR` to it instead of
    rebuilding.
- **Image decoding** in tools uses the already-vendored `stb_image`
  (`animkit/native/vendor/stb`), not OpenCV `imgcodecs`.
- **Threading.** VisionKit functions are synchronous and never spawn threads,
  except OpenCV's internal parallelism, which is capped by an option (default
  1). Callers run heavy work (remap, calibration, fiducial detection on large
  frames) off the RobotKit owner thread and the `robotd` main loop, the same
  rule as inference.
- **Calibration is deployment data.** A physical camera's calibration is
  referenced from `deployment.json` by `sensorId`, like `DeviceLayout`. It is
  not part of the robot model. The model may carry nominal intrinsics for
  simulated cameras.

## Relevant existing code (verify before building on it)

- **`motionkit/`** is the structural template: `native/CMakeLists.txt`,
  `native/THIRD_PARTY.md`, `native/bindings/motionkit.hxi`, `haxeon.json`
  (`ffi.interfaces`). RobotKit pulls it in at `robotkit/CMakeLists.txt` (~35)
  and `robotkit/robotd/native/CMakeLists.txt` (~25-45).
- **`robotkit/policy/`** shows the C ABI style (opaque handles, `struct_size`,
  `RK_API`, annotations) and the `.hxi` regeneration script (`tools/check-hxi.sh`
  using `haxeon-ffi-audit`).
- **`robotkit/policy/cmake/OnnxRuntime.cmake`** is the pattern for a pinned
  dependency with a hash check and an override directory.
- **`robotkit/haxe/robotkit/perception/`:**
  - `PinholeCameraIntrinsics` has fx, fy, cx and cy for a rectified image. It
    has no distortion and no image size.
  - `FiducialDetector` is an interface, `detect(SensorFrame)` returning
    `Array<FiducialMarkerObservation>`. Its only implementation is a test double
    in `robotkit/tests/src/tests/RobotWorldTests.hx`.
  - `FiducialMarkerObservation` holds a marker id, a planar `Pose2`, an optional
    `Pose3` in the camera frame, and a confidence.
  - `FiducialPerception` and `FiducialTargetConfig` consume them.
- **Images.** `robotkit/haxe/robotkit/world/CameraImage.hx` supports `rgb8`,
  `depth32f` and `jpeg`.
- **Related plan.** `robotkit/plans/PERCEPTION.md` (in the `perception`
  worktree) defines `ImageDetection`, `ImageDetectionObservation` and deployment
  schema v5. VisionKit phase V4 builds on its phase 4.

## Phases

Each phase is its own commit, with tests passing. Stop after each phase and
report what changed, decisions made, deviations and test results.

### V0: kit skeleton and OpenCV build

1. Create `visionkit/` with the MotionKit layout: `native/CMakeLists.txt`
   (project `visionkit_core`, alias `VisionKit::core`, option `VK_BUILD_TESTS`),
   `native/cmake/OpenCV.cmake`, `THIRD_PARTY.md`, `README.md`, `haxeon.json` and
   `haxe/`.
2. `OpenCV.cmake` implements the pinned build described in the decisions:
   - honor `VISIONKIT_OPENCV_DIR`;
   - otherwise use `ExternalProject` from a hashed tarball, with the trimmed
     module list and all downloads disabled;
   - produce imported static targets.
   - Add a CMake check that fails if an excluded module (`dnn`, `highgui`,
     `videoio`, `imgcodecs`) is found in the build.
3. Add a minimal ABI (`visionkit.h`) with `vk_version()` and a result-code enum
   following RobotKit's `rk_result` conventions, plus a native smoke test.
4. Add Haxe bindings: `native/bindings/visionkit.hxi` generated by a
   `tools/check-hxi.sh` modelled on `robotkit/policy/tools/check-hxi.sh`,
   registered in `visionkit/haxeon.json`. Add a Haxe smoke test.
5. Build OpenCV once into the shared deps location. Record the exact path,
   configure flags and build time in `THIRD_PARTY.md` and in the phase summary.
6. `THIRD_PARTY.md` records: OpenCV's version, source URL and hash, the
   Apache-2.0 license, the enabled and disabled modules, and the flags that
   disable downloads.

### V1: camera model, undistortion, projection

ABI and Haxe API:

- **`vk_camera_model`** (plain struct):
  - image width and height;
  - fx, fy, cx, cy;
  - distortion model: `none`, or `plumb_bob` (k1, k2, p1, p2, k3). Fisheye is
    out of scope.
- **Projection:** `vk_project_points` and `vk_unproject_points`. Unprojection
  undistorts pixel coordinates into normalized camera rays, returned in
  Materia's camera convention.
- **Undistortion maps:** `vk_undistort_map_create(model, output_model_policy)`
  returns an opaque map. The policy is keep-all-pixels or crop-valid, and the
  map also reports the rectified pinhole model.
- **Remap:** `vk_undistort_image(map, src_view, dst_view)` for `rgb8`, `gray8`
  and `depth32f`. Depth uses nearest-neighbour sampling, never interpolation
  across depth edges.
- **Haxe side:**
  - `visionkit.CameraModel` and `visionkit.ImageView` wrappers.
  - `CameraCalibration` holds a camera model plus metadata: RMS reprojection
    error, calibration time, board description and source.
  - JSON encode and decode for `CameraCalibration`, versioned.

Tests:
- Project and unproject round-trip, with and without distortion.
- Undistort a synthetic distorted grid and check straight lines.
- Check the convention mapping: a point straight ahead of the camera maps to
  +X.
- Stride handling, and invalid arguments.

### V2: pose from points and fiducials

- **`vk_solve_pnp`.** Takes object points (metres, object frame) and image
  points, with methods iterative and IPPE-square. It returns `camera_T_object`
  in Materia's convention and a per-point reprojection error.
- **Marker detection.** `vk_marker_detector_create(dictionary, params)` supports
  ArUco dictionaries and the AprilTag dictionaries OpenCV's `objdetect`
  provides (`APRILTAG_36h11` at minimum).
  - `vk_marker_detect(detector, gray_or_rgb_view, camera_model, marker_size_m,
    out)` returns, for each marker: id, corners in pixels, `camera_T_marker`,
    reprojection error, and a confidence score. Define and document how the
    confidence score is computed.
- **RobotKit adapter** (in RobotKit, not VisionKit). `robotkit.perception`
  gains `VisionKitFiducialDetector implements FiducialDetector`.
  - It takes a `CameraCalibration`, a dictionary and a marker size.
  - It produces `FiducialMarkerObservation.fromPose3(...)` values in the
    camera's frame.
  - Wire RobotKit to VisionKit through `robotkit/haxeon.json` dependencies and
    `robotkit/CMakeLists.txt`, the same way as MotionKit.
  - Decide whether the dependency is optional (as `RK_BUILD_POLICY` is for
    ONNX Runtime) and follow that precedent. Check how the policy bindings
    behave when the native library isn't built before choosing.

Tests:
- Synthesize a marker image (OpenCV marker generation inside the native test),
  warp it with a known pose through a known camera, detect it and recover the
  pose within tolerance.
- Several markers, a partially hidden marker, and a wrong dictionary.
- A RobotKit test feeds a synthetic `SensorFrame` through
  `VisionKitFiducialDetector` into the existing `FiducialPerception`.

### V3: calibration

- **Board detection.** `vk_board_detect` handles chessboard and ChArUco boards,
  with sub-pixel corner refinement.
- **Calibration.** `vk_calibrate(views…, board, flags)` returns a
  `vk_camera_model`, RMS error, per-view error and per-view `camera_T_board`.
  Reject calibrations whose coverage or RMS error fall outside configurable
  limits, with a specific reason.
- **CLI tool.** `visionkit/native/tools/calibrate`:
  - reads PNG or JPEG images from a directory (decoded with `stb_image`);
  - writes a `CameraCalibration` JSON file;
  - prints a per-view report.
- **Deployment.** Add a `cameras` section to `deployment.json` mapping
  `sensorId` to a calibration file path and SHA-256. Parse and validate it in
  `SerialDeployment`:
  - the sensor must exist and be a camera;
  - the image size must match the sensor's declared size, if the model declares
    one.

  Coordinate the schema version with perception phase 4d (v5). If both land on
  the same unreleased branch, share v5 rather than bumping twice.

Tests:
- Render synthetic chessboard and ChArUco views from known poses through a known
  camera with distortion. Calibrate them and recover the parameters within
  tolerance.
- The rejection paths.
- A CLI run on a fixture directory.
- Deployment fixtures, both valid and invalid.

### V4: metric perception in RobotKit

This phase depends on perception plan phase 4 (`ImageDetection`).

- **Undistortion before detection.** Add an optional step to
  `ObjectDetectorPipeline`: when a calibration is configured, undistort the
  boxes' corner points, not the whole image, with `vk_unproject_points`. Make
  whole-image undistortion an option for models trained on rectified images.
- **Lifting to `Detection`.** Add `ImageDetectionLifter`, a synchronous
  `Perception` adapter that turns `ImageDetection` into planar `Detection`
  using one of:
  - an aligned depth frame: take the median depth inside the box and unproject;
  - a ground-plane assumption: intersect the ray through the bottom-centre of
    the box with the floor plane, given the camera mount transform.

  The results feed `FrameAwarePerception` unchanged. Carry the timestamps and
  clock IDs through unchanged.

Tests:
- Synthetic detections with known depth or geometry lift to known positions.
- A mismatch between the depth frame's and the camera's sequence or timestamps
  is rejected, rather than silently pairing frames from different times.

## Out of scope (list in `visionkit/TODO.md`)

- Features and optical flow, and visual odometry (ORB, KLT) for localization.
- Stereo matching and rectification.
- The fisheye and omnidirectional camera models.
- Camera capture (V4L2, libcamera), which belongs to SensorKit or the `robotd`
  sensor producers.
- JPEG decode for perception inputs (`stb_image` or libjpeg-turbo, separately).
- OpenCV `dnn`, CUDA and OpenCL.
- Hand-eye calibration (camera to arm or base), and multi-camera extrinsic
  calibration.

## Working rules

- **Worktree.** Work in a dedicated git worktree, never in the shared checkout
  at `/home/joao/dev/materia`. If the worktree lacks submodules or `haxeon`
  artifacts, clone them from the main checkout at the pinned commits and copy
  `haxeon/.tools` and `haxeon/out/haxeon_runtime.hdll`.
- **OpenCV builds.** Never rebuild OpenCV once the shared build exists. Set
  `VISIONKIT_OPENCV_DIR` instead.
- **OCCT and CadKit.** Never rebuild OCCT or CadKit. Use
  `CADKIT_BUILD_ROOT=/home/joao/dev/materia/cadkit/build/debug` and
  `CADKIT_OCCT_DIR` if anything needs them.
- **haxeon gaps.** If haxeon lacks a feature or stdlib member, fix haxeon rather
  than working around it. Known quirks: no class-to-anonymous structural
  subtyping (use an interface), a reduced `Math`, and no static-function
  imports.
- **Diagnose first.** When a tool or compiler error is unclear, read its source
  and confirm the root cause before working around it.
- **Verify OpenCV behavior.** Check behavior (API signatures in the pinned
  version, the camera convention, AprilTag dictionary availability, ChArUco API
  changes across 4.x) in the pinned OpenCV source before relying on it.
- **Follow-ups** go in `visionkit/TODO.md` (or `robotkit/TODO.md` for RobotKit
  adapters), not the root `TODO.md`.

## Done when

- **V0.** VisionKit builds against the shared OpenCV. The excluded-module check
  passes. The native and Haxe smoke tests pass. `THIRD_PARTY.md` is complete.
- **V1.** Projection, undistortion and convention tests pass. `CameraCalibration`
  round-trips through JSON.
- **V2.** Synthetic marker poses are recovered within tolerance, and
  `VisionKitFiducialDetector` works through `FiducialPerception`.
- **V3.** Synthetic calibrations recover known parameters. The CLI works on
  fixtures. The deployment `cameras` section validates.
- **V4.** Lifted detections match their known positions, and mismatched
  depth/camera frames are rejected.
- **Every phase.** The relevant test runners pass (`visionkit` ctest, the Haxe
  tests, and `robotkit/tests/run-all.sh` once RobotKit depends on VisionKit),
  and `ARCHITECTURE.md` or `README.md` explain the kit's boundary and conventions.
