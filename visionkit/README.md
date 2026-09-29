# VisionKit

Classical camera geometry for Materia, implemented with a private OpenCV build.
The public C ABI exposes plain values only. Coordinates returned by future
geometry calls use Materia's right-handed frame (+X forward, +Z up), metres,
radians, and `x,y,z,w` quaternions.

Configure the native kit with `cmake -S visionkit/native -B <build-dir>
-DVISIONKIT_OPENCV_DIR=<install-prefix>`, then build and run `ctest` there.
The OpenCV install is shared between worktrees; see `native/THIRD_PARTY.md`.
`VK_OPENCV_THREADS` caps OpenCV internal worker threads and defaults to 1.

## Camera geometry

`vk_camera_model` stores image dimensions, pinhole intrinsics, and either no
distortion or plumb-bob coefficients `k1,k2,p1,p2,k3`. The public API uses
Materia's camera frame: +X forward, +Y left, +Z up. Metre points project to
pixel centres; unprojection returns **unit** rays. OpenCV's camera frame is
converted inside the library. Points behind the camera are rejected.

An undistortion map keeps the input dimensions. `VK_KEEP_ALL_PIXELS` keeps as
much source content as possible and may add black borders; `VK_CROP_VALID`
zooms to valid pixels. Map creation returns the matching distortion-free
pinhole model. Remapping supports RGB8, gray8, and depth32f image views with
explicit byte strides and buffer lengths. RGB and gray use linear sampling;
depth uses nearest-neighbour sampling. Pixels outside the source become zero.
Source and destination buffers must be separate.

`CameraCalibration` serializes as JSON schema version 1, with the camera model,
RMS reprojection error, calibration time, board description, and source.

## Pose and markers

`vk_solve_pnp` accepts object points in a Materia object frame and pixels in
the calibrated image. It returns `camera_T_object` with a quaternion ordered
`x,y,z,w` and a pixel reprojection error for each point. Iterative PnP accepts
general 3D points. IPPE-square accepts exactly four corners of a square in its
local YZ plane, ordered top-left, top-right, bottom-right, bottom-left. In that
marker frame, +X points into the scene, +Y left, and +Z up. IPPE results whose
RMS reprojection error exceeds two pixels are checked against an independent
iterative solve; the lower-error result is returned.

`MarkerDetector` supports ArUco 4×4/50, ArUco 5×5/100, and AprilTag 36h11.
Detection returns marker IDs, four pixel corners, `camera_T_marker`, RMS pixel
reprojection error, and a confidence of `1 / (1 + RMS/2)`. This is a geometric
fit score: 1 means zero error, 0.5 means two pixels RMS. It does not estimate
the probability that a decoded marker ID is correct. Detection is synchronous;
callers should run it off time-critical owner and robotd threads.

RobotKit's `VisionKitFiducialDetector` consumes raw RGB8 `SensorFrame` images
matching a `CameraCalibration`, converts marker poses to RobotKit `Pose3`, and
feeds the existing `FiducialPerception` interface. RobotKit's Haxe dependency
on VisionKit is always available; its native build enables VisionKit with
`RK_BUILD_VISIONKIT=ON` when an application uses the adapter.

## Calibration and deployment

`vk_board_detect` finds chessboard inner corners or ChArUco corners in gray8
or RGB8 images. Chessboard `columns` and `rows` count inner corners; ChArUco
dimensions count squares. IDs are row major. `vk_calibrate` requires at least
five detected views with six corners each, one image size, and limits for
minimum horizontal and vertical image coverage and maximum RMS pixel error.
The coverage limit also requires that fraction of a 4×4 image grid to contain
corners in at least two views, so one unusually wide view cannot compensate
for otherwise clustered images.
Flags can fix the principal point and force tangential distortion to zero.
It returns the camera model, overall RMS, and each view's RMS and
`camera_T_board`. The board frame is +X normal into the scene, +Y left, +Z up.
Its origin is the first inner corner for a chessboard and the upper-left board
edge for ChArUco, following the board's generated coordinate grid.
An ordinary symmetric chessboard cannot distinguish a 180° rotation from its
image alone, so per-view board yaw may flip. Use ChArUco when stable board
orientation matters. The CLI accepts `--dictionary=...` and
`--legacy-pattern` for boards printed with OpenCV's pre-4.6 layout.
Coverage and RMS failures return `VK_ERROR_CALIBRATION_COVERAGE` and
`VK_ERROR_CALIBRATION_RMS`; output values remain unchanged on rejection.

The `visionkit_calibrate` executable reads PNG/JPEG files using vendored
`stb_image` and writes a version 1 `CameraCalibration` JSON file. For example:

```sh
visionkit_calibrate images/ camera.json charuco 8 6 0.04 0.025
visionkit_calibrate images/ camera.json chessboard 7 5 0.04
```

Optional trailing values set minimum coverage (default 0.35 for each axis and
the repeated grid-cell fraction) and
maximum RMS in pixels (default 1.5). For ChArUco, marker size is required.
The tool reports detected corners per file and per-view errors.

RobotKit deployment schema version 5 accepts an optional `cameras` array:

```json
"cameras": [{"sensorId":"sensor/cam", "calibration":"camera.json", "sha256":"<64 lowercase hex digits>"}]
```

The SHA-256 covers the exact calibration file bytes. Each sensor must exist in
the robot model with kind `camera`. The current RobotModel sensor schema has no
image dimensions to compare; calibration dimensions are validated when used
with a `SensorFrame`. Schemas 3 and 4 remain supported without `cameras`.
Calibration references stay within the deployment directory. A camera frame
is optional for intrinsic calibration; a world pose pipeline must require a
mounted frame separately.

The Haxe smoke test can be compiled with
`./haxeon/scripts/haxeon build --compiler-only --project visionkit/tests/haxeon.json`.
Run its `main.hl` with the repository's `haxeon/.tools/hashlink/hl`, with the
native build directory and `haxeon/out` on `LD_LIBRARY_PATH`.
Run native, Haxe smoke, and RobotKit adapter checks together with
`VISIONKIT_OPENCV_DIR=<prefix> visionkit/tests/run-all.sh`.
