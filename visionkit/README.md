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

The Haxe smoke test can be compiled with
`./haxeon/scripts/haxeon build --compiler-only --project visionkit/tests/haxeon.json`.
Run its `main.hl` with the repository's `haxeon/.tools/hashlink/hl`, with the
native build directory and `haxeon/out` on `LD_LIBRARY_PATH`.
