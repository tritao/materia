# Future VisionKit work

- Features, optical flow, and visual odometry.
- Stereo matching and rectification.
- Fisheye and omnidirectional camera models.
- Camera capture in SensorKit or robotd.
- JPEG decoding for perception inputs.
- OpenCV dnn, CUDA, and OpenCL.
- Hand-eye and multi-camera extrinsic calibration.
- When the perception phase 4 branch is integrated, combine its `perception`
  parser with schema v5 `cameras` in `SerialDeployment` and share one strict
  key validator. Perception's mounted-frame requirement belongs to pipelines
  that need a world pose; intrinsic calibration alone does not require one.
- Add nominal image dimensions to camera sensors in RobotModel, then validate
  deployment calibration dimensions against them.
- When a new shared OpenCV install is needed, build it with the final opt-out
  flags and name the prefix by a configure-flag digest. The current shared
  `trimmed-v2` install has Eigen and LAPACK enabled.
- Evaluate pose ambiguity for noisy nearly front-facing planar markers; if
  needed, use a temporal pose prior to choose between valid planar solutions.
