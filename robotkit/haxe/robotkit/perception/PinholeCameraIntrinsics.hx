package robotkit.perception;

/** Calibration for a rectified pinhole depth image, in pixel units. */
class PinholeCameraIntrinsics {
  public final focalLengthXPixels:Float;
  public final focalLengthYPixels:Float;
  public final principalPointXPixels:Float;
  public final principalPointYPixels:Float;

  public function new(focalLengthXPixels:Float, focalLengthYPixels:Float,
      principalPointXPixels:Float, principalPointYPixels:Float) {
    if (!Math.isFinite(focalLengthXPixels) || focalLengthXPixels <= 0.0 ||
        !Math.isFinite(focalLengthYPixels) || focalLengthYPixels <= 0.0 ||
        !Math.isFinite(principalPointXPixels) || principalPointXPixels < 0.0 ||
        !Math.isFinite(principalPointYPixels) || principalPointYPixels < 0.0)
      throw "Pinhole camera calibration requires positive focal lengths and finite non-negative principal points";
    this.focalLengthXPixels = focalLengthXPixels;
    this.focalLengthYPixels = focalLengthYPixels;
    this.principalPointXPixels = principalPointXPixels;
    this.principalPointYPixels = principalPointYPixels;
  }
}
