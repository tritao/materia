package gearkit;

/** Deliberate departures of the flexspline and wave generator from the design state, for the fixed-circular-spline mesh.
    - `camAmplitude`: mm added to the cam's major-axis amplitude w0 (the cam shape is rebuilt at the new amplitude);
    - `toothRotation`: a uniform rigid rotation of every flexspline tooth about the axis, radians, positive in the
      drive direction (the direction the flexspline turns under the drive torque);
    - `offsets`: translations of the whole tooth ring (see `FlexOffset`). */
class MeshErrors {
  public final camAmplitude:Float;
  public final toothRotation:Float;
  public final offsets:Array<FlexOffset>;

  public function new(camAmplitude:Float = 0, toothRotation:Float = 0, ?offsets:Array<FlexOffset>) {
    if (!Math.isFinite(camAmplitude) || !Math.isFinite(toothRotation)) throw "mesh errors must be finite";
    this.camAmplitude = camAmplitude;
    this.toothRotation = toothRotation;
    this.offsets = offsets == null ? [] : offsets;
  }

  public static function none():MeshErrors return new MeshErrors();
}
