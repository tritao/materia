package gearkit;

/** Rigid planar placement of a flexspline tooth: rotate by `angle` (counter-clockwise) about the
    origin, then translate by (tx, ty). Maps tooth coordinates (tooth centred on +Y) into the
    circular-spline frame. */
class ToothPose {
  public final angle:Float;
  public final tx:Float;
  public final ty:Float;

  public function new(angle:Float, tx:Float, ty:Float) {
    this.angle = angle;
    this.tx = tx;
    this.ty = ty;
  }

  public function apply(p:Point2):Point2 {
    var c = Math.cos(angle), s = Math.sin(angle);
    return new Point2(c * p.x - s * p.y + tx, s * p.x + c * p.y + ty);
  }
}
