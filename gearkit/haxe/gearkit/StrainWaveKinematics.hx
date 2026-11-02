package gearkit;

/** Where each flexspline tooth is, in the circular-spline frame, as the wave generator turns
    (TOOTH_DESIGN_PLAN.md TD-D1). Angles are counter-clockwise from +Y, the direction a tooth
    points at rest. The circular spline is fixed. Each tooth is rigid and anchored to its
    neutral-line section: the point (0, rn) of the tooth frame sits on the deformed neutral line. */
class StrainWaveKinematics {
  public final flexTeeth:Int;
  public final circularTeeth:Int;
  public final deformation:WaveDeformation;

  public function new(flexTeeth:Int, circularTeeth:Int, deformation:WaveDeformation) {
    if (circularTeeth - flexTeeth != 2) throw "two-lobe kinematics need exactly two more circular-spline teeth";
    this.flexTeeth = flexTeeth;
    this.circularTeeth = circularTeeth;
    this.deformation = deformation;
  }

  /** Mean rotation of the flexspline when the wave generator is at `theta`: eta = -2 theta / Zf. */
  public function meanRotation(theta:Float):Float return -(circularTeeth - flexTeeth) * theta / flexTeeth;

  /** Section angle from the major axis of tooth `j` with the wave generator at `theta`. */
  public function sectionAngle(j:Int, theta:Float):Float return 2 * Math.PI * j / flexTeeth + meanRotation(theta) - theta;

  /** Pose of a tooth whose section lies at `delta` from the major axis, with the major axis at `theta`. */
  public function poseAt(theta:Float, delta:Float):ToothPose {
    var d = deformation, rn = d.neutralRadius, mu = d.mu(delta);
    var c = Math.cos(mu), s = Math.sin(mu);
    // Local frame with the tooth on +Y: neutral point (-v, rn + w), then pivot the tooth about (0, rn) by mu.
    var lx = -d.v(delta) - (-s * rn), ly = rn + d.w(delta) - c * rn;
    var alpha = theta + delta, ca = Math.cos(alpha), sa = Math.sin(alpha);
    return new ToothPose(alpha + mu, ca * lx - sa * ly, sa * lx + ca * ly);
  }

  public function pose(j:Int, theta:Float):ToothPose return poseAt(theta, sectionAngle(j, theta));
}
