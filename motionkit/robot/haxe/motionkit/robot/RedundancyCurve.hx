package motionkit.robot;

/** Smooth redundancy geometry shared by refinement and differentiation. */
interface RedundancyCurve {
  public function evaluate(distance:Float):motionkit.robot.RedundancySpline.SplineSample;
}
