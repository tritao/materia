package gearkit;

/** One tooth of one gear, centred on +Y with the right flank at positive X, in the gear's own
    frame: flank, root blend, tip blend and outline, whatever the tooth form. The rack, hob and
    metrology code work only through this interface. Curve parameters run 0..1; flank and blend
    derivatives are with respect to the parameter and oriented as `InvoluteProfile` documents.
    The root blend joins the flank at t=0 and the root circle at t=1; the tip blend joins the
    flank's outer end at t=0 and the tip circle at t=1. */
interface ToothProfile {
  function gearSpec():GearSpec;
  /** Radii where the flank meets the root and tip blends (root first, whatever the gear kind). */
  function rootJoinRadius():Float;
  function tipJoinRadius():Float;
  function flank(t:Float):Point2;
  function flankDerivative(t:Float):Point2;
  /** Exact flank point at a radius between the two joins; outside is rejected. */
  function flankAtRadius(radius:Float):Point2;
  function rootFillet(t:Float):Point2;
  function rootDerivative(t:Float):Point2;
  function tipFillet(t:Float):Point2;
  function tipDerivative(t:Float):Point2;
  function evaluationRange():GearEvaluationRange;
  /** Closed polygon of one pitch, including root and tip lands. Mirror for the left flank. */
  function outline(samples:Int):Array<Point2>;
  /** Half the tooth's angular thickness at `radius` on the flank (radians). */
  function halfAngleAt(radius:Float):Float;
  /** Monotone abscissa along the flank (mm) used for profile traces; roll length for an involute. */
  function profileCoordinate(radius:Float):Float;
  function radiusAtCoordinate(coordinate:Float):Float;
  /** Lever (mm) turning an angular flank error into a flank-normal error; the base radius for an involute. */
  function normalLever(radius:Float):Float;
}
