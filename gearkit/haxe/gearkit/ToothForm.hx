package gearkit;

/** A tooth form: the flank family a gear is cut in. `GearSpec` holds one; everything else
    (rack recovery, hob design, metrology) works from the `ToothProfile` it builds, so no
    code outside the form assumes a flank shape. Implementations: `Involute`. */
interface ToothForm {
  function name():String;
  /** Nominal pressure angle at the pitch point (radians). Gear and rack must agree on it. */
  function pressureAngle():Float;
  /** Height above the pitch circle (below it for an internal gear) before profile shift. */
  function addendum(module:Float):Float;
  /** Depth below the pitch circle (above it for an internal gear) before profile shift. */
  function dedendum(module:Float):Float;
  /** Circular tooth thickness on the pitch circle, including profile shift. */
  function pitchToothThickness(spec:GearSpec):Float;
  /** Throws, naming the parameter, when the form cannot make a valid tooth for this spec. */
  function validate(spec:GearSpec):Void;
  function profile(spec:GearSpec):ToothProfile;
}
