package gearkit;

/** A wave-generator cam seen as the radial displacement `w(phi)` of the flexspline neutral line,
    with `phi` the angle from the major axis. Two-lobe: even about both axes, period pi, zero mean
    (so the tangential displacement closes), `w(0) = +w0` at the major axis. */
interface Cam {
  function name():String;
  /** Major-axis radial displacement amplitude, mm. */
  function w0():Float;
  function w(phi:Float):Float;
  /** dw/dphi, mm per radian. */
  function dw(phi:Float):Float;
  /** Angles in (0, pi/2) where w is only piecewise smooth (arc joins). */
  function breakpoints():Array<Float>;
}
