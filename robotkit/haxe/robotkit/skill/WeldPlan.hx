package robotkit.skill;

import robotkit.spatial.Transform3;

/**
 * How one seam is welded: the arc (wire speed in metres per minute, voltage in volts), the travel speed along the seam
 * in metres per second, and the sequence around it. The torch comes in from `approach` metres out along the wire, strikes
 * the arc at the start and holds until it is established, dwells `startDwell` seconds, travels the seam, dwells
 * `craterDwell` seconds at its end to fill the crater, stops the wire, lets the arc burn back for `burnback` seconds
 * and goes out again.
 */
typedef WeldParameters = {
  var wireSpeed:Float;
  var voltage:Float;
  var travelSpeed:Float;
  var approach:Float;
  var startDwell:Float;
  var craterDwell:Float;
  var burnback:Float;
}

/**
 * A straight seam to weld: the poses of the wire tip at its two ends (+Z along the wire out of the torch, +X along the
 * direction of travel) in one frame, and the parameters. The torch holds its orientation from `start` to `stop`.
 */
class WeldPlan {
  public final start:Transform3;
  public final stop:Transform3;
  public final parameters:WeldParameters;

  public function new(start:Transform3, stop:Transform3, parameters:WeldParameters) {
    if (start == null || stop == null || parameters == null) throw "A weld plan needs a start, a stop and parameters";
    if (!(stop.translation.sub(start.translation).norm() > 1e-6)) throw "A weld plan needs a seam with a length";
    if (!(parameters.wireSpeed > 0) || !(parameters.voltage > 0) || !(parameters.travelSpeed > 0) || !(parameters.approach > 0) ||
        !(parameters.startDwell >= 0) || !(parameters.craterDwell >= 0) || !(parameters.burnback >= 0))
      throw "A weld plan needs positive wire speed, voltage, travel speed and approach, and dwells of zero or more";
    this.start = start;
    this.stop = stop;
    this.parameters = parameters;
  }

  public function length():Float return stop.translation.sub(start.translation).norm();

  /** The plan in the frame `frame_T_this` leads to: its poses become `frame_T_this * pose`. */
  public function transformed(frame_T_this:Transform3):WeldPlan
    return new WeldPlan(frame_T_this.compose(start), frame_T_this.compose(stop), parameters);
}
