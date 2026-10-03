package robotkit.skill;

import robotkit.spatial.Transform3;

/**
 * How a weld is run: the arc (wire speed in metres per minute, voltage in volts), the travel speed along the path
 * in metres per second, and the sequence around it. The torch comes in from `approach` metres out along the wire, strikes
 * the arc at the start and holds until it is established, dwells `startDwell` seconds, travels the path, dwells
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
 * One straight piece of a weld path: the poses of the wire tip at its two ends (+Z along the wire out of the torch, +X
 * along the direction of travel), in the plan's frame. The torch holds its orientation from `start` to `stop`.
 */
class WeldSegment {
  public final start:Transform3;
  public final stop:Transform3;

  public function new(start:Transform3, stop:Transform3) {
    if (start == null || stop == null) throw "A weld segment needs a start and a stop";
    if (!(stop.translation.sub(start.translation).norm() > 1e-6)) throw "A weld segment needs a length";
    this.start = start;
    this.stop = stop;
  }

  public function length():Float return stop.translation.sub(start.translation).norm();
}

/**
 * A weld to run: a path of segments, each starting where the one before ends (a straight seam is one segment, the four
 * sides of a tube four), and the parameters. The arc is struck at the start of the first segment, stays up along the
 * whole path and ends at the end of the last; where one segment's orientation differs from the next one's the torch
 * turns between them as it passes the corner.
 */
class WeldPlan {
  /** Positions of consecutive segments this close are joined, in metres. */
  public static inline var JOIN:Float = 1e-5;
  /**
   * The least time the arc burns back with the wire stopped before it is switched off, in seconds. With none, wire and
   * arc would stop at the same instant and the wire would stick in the pool (the arc model does not flag that, a real
   * supply's burnback does it).
   */
  public static inline var MIN_BURNBACK:Float = 0.05;

  public final segments:Array<WeldSegment>;
  public final parameters:WeldParameters;

  public function new(segments:Array<WeldSegment>, parameters:WeldParameters) {
    if (segments == null || segments.length == 0 || parameters == null) throw "A weld plan needs segments and parameters";
    for (index in 1...segments.length)
      if (segments[index].start.translation.sub(segments[index - 1].stop.translation).norm() > JOIN)
        throw 'Weld segment $index does not start where the one before it ends';
    if (!(parameters.wireSpeed > 0) || !(parameters.voltage > 0) || !(parameters.travelSpeed > 0) || !(parameters.approach > 0) ||
        !(parameters.startDwell >= 0) || !(parameters.craterDwell >= 0) || !(parameters.burnback >= MIN_BURNBACK))
      throw 'A weld plan needs positive wire speed, voltage, travel speed and approach, dwells of zero or more, and a burnback of at least $MIN_BURNBACK s';
    this.segments = segments.copy();
    this.parameters = parameters;
  }

  /** A plan for one straight seam from `start` to `stop`. */
  public static function straight(start:Transform3, stop:Transform3, parameters:WeldParameters):WeldPlan
    return new WeldPlan([new WeldSegment(start, stop)], parameters);

  /** Where the path starts and ends. */
  public function start():Transform3 return segments[0].start;
  public function stop():Transform3 return segments[segments.length - 1].stop;

  /** The length of the segments together. */
  public function length():Float {
    var sum = 0.0;
    for (segment in segments) sum += segment.length();
    return sum;
  }

  /** The plan in the frame `frame_T_this` leads to: its poses become `frame_T_this * pose`. */
  public function transformed(frame_T_this:Transform3):WeldPlan
    return new WeldPlan([for (segment in segments) new WeldSegment(frame_T_this.compose(segment.start), frame_T_this.compose(segment.stop))],
      parameters);
}
