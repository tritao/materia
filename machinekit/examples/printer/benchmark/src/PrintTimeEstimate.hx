import toolpathkit.path.GeometryTools;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;

private typedef Segment = {
  var length:Float;
  var ux:Float;
  var uy:Float;
  var uz:Float;
  /** Fastest the segment may go: its feed, and each axis's speed limit along its direction. */
  var speed:Float;
  /** Acceleration along it: the slowest axis's limit divided by that axis's share of the direction. */
  var acceleration:Float;
}

/**
 * The shortest time a firmware with a look-ahead planner would take for a program's moves: straight segments (arcs
 * as fine chords) with trapezoidal speed profiles, junction speeds from Marlin's junction deviation, and a forward
 * and a backward pass for the speeds the accelerations allow. It is a lower bound for our planner: Marlin's buffer
 * is finite and its step generator has its own limits, ours has neither a finite buffer nor jerk-free corners.
 * Metres, seconds. Extruder-only moves, heating and fans take no time here.
 */
class PrintTimeEstimate {
  /** Marlin's default `JUNCTION_DEVIATION_MM`. */
  public static inline var JUNCTION_DEVIATION:Float = 0.013e-3;
  /** The most an arc may stray from its chords. */
  static inline var CHORD_SAGITTA:Float = 0.01e-3;

  /** Segments of the program's moves: their count, and in `seconds` the time. */
  public static function estimate(program:ToolpathProgram, axisSpeed:Array<Float>, axisAcceleration:Array<Float>,
      ?junctionDeviation:Float = JUNCTION_DEVIATION):{seconds:Float, segments:Int} {
    var segments = segmentsOf(program, axisSpeed, axisAcceleration);
    var n = segments.length;
    if (n == 0) return {seconds: 0.0, segments: 0};
    // junction[i] is the speed between segment i-1 and i; the first and last are rest.
    var junction = [for (_ in 0...(n + 1)) 0.0];
    for (i in 1...n) {
      var before = segments[i - 1], after = segments[i];
      var cosTheta = Math.max(-1.0, Math.min(1.0, -(before.ux * after.ux + before.uy * after.uy + before.uz * after.uz)));
      var limit = Math.min(before.speed, after.speed);
      if (cosTheta > -0.999999) {
        var sinHalf = Math.sqrt(0.5 * (1.0 - cosTheta));
        var acceleration = Math.min(before.acceleration, after.acceleration);
        limit = Math.min(limit, Math.sqrt(acceleration * junctionDeviation * sinHalf / (1.0 - sinHalf)));
      }
      junction[i] = limit;
    }
    var i = n - 1;
    while (i >= 0) {
      junction[i] = Math.min(junction[i], Math.sqrt(junction[i + 1] * junction[i + 1] + 2.0 * segments[i].acceleration * segments[i].length));
      i--;
    }
    for (i in 0...n)
      junction[i + 1] = Math.min(junction[i + 1], Math.sqrt(junction[i] * junction[i] + 2.0 * segments[i].acceleration * segments[i].length));
    var seconds = 0.0;
    for (i in 0...n) {
      var segment = segments[i], entry = junction[i], exit = junction[i + 1], a = segment.acceleration;
      var peak = Math.min(segment.speed, Math.sqrt((2.0 * a * segment.length + entry * entry + exit * exit) / 2.0));
      peak = Math.max(peak, Math.max(entry, exit));
      var accelerating = (peak * peak - entry * entry) / (2.0 * a), braking = (peak * peak - exit * exit) / (2.0 * a);
      var cruise = Math.max(0.0, segment.length - accelerating - braking);
      var taken = (peak - entry) / a + (peak - exit) / a + (cruise > 0.0 ? cruise / peak : 0.0);
      if (!Math.isFinite(taken))
        throw 'estimate: segment $i of $n takes $taken s (length ${segment.length}, entry $entry, exit $exit, peak $peak, speed ${segment.speed}, acceleration $a)';
      seconds += taken;
    }
    return {seconds: seconds, segments: n};
  }

  static function segmentsOf(program:ToolpathProgram, axisSpeed:Array<Float>, axisAcceleration:Array<Float>):Array<Segment> {
    var result:Array<Segment> = [];
    function add(from:toolpathkit.path.Point3, to:toolpathkit.path.Point3, feed:Float):Void {
      var dx = to.x - from.x, dy = to.y - from.y, dz = to.z - from.z;
      var length = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (length < 1e-12) return;
      var u = [dx / length, dy / length, dz / length];
      var speed = feed, acceleration = Math.POSITIVE_INFINITY;
      for (axis in 0...3) if (Math.abs(u[axis]) > 1e-9) {
        speed = Math.min(speed, axisSpeed[axis] / Math.abs(u[axis]));
        acceleration = Math.min(acceleration, axisAcceleration[axis] / Math.abs(u[axis]));
      }
      if (!(speed > 0.0) || !Math.isFinite(acceleration) || !(acceleration > 0.0))
        throw 'estimate: a segment of ${length} m has speed $speed and acceleration $acceleration';
      result.push({length: length, ux: u[0], uy: u[1], uz: u[2], speed: speed, acceleration: acceleration});
    }
    for (op in program.ops) switch op {
      case Move(_, geometry, feed, _, _):
        var length = GeometryTools.length(geometry);
        var pieces = switch geometry {
          case Line(_, _): 1;
          case Arc(_, radius, _, sweep), Circular(_, radius, _, sweep, _, _):
            var step = 2.0 * Math.acos(Math.max(0.0, 1.0 - CHORD_SAGITTA / radius));
            Std.int(Math.max(1, Math.ceil(Math.abs(sweep) / Math.max(step, 1e-3))));
        };
        var previous = GeometryTools.pointAt(geometry, 0.0);
        for (piece in 1...(pieces + 1)) {
          var next = GeometryTools.pointAt(geometry, piece == pieces ? length : length * piece / pieces);
          add(previous, next, feed);
          previous = next;
        }
      case _:
    }
    return result;
  }
}
