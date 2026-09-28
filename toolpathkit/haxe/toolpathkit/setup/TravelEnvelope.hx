package toolpathkit.setup;

import toolpathkit.path.PathGeometry;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Provenance;

/** Checks the complete authored motion, including arc extrema, in machine space. */
class TravelEnvelope {
  public static function check(lower:Array<Float>, upper:Array<Float>,
      ops:Array<ToolpathOp>):Array<TravelViolation> {
    if (lower == null || upper == null) return [];
    var violations:Array<TravelViolation> = [];
    for (op in ops) {
      var geometry:PathGeometry = null, span:Provenance = null;
      switch op {
        case Move(_, g, _, _, s): geometry = g; span = s;
        case _:
      }
      if (geometry == null) continue;
      var distances = [0.0, GeometryTools.length(geometry)];
      switch geometry {
        case Arc(_, _, start, turn), Circular(_, _, start, turn, _, _):
          var first = Std.int(Math.ceil(Math.min(start, start + turn) /
            (Math.PI * 0.5)));
          var last = Std.int(Math.floor(Math.max(start, start + turn) /
            (Math.PI * 0.5)));
          for (quadrant in first...(last + 1)) {
            var fraction = ((quadrant * Math.PI * 0.5) - start) / turn;
            if (fraction > 0.0 && fraction < 1.0)
              distances.push(GeometryTools.length(geometry) * fraction);
          }
        case _:
      }
      var out = false;
      for (distance in distances) {
        var point = GeometryTools.pointAt(geometry, distance);
        var values = [point.x, point.y, point.z];
        for (axis in 0...3) if (values[axis] < lower[axis] - 1e-9 ||
            values[axis] > upper[axis] + 1e-9) {
          violations.push(new TravelViolation(axis, values[axis],
            lower[axis], upper[axis], span));
          out = true;
          break;
        }
        if (out) break;
      }
    }
    return violations;
  }
}

/** One toolpath point outside the configured machine travel bounds. */
class TravelViolation {
  public final axis:Int;
  public final value:Float;
  public final lower:Float;
  public final upper:Float;
  public final provenance:Provenance;

  public function new(axis:Int, value:Float, lower:Float, upper:Float,
      provenance:Provenance) {
    this.axis = axis; this.value = value; this.lower = lower;
    this.upper = upper; this.provenance = provenance;
  }

  public function message():String
    return '${["X", "Y", "Z"][axis]} travel $value m outside [$lower, $upper] m';
}
