package toolpathkit.setup;

import toolpathkit.geometry.PathGeometry;
import toolpathkit.geometry.GeometryTools;
import toolpathkit.path.Provenance;
import toolpathkit.geometry.Point3;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.geometry.GeometryOffset;
import toolpathkit.path.ToolpathFrame;

/** Checks the controlled point in machine space: setup origin plus G43 length. */
class TravelEnvelope {
  public final lower:Point3;
  public final upper:Point3;

  public function new(lower:Point3, upper:Point3) {
    if (lower == null || upper == null || !Math.isFinite(lower.x) || !Math.isFinite(lower.y) || !Math.isFinite(lower.z)
      || !Math.isFinite(upper.x) || !Math.isFinite(upper.y) || !Math.isFinite(upper.z) || lower.x >= upper.x
      || lower.y >= upper.y || lower.z >= upper.z) throw "machine travel envelope needs finite ordered bounds";
    this.lower = lower;
    this.upper = upper;
  }

  public function check(program:ToolpathProgram):Array<TravelViolation> {
    if (program == null) throw "travel check needs a program";
    var violations:Array<TravelViolation> = [];
    var frame = ToolpathFrame.of(program);
    for (op in program.ops) {
      frame.advance(op);
      var geometry:PathGeometry = null, span:Provenance = null;
      switch op {
        case Move(_, g, _, _, s):
          geometry = GeometryOffset.translate(g, frame.toMachine());
          span = s;
        case MachineMove(_, g, _, _, s):
          geometry = g;
          span = s;
        case _:
      }
      if (geometry == null) continue;
      var distances = [0.0, GeometryTools.length(geometry)];
      switch geometry {
        case Arc(_, _, start, turn), Circular(_, _, start, turn, _, _):
          var first = Std.int(Math.ceil(Math.min(start, start + turn) / (Math.PI * 0.5)));
          var last = Std.int(Math.floor(Math.max(start, start + turn) / (Math.PI * 0.5)));
          for (quadrant in first...(last + 1)) {
            var fraction = ((quadrant * Math.PI * 0.5) - start) / turn;
            if (fraction > 0.0 && fraction < 1.0) distances.push(GeometryTools.length(geometry) * fraction);
          }
        case _:
      }
      var out = false;
      for (distance in distances) {
        var point = GeometryTools.pointAt(geometry, distance);
        var values = [point.x, point.y, point.z];
        var lows = [lower.x, lower.y, lower.z];
        var highs = [upper.x, upper.y, upper.z];
        for (axis in 0...3) if (values[axis] < lows[axis] - 1e-9 || values[axis] > highs[axis] + 1e-9) {
          violations.push(new TravelViolation(axis, values[axis], lows[axis], highs[axis], span));
          out = true;
          break;
        }
        if (out) break;
      }
    }
    return violations;
  }
}
