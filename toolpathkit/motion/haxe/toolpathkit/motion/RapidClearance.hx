package toolpathkit.motion;

import toolpathkit.path.ToolpathProgram;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathFrame;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.path.MoveKind;
import toolpathkit.tool.CutterProfile;

/** A fixed stock envelope in machine coordinates, metres. */
typedef ClearanceStock = { var minimum:Array<Float>; var maximum:Array<Float>; }

/** Reserves drive displacement around stock. Portions without this clearance are explicit exact-stop feed approaches.
 * Raw stock boxes are conservative: an already machined pocket is still treated as occupied. No geometry is displaced.
 */
class RapidClearance {
  public static function apply(program:ToolpathProgram, stocks:Array<ClearanceStock>, displacement:Array<Float>,
      initialTool:Int = -1):ToolpathProgram {
    if (displacement == null || displacement.length != 3) throw "Rapid clearance needs three axis displacements";
    for (value in displacement) if (!Math.isFinite(value) || value < 0) throw "Rapid clearance must be finite and non-negative";
    for (stock in stocks) {
      if (stock.minimum.length != 3 || stock.maximum.length != 3) throw "Rapid stock needs three bounds";
      for (axis in 0...3) if (!Math.isFinite(stock.minimum[axis]) || !Math.isFinite(stock.maximum[axis]) ||
          stock.minimum[axis] > stock.maximum[axis]) throw "Rapid stock bounds must be ordered and finite";
    }
    if (stocks.length == 0) return program;
    // G0 has no feed. Use the slowest cutting feed in this program, including a feed first declared after the approach.
    var feed = Math.POSITIVE_INFINITY;
    for (op in program.ops) switch op {
      case Move(kind, _, speed, _, _), MachineMove(kind, _, speed, _, _):
        if (kind != Rapid && kind != Link && kind != Retract && speed > 0) feed = Math.min(feed, speed);
      case _:
    }
    var result:Array<ToolpathOp> = [], frame = ToolpathFrame.of(program), toolNumber = initialTool;
    for (op in program.ops) {
      frame.advance(op);
      switch op {
        case ToolChange(number, _): toolNumber = number; result.push(op);
        case Move(kind, geometry, speed, tolerance, span), MachineMove(kind, geometry, speed, tolerance, span)
            if (kind == Rapid || kind == Link || kind == Retract):
          var tool = toolNumber < 0 ? null : program.tools.tool(toolNumber);
          if (tool == null) { result.push(op); continue; }
          var machine = switch op { case MachineMove(_, _, _, _, _): true; case _: false; };
          var shift = machine ? [0.0, 0.0, 0.0] : frame.toMachine();
          var profile = tool.profile(), radius = 0.0;
          for (segment in profile.segments) {
            radius = Math.max(radius, Math.max(CutterProfile.startOf(segment).r, CutterProfile.endOf(segment).r));
            switch segment {
              case Arc(cr, cz, r0, z0, _, _, _): radius = Math.max(radius, cr + Math.sqrt((r0-cr)*(r0-cr)+(z0-cz)*(z0-cz)));
              case _:
            }
          }
          var from = 1.0, to = 0.0;
          for (stock in stocks) {
            // A vertical tool occupies [gauge-length, gauge-length+profile.height]. Expand the stock by that solid
            // and the independent drive displacement. Work origins and the active G43 are in shift, not the actual tool.
            var low = [stock.minimum[0]-radius-displacement[0]-shift[0], stock.minimum[1]-radius-displacement[1]-shift[1],
              stock.minimum[2]+tool.length-profile.height()-displacement[2]-shift[2]];
            var high = [stock.maximum[0]+radius+displacement[0]-shift[0], stock.maximum[1]+radius+displacement[1]-shift[1],
              stock.maximum[2]+tool.length+displacement[2]-shift[2]];
            switch geometry {
              case Line(start, end):
                var interval = clip(start, end, low, high);
                if (interval != null) { from = Math.min(from, interval[0]); to = Math.max(to, interval[1]); }
              case _:
                // G0 is linear in CNC. A caller supplying a curved rapid gets a conservative feed over the whole curve.
                from = 0.0; to = 1.0;
            }
          }
          if (from > to) { result.push(op); continue; }
          if (!Math.isFinite(feed)) throw 'Rapid approach on line ${span.line} needs a cutting feed';
          switch geometry {
            case Line(start, end):
              if (from > 0.0) emit(result, machine, span, kind, Line(start, at(start,end,from)), speed);
              if (to > from) emit(result, machine, span, Cut, Line(at(start,end,from), at(start,end,to)), feed);
              if (to < 1.0) emit(result, machine, span, kind, Line(at(start,end,to), end), speed);
            case _: emit(result, machine, span, Cut, geometry, feed);
          }
        case _: result.push(op);
      }
    }
    return new ToolpathProgram(result, program.tools, program.setups);
  }

  static function at(start:Point3, end:Point3, t:Float):Point3
    return new Point3(start.x+(end.x-start.x)*t, start.y+(end.y-start.y)*t, start.z+(end.z-start.z)*t);

  static function emit(result:Array<ToolpathOp>, machine:Bool, span:Provenance, kind:MoveKind, path:PathGeometry, velocity:Float):Void {
    // Distinct identities let execution distinguish a split rapid from its feed approach on one source line.
    var source = new Provenance(span.line, span.column, span.length, span.kind, span.operationIndex, span.operationId, span.featureRef);
    result.push(machine ? MachineMove(kind, path, velocity, 0.0, source) : Move(kind, path, velocity, 0.0, source));
  }

  /** Closed segment/box intersection, including a tangent: rapid space is the outside of the expanded envelope. */
  static function clip(start:Point3, end:Point3, low:Array<Float>, high:Array<Float>):Null<Array<Float>> {
    var a = [start.x,start.y,start.z], b = [end.x,end.y,end.z], from = 0.0, to = 1.0;
    for (axis in 0...3) {
      var delta = b[axis]-a[axis];
      if (delta == 0.0) { if (a[axis] < low[axis] || a[axis] > high[axis]) return null; }
      else {
        var first = (low[axis]-a[axis])/delta, last = (high[axis]-a[axis])/delta;
        from = Math.max(from, Math.min(first,last)); to = Math.min(to,Math.max(first,last));
        if (from > to) return null;
      }
    }
    return [from,to];
  }
}
