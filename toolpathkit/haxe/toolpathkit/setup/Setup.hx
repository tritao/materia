package toolpathkit.setup;

import toolpathkit.path.PathGeometry;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.tool.ToolLibrary;

/** Stock, clearance and clamp checks in the same work coordinates as CAM IR. */
class Setup {
  public final id:String;
  /** Translation from work coordinates to machine coordinates, in metres. */
  public final workOrigin:Point3;
  public final stock:Null<SetupStock>;

  public function new(id:String, workOrigin:Point3, ?stock:SetupStock) {
    if (id == null || id.length == 0) throw "setup needs an ID";
    if (workOrigin == null || !Math.isFinite(workOrigin.x) ||
        !Math.isFinite(workOrigin.y) || !Math.isFinite(workOrigin.z))
      throw "setup needs a finite work origin";
    this.id = id;
    this.workOrigin = workOrigin;
    this.stock = stock;
  }

  /** Rejects a program before export. The cutter tip disk is swept through each path. */
  public function validate(ops:Array<ToolpathOp>, tools:ToolLibrary):Void {
    if (ops == null || tools == null) throw "CAM export needs a program and machine";
    if (stock == null) throw "CAM validation needs stock and clearance bounds";
    var radius = 0.0;
    for (op in ops) switch op {
      case ToolChange(number, span):
        var tool = tools.tool(number);
        if (tool.diameter <= 0.0) fail(span, 'tool $number needs a positive diameter');
        radius = tool.diameter * 0.5;
      case Move(Rapid, geometry, _, _, span), Move(Link, geometry, _, _, span),
          Move(Retract, geometry, _, _, span):
        checkPath(geometry, span, radius, false);
      case Move(_, geometry, _, _, span): checkPath(geometry, span, radius, true);
      case _:
    }
  }

  function checkPath(geometry:PathGeometry, span:Provenance, radius:Float,
      cutting:Bool):Void {
    var stock = this.stock;
    if (stock == null) throw "CAM validation needs stock and clearance bounds";
    var length = GeometryTools.length(geometry);
    var start = GeometryTools.pointAt(geometry, 0.0);
    var end = GeometryTools.pointAt(geometry, length);
    if (cutting && Math.min(start.z, end.z) < stock.bottom - 1e-9)
      fail(span, 'cut goes below stock bottom ${stock.bottom}');
    if (!cutting && (Math.abs(start.x - end.x) > 1e-9 ||
        Math.abs(start.y - end.y) > 1e-9) &&
        Math.min(start.z, end.z) < stock.safeZ - 1e-9)
      fail(span, 'rapid traverse is below safe Z ${stock.safeZ}');
    // Short chords plus their maximum arc sagitta conservatively cover a curved path.
    var segments = switch geometry {
      case Line(_, _): 1;
      case Arc(_, radius, _, sweep), Circular(_, radius, _, sweep, _, _):
        Std.int(Math.ceil(Math.max(length / 0.00025,
          Math.abs(sweep) * Math.sqrt(radius / 0.000001) / 4.0)));
    };
    segments = Std.int(Math.max(1, segments));
    var previous = start;
    for (index in 1...(segments + 1)) {
      var current = GeometryTools.pointAt(geometry,
        length * index / segments);
      var margin = switch geometry {
        case Line(_, _): 0.0;
        case Arc(_, arcRadius, _, sweep), Circular(_, arcRadius, _, sweep, _, _):
          arcRadius * (1.0 - Math.cos(Math.abs(sweep) / segments / 2.0));
      };
      for (fixture in stock.fixtures)
        if (intersectsFixture(previous, current, radius + margin, fixture))
          fail(span, 'cutter intersects fixture "${fixture.name}"');
      previous = current;
    }
  }

  static function intersectsFixture(a:Point3, b:Point3, radius:Float,
      fixture:Fixture):Bool {
    // A cutter tip below the clamp can still leave its shank inside it.
    if (Math.min(a.z, b.z) > fixture.maxZ) return false;
    var lo = 0.0, hi = 1.0;
    // Clip the segment to the fixture's vertical range, then to its expanded XY box.
    for (bound in [
      {origin:a.z, delta:b.z - a.z, min:Math.NEGATIVE_INFINITY, max:fixture.maxZ},
      {origin:a.x, delta:b.x - a.x, min:fixture.minX - radius, max:fixture.maxX + radius},
      {origin:a.y, delta:b.y - a.y, min:fixture.minY - radius, max:fixture.maxY + radius}
    ]) {
      if (Math.abs(bound.delta) < 1e-14) {
        if (bound.origin < bound.min || bound.origin > bound.max) return false;
      } else {
        var first = (bound.min - bound.origin) / bound.delta;
        var last = (bound.max - bound.origin) / bound.delta;
        lo = Math.max(lo, Math.min(first, last));
        hi = Math.min(hi, Math.max(first, last));
        if (lo > hi) return false;
      }
    }
    return true;
  }

  static function fail(span:Provenance, detail:String):Void
    throw 'CAM setup line ${span.line}: $detail';
}
