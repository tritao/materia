package camkit;

import cnckit.CncMachine;
import cnckit.CncTravelChecks;
import cnckit.ir.CncGeometry;
import cnckit.ir.CncGeometryTools;
import cnckit.ir.CncOp;
import cnckit.ir.CncPoint;
import cnckit.parse.CncSpan;

/** Stock, clearance and clamp checks in the same work coordinates as CAM IR. */
class CamSetup {
  public final stockMinX:Float;
  public final stockMaxX:Float;
  public final stockMinY:Float;
  public final stockMaxY:Float;
  public final stockTop:Float;
  public final stockBottom:Float;
  public final safeZ:Float;
  public final fixtures:Array<CamFixture>;

  public function new(stockMinX:Float, stockMaxX:Float, stockMinY:Float,
      stockMaxY:Float, stockTop:Float, stockBottom:Float, safeZ:Float,
      ?fixtures:Array<CamFixture>) {
    for (value in [stockMinX, stockMaxX, stockMinY, stockMaxY,
        stockTop, stockBottom, safeZ])
      if (!Math.isFinite(value)) throw "CAM setup needs finite bounds";
    if (stockMinX >= stockMaxX || stockMinY >= stockMaxY ||
        stockBottom >= stockTop || safeZ < stockTop)
      throw "CAM setup needs ordered stock bounds and safe Z above stock";
    this.stockMinX = stockMinX; this.stockMaxX = stockMaxX;
    this.stockMinY = stockMinY; this.stockMaxY = stockMaxY;
    this.stockTop = stockTop; this.stockBottom = stockBottom;
    this.safeZ = safeZ;
    this.fixtures = fixtures == null ? [] : fixtures.copy();
    for (fixture in this.fixtures)
      if (fixture == null || safeZ <= fixture.maxZ)
        throw "CAM safe Z must clear every fixture";
  }

  /** Rejects a program before export. The cutter tip disk is swept through each path. */
  public function validate(program:CamProgram, machine:CncMachine):Void {
    if (program == null || machine == null) throw "CAM export needs a program and machine";
    var travel = CncTravelChecks.check(machine, program.ops);
    if (travel.length > 0) throw 'CAM setup line ${travel[0].span.line}: ${travel[0].message}';
    var radius = 0.0;
    for (op in program.ops) switch op {
      case ToolChange(number, span):
        var tool = machine.tool(number);
        if (tool.diameter <= 0.0) fail(span, 'tool $number needs a positive diameter');
        radius = tool.diameter * 0.5;
      case Rapid(geometry, span): checkPath(geometry, span, radius, false);
      case Feed(geometry, _, _, span): checkPath(geometry, span, radius, true);
      case _:
    }
  }

  function checkPath(geometry:CncGeometry, span:CncSpan, radius:Float,
      cutting:Bool):Void {
    var length = CncGeometryTools.length(geometry);
    var start = CncGeometryTools.pointAt(geometry, 0.0);
    var end = CncGeometryTools.pointAt(geometry, length);
    if (cutting && Math.min(start.z, end.z) < stockBottom - 1e-9)
      fail(span, 'cut goes below stock bottom $stockBottom');
    if (!cutting && (Math.abs(start.x - end.x) > 1e-9 ||
        Math.abs(start.y - end.y) > 1e-9) &&
        Math.min(start.z, end.z) < safeZ - 1e-9)
      fail(span, 'rapid traverse is below safe Z $safeZ');
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
      var current = CncGeometryTools.pointAt(geometry,
        length * index / segments);
      var margin = switch geometry {
        case Line(_, _): 0.0;
        case Arc(_, arcRadius, _, sweep), Circular(_, arcRadius, _, sweep, _, _):
          arcRadius * (1.0 - Math.cos(Math.abs(sweep) / segments / 2.0));
      };
      for (fixture in fixtures)
        if (intersectsFixture(previous, current, radius + margin, fixture))
          fail(span, 'cutter intersects fixture "${fixture.name}"');
      previous = current;
    }
  }

  static function intersectsFixture(a:CncPoint, b:CncPoint, radius:Float,
      fixture:CamFixture):Bool {
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

  static function fail(span:CncSpan, detail:String):Void
    throw 'CAM setup line ${span.line}: $detail';
}
