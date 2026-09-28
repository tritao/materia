import camkit.CamGCodeWriter;
import camkit.CamJob;
import cadkit.modeling.Curve;
import cadkit.modeling.Part;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cnckit.CncCompiler;
import cnckit.CncMachine;
import cnckit.CncTool;
import cnckit.ir.CncGeometryTools;
import cnckit.ir.CncOp;
import cnckit.ir.CncPoint;
import stockkit.CutMoves;
import stockkit.Stock;
import stockkit.StockGrid;

/** Generated plate pocket with a central raised boss. */
class CamIslandPocketFixture {
  public static function run(check:Bool->String->Void):Void {
    var outer = rectangle(0, 0, 40, 30);
    var boss = rectangle(15, 11, 25, 19);
    var sketch = Sketch.face(outer, [boss]);
    var face = sketch.shape.faces().at(0);
    var tool = new CncTool(10, 0, 0.002);
    var program = new CamJob(0.005, 10000)
      .pocketFace(face, tool, -0.002, 0.005, 0.001, 0.001,
        "mm", 0.00005, 0.001).finish();
    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    machine.setTool(tool);
    var lowered = program.lower(machine);
    check(lowered.program != null && lowered.diagnostics.length == 0,
      "island pocket lowers through MotionKit");
    var hasPocketSpan = false;
    for (entry in lowered.sourceMap.entries)
      if (entry.span.line == 1) hasPocketSpan = true;
    check(hasPocketSpan, "island pocket keeps its operation source span");

    var left = -1, right = -1, bossFinishing = 0;
    var ramps = 0, plunges = 0;
    for (index in 0...program.ops.length) switch program.ops[index] {
      case Feed(Line(a, b), speed, _, _) if (b.z < a.z - 1e-9):
        var xy = Math.sqrt(Math.pow(b.x - a.x, 2) +
          Math.pow(b.y - a.y, 2));
        if (xy > 1e-9) {
          ramps++;
          check(Math.abs(speed - 0.005) < 1e-10 &&
            (a.z - b.z) / xy <= 0.1 + 1e-9 &&
            outsideBoss(a) && outsideBoss(b),
            "island pocket ramps within the allowed clearing region");
        } else {
          plunges++;
          check(Math.abs(speed - 0.001) < 1e-10,
            "island pocket vertical entry uses configured plunge feed");
        }
      case Feed(Line(a, b), _, _, _) if (Math.abs(a.z + 0.002) < 1e-9 &&
          Math.abs(b.z + 0.002) < 1e-9 && Math.abs(a.y - 0.015) < 1e-9 &&
          Math.abs(b.y - 0.015) < 1e-9):
        if (a.x < 0.015 && b.x < 0.015) left = index;
        if (a.x > 0.025 && b.x > 0.025) right = index;
      case Feed(Arc(center, _, _, _), _, _, _) if (Math.abs(center.z + 0.002) < 1e-9 &&
          center.x >= 0.015 && center.x <= 0.025 &&
          center.y >= 0.011 && center.y <= 0.019):
        bossFinishing++;
      case _:
    }
    check(left >= 0 && right > left,
      "boss splits the middle clearing row into two passes");
    check(bossFinishing == 4,
      "island boundary gets four rounded outside finishing corners");
    check(ramps > 0 && plunges > 0,
      "island pocket uses ramps where possible and slow plunges in short spans");
    var retracted = false;
    for (index in (left + 1)...right) switch program.ops[index] {
      case Rapid(Line(a, b), _) if (a.z < 0 && b.z >= 0.005 - 1e-9):
        retracted = true;
      case _:
    }
    check(retracted, "tool retracts before crossing the raised boss");
    simulate(program, tool, check);

    var cutterRadius = tool.diameter * 0.5;
    for (op in program.ops) switch op {
      case Feed(geometry, _, _, _):
        var start = CncGeometryTools.pointAt(geometry, 0);
        var end = CncGeometryTools.pointAt(geometry,
          CncGeometryTools.length(geometry));
        if (Math.abs(start.z - end.z) > 1e-9) continue;
        for (fraction in [0.0, 0.25, 0.5, 0.75, 1.0]) {
          var point = CncGeometryTools.pointAt(geometry,
            CncGeometryTools.length(geometry) * fraction);
          check(outsideBoss(point) &&
            bossDistance(point) >= cutterRadius - 1e-8,
            "island pocket keeps the full cutter outside the boss");
        }
      case _:
    }
    for (ix in 0...40) for (iy in 0...30) {
      var point = new CncPoint((ix + 0.5) * 0.001,
        (iy + 0.5) * 0.001, -0.002);
      if (!outsideBoss(point)) continue;
      var best = Math.POSITIVE_INFINITY;
      for (op in program.ops) switch op {
        case Feed(geometry, _, _, _):
          var start = CncGeometryTools.pointAt(geometry, 0);
          var end = CncGeometryTools.pointAt(geometry,
            CncGeometryTools.length(geometry));
          if (Math.abs(start.z + 0.002) > 1e-9 ||
              Math.abs(end.z + 0.002) > 1e-9) continue;
          switch geometry {
            case Line(a, b): best = Math.min(best, segmentDistance(point, a, b));
            case Arc(_, _, _, _):
              for (sample in 0...65) {
                var position = CncGeometryTools.pointAt(geometry,
                  CncGeometryTools.length(geometry) * sample / 64);
                best = Math.min(best, point.distanceTo(position));
              }
            case _:
          }
        case _:
      }
      check(best <= cutterRadius + 0.00001,
        'island pocket covers the plate grid point ${point.x},${point.y}: $best');
    }

    var imported = new CncCompiler(machine).compileDetailed(
      CamGCodeWriter.write(program, CamTestSetup.standard(), machine));
    check(imported.diagnostics.length == 0 &&
      imported.ops.length == program.ops.length,
      "island pocket G-code recompiles with the same operation count");
    for (index in 0...program.ops.length) switch [program.ops[index], imported.ops[index]] {
      case [Feed(a, feedA, _, _), Feed(b, feedB, _, _)]:
        check(Math.abs(feedA - feedB) < 1e-8,
          "island pocket G-code preserves ramp and plunge feeds");
        for (fraction in [0.0, 0.5, 1.0]) {
          var original = CncGeometryTools.pointAt(a,
            CncGeometryTools.length(a) * fraction);
          var reparsed = CncGeometryTools.pointAt(b,
            CncGeometryTools.length(b) * fraction);
          check(original.distanceTo(reparsed) < 1e-8,
            "island pocket G-code preserves geometry");
        }
      case [Rapid(a, _), Rapid(b, _)]:
        for (fraction in [0.0, 0.5, 1.0]) {
          var original = CncGeometryTools.pointAt(a,
            CncGeometryTools.length(a) * fraction);
          var reparsed = CncGeometryTools.pointAt(b,
            CncGeometryTools.length(b) * fraction);
          check(original.distanceTo(reparsed) < 1e-8,
            "island pocket G-code preserves geometry");
        }
      case _:
    }
    var secondBoss = rectangle(5, 5, 9, 9);
    var twoBossSketch = Sketch.face(outer, [boss, secondBoss]);
    var twoBossFace = twoBossSketch.shape.faces().at(0);
    var twoBossProgram = new CamJob(0.005, 10000)
      .pocketFace(twoBossFace, tool, -0.001, 0.005, 0.001).finish();
    check(twoBossProgram.lower(machine).diagnostics.length == 0,
      "face pocket supports two separate islands");
    for (op in twoBossProgram.ops) switch op {
      case Feed(geometry, _, _, _):
        var a = CncGeometryTools.pointAt(geometry, 0);
        var b = CncGeometryTools.pointAt(geometry,
          CncGeometryTools.length(geometry));
        if (Math.abs(a.z - b.z) > 1e-9) continue;
        for (fraction in [0.0, 0.5, 1.0]) {
          var point = CncGeometryTools.pointAt(geometry,
            CncGeometryTools.length(geometry) * fraction);
          var x = Math.max(0.005, Math.min(0.009, point.x));
          var y = Math.max(0.005, Math.min(0.009, point.y));
          check(Math.sqrt(Math.pow(point.x - x, 2) +
            Math.pow(point.y - y, 2)) >= cutterRadius - 1e-8,
            "second island retains cutter-radius clearance");
        }
      case _:
    }
    twoBossFace.close(); twoBossSketch.close(); secondBoss.close();

    var tightBoss = rectangle(1, 11, 10, 19);
    var tightSketch = Sketch.face(outer, [tightBoss]);
    var tightFace = tightSketch.shape.faces().at(0);
    var rejectedJob = new CamJob(0.005, 10000), rejected = false;
    try rejectedJob.pocketFace(tightFace,
      new CncTool(11, 0, 0.004), -0.001, 0.005, 0.001)
    catch (_:Dynamic) rejected = true;
    check(rejected, "pocket rejects an island closer than the cutter radius to the wall");
    rejected = false;
    try rejectedJob.finish() catch (_:Dynamic) rejected = true;
    check(rejected, "rejected island pocket leaves no partial CAM operation");
    tightFace.close(); tightSketch.close(); tightBoss.close();
    face.close(); sketch.close(); outer.close(); boss.close();
  }

  /**
    Cuts the program from a plate with StockKit and compares it with the
    finished part: nothing gouged, no rapid through stock, no shank contact,
    and leftover only in the pocket's four inside corners.
  **/
  static function simulate(program:camkit.CamProgram, tool:CncTool,
      check:Bool->String->Void):Void {
    var solids:Array<Part> = [];
    function box(width:Float, depth:Float, height:Float, x:Float, y:Float, z:Float):Part {
      var local = Part.box(width, depth, height, Min, Min, Min);
      var placed = local.translated(new Vector(x, y, z));
      solids.push(local);
      solids.push(placed);
      return placed;
    }
    var blank = box(0.05, 0.04, 0.01, -0.005, -0.005, -0.01);
    var pocket = box(0.04, 0.03, 0.003, 0, 0, -0.002).subtract(box(0.01, 0.008, 0.005, 0.015, 0.011, -0.003));
    var part = blank.subtract(pocket);
    var mesh = part.shape.tessellate(1e-6, 0.1);
    solids.push(pocket);
    solids.push(part);
    for (solid in solids) solid.close();
    // Rays off the part's round coordinates.
    var grid = new StockGrid(-0.00487, -0.00491, 0.00025, 200, 160);
    var target = Stock.fromMesh(grid, mesh);
    var stock = Stock.box(grid, -0.005, -0.005, -0.01, 0.045, 0.035, 0);
    var report = stock.cut(CutMoves.fromOps(program.ops, program.tool));
    check(report.rapidContacts().length == 0, "island pocket never rapids through stock");
    check(report.collisions().length == 0, "island pocket keeps the shank out of the stock");
    var comparison = stock.compare(target);
    check(comparison.deepestGouge() < 1e-9,
      'island pocket does not gouge its part (deepest ${comparison.deepestGouge()})');
    check(comparison.thickestLeftover() <= 0.002 + 1e-12,
      "island pocket leaves nothing thicker than its depth");
    // A round cutter leaves a fillet of its radius in each inside corner of
    // the outer wall, (1 - pi / 4) r^2 each; the boss's corners are outside
    // corners. At 0.25 mm rays each fillet is about three rays.
    var r = tool.diameter / 2;
    var fillets = (4 - Math.PI) * r * r * 0.002;
    check(Math.abs(comparison.leftoverVolume() - fillets) <= 0.25 * fillets,
      'island pocket leftover is the corner fillets: ${comparison.leftoverVolume()} vs $fillets');
    target.dispose();
    stock.dispose();
  }

  static function rectangle(x0:Float, y0:Float, x1:Float, y1:Float):Curve
    return Curve.polyline([new Vector(x0, y0), new Vector(x1, y0),
      new Vector(x1, y1), new Vector(x0, y1)], true);

  static function outsideBoss(point:CncPoint):Bool
    return point.x <= 0.015 || point.x >= 0.025 ||
      point.y <= 0.011 || point.y >= 0.019;

  static function bossDistance(point:CncPoint):Float {
    var x = Math.max(0.015, Math.min(0.025, point.x));
    var y = Math.max(0.011, Math.min(0.019, point.y));
    return Math.sqrt(Math.pow(point.x - x, 2) +
      Math.pow(point.y - y, 2));
  }

  static function segmentDistance(point:CncPoint, a:CncPoint, b:CncPoint):Float {
    var dx = b.x - a.x, dy = b.y - a.y;
    var t = Math.max(0.0, Math.min(1.0,
      ((point.x - a.x) * dx + (point.y - a.y) * dy) / (dx * dx + dy * dy)));
    return Math.sqrt(Math.pow(point.x - a.x - t * dx, 2) +
      Math.pow(point.y - a.y - t * dy, 2));
  }
}
