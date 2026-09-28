import cnckit.CncWriter;
import camkit.CamJob;
import cadkit.modeling.Curve;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cnckit.CncCompiler;
import cnckit.CncMachine;
import toolpathkit.tool.Tool;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Point3;

/** Generated plate pocket with a central raised boss. */
class CamIslandPocketFixture {
  public static function run(check:Bool->String->Void):Void {
    var outer = rectangle(0, 0, 40, 30);
    var boss = rectangle(15, 11, 25, 19);
    var sketch = Sketch.face(outer, [boss]);
    var face = sketch.shape.faces().at(0);
    var tool = new Tool(10, 0, 0.002);
    var program = new CamJob(0.005, 10000)
      .pocketFace(face, tool, -0.002, 0.005, 0.001, 0.001,
        "mm", 0.00005, 0.001).finish();
    var hasFaceRef = false;
    for (op in program.ops) switch op {
      case Move(_, _, _, _, provenance):
        if (provenance.featureRef == 'face:${face.index}' &&
            provenance.operationId != null) hasFaceRef = true;
      case _:
    }
    check(hasFaceRef, "face pocket preserves CAD face reference");
    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    machine.toolLibrary.set(tool);
    var lowered = CamTestLowering.lower(program, machine);
    check(lowered.program != null && lowered.diagnostics.length == 0,
      "island pocket lowers through MotionKit");
    var hasPocketSpan = false;
    for (entry in lowered.sourceMap.entries)
      if (entry.provenance.line == 1) hasPocketSpan = true;
    check(hasPocketSpan, "island pocket keeps its operation source span");

    var left = -1, right = -1, bossFinishing = 0;
    var ramps = 0, plunges = 0;
    for (index in 0...program.ops.length) switch program.ops[index] {
      case Move(kind, Line(a, b), speed, _, _) if ((kind == Plunge || kind == Ramp) && b.z < a.z - 1e-9):
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
      case Move(Cut, Line(a, b), _, _, _) if (Math.abs(a.z + 0.002) < 1e-9 &&
          Math.abs(b.z + 0.002) < 1e-9 && Math.abs(a.y - 0.015) < 1e-9 &&
          Math.abs(b.y - 0.015) < 1e-9):
        if (a.x < 0.015 && b.x < 0.015) left = index;
        if (a.x > 0.025 && b.x > 0.025) right = index;
      case Move(Cut, Arc(center, _, _, _), _, _, _) if (Math.abs(center.z + 0.002) < 1e-9 &&
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
      case Move(Retract, Line(a, b), _, _, _) if (a.z < 0 && b.z >= 0.005 - 1e-9):
        retracted = true;
      case _:
    }
    check(retracted, "tool retracts before crossing the raised boss");

    var cutterRadius = tool.diameter * 0.5;
    for (op in program.ops) switch op {
      case Move(Cut, geometry, _, _, _):
        var start = GeometryTools.pointAt(geometry, 0);
        var end = GeometryTools.pointAt(geometry,
          GeometryTools.length(geometry));
        if (Math.abs(start.z - end.z) > 1e-9) continue;
        for (fraction in [0.0, 0.25, 0.5, 0.75, 1.0]) {
          var point = GeometryTools.pointAt(geometry,
            GeometryTools.length(geometry) * fraction);
          check(outsideBoss(point) &&
            bossDistance(point) >= cutterRadius - 1e-8,
            "island pocket keeps the full cutter outside the boss");
        }
      case _:
    }
    for (ix in 0...40) for (iy in 0...30) {
      var point = new Point3((ix + 0.5) * 0.001,
        (iy + 0.5) * 0.001, -0.002);
      if (!outsideBoss(point)) continue;
      var best = Math.POSITIVE_INFINITY;
      for (op in program.ops) switch op {
        case Move(Cut, geometry, _, _, _):
          var start = GeometryTools.pointAt(geometry, 0);
          var end = GeometryTools.pointAt(geometry,
            GeometryTools.length(geometry));
          if (Math.abs(start.z + 0.002) > 1e-9 ||
              Math.abs(end.z + 0.002) > 1e-9) continue;
          switch geometry {
            case Line(a, b): best = Math.min(best, segmentDistance(point, a, b));
            case Arc(_, _, _, _):
              for (sample in 0...65) {
                var position = GeometryTools.pointAt(geometry,
                  GeometryTools.length(geometry) * sample / 64);
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
      CncWriter.write(program.ops, CamTestSetup.standard(), machine));
    check(imported.diagnostics.length == 0 &&
      imported.ops.length == program.ops.length,
      "island pocket G-code recompiles with the same operation count");
    for (index in 0...program.ops.length) switch [program.ops[index], imported.ops[index]] {
      case [Move(Cut, a, feedA, _, _), Move(Cut, b, feedB, _, _)]:
        check(Math.abs(feedA - feedB) < 1e-8,
          "island pocket G-code preserves ramp and plunge feeds");
        for (fraction in [0.0, 0.5, 1.0]) {
          var original = GeometryTools.pointAt(a,
            GeometryTools.length(a) * fraction);
          var reparsed = GeometryTools.pointAt(b,
            GeometryTools.length(b) * fraction);
          check(original.distanceTo(reparsed) < 1e-8,
            "island pocket G-code preserves geometry");
        }
      case [Move(_, a, _, _, _), Move(_, b, _, _, _)]:
        for (fraction in [0.0, 0.5, 1.0]) {
          var original = GeometryTools.pointAt(a,
            GeometryTools.length(a) * fraction);
          var reparsed = GeometryTools.pointAt(b,
            GeometryTools.length(b) * fraction);
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
    check(CamTestLowering.lower(twoBossProgram, machine).diagnostics.length == 0,
      "face pocket supports two separate islands");
    for (op in twoBossProgram.ops) switch op {
      case Move(Cut, geometry, _, _, _):
        var a = GeometryTools.pointAt(geometry, 0);
        var b = GeometryTools.pointAt(geometry,
          GeometryTools.length(geometry));
        if (Math.abs(a.z - b.z) > 1e-9) continue;
        for (fraction in [0.0, 0.5, 1.0]) {
          var point = GeometryTools.pointAt(geometry,
            GeometryTools.length(geometry) * fraction);
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
      new Tool(11, 0, 0.004), -0.001, 0.005, 0.001)
    catch (_:Dynamic) rejected = true;
    check(rejected, "pocket rejects an island closer than the cutter radius to the wall");
    rejected = false;
    try rejectedJob.finish() catch (_:Dynamic) rejected = true;
    check(rejected, "rejected island pocket leaves no partial CAM operation");
    tightFace.close(); tightSketch.close(); tightBoss.close();
    face.close(); sketch.close(); outer.close(); boss.close();
  }

  static function rectangle(x0:Float, y0:Float, x1:Float, y1:Float):Curve
    return Curve.polyline([new Vector(x0, y0), new Vector(x1, y0),
      new Vector(x1, y1), new Vector(x0, y1)], true);

  static function outsideBoss(point:Point3):Bool
    return point.x <= 0.015 || point.x >= 0.025 ||
      point.y <= 0.011 || point.y >= 0.019;

  static function bossDistance(point:Point3):Float {
    var x = Math.max(0.015, Math.min(0.025, point.x));
    var y = Math.max(0.011, Math.min(0.019, point.y));
    return Math.sqrt(Math.pow(point.x - x, 2) +
      Math.pow(point.y - y, 2));
  }

  static function segmentDistance(point:Point3, a:Point3, b:Point3):Float {
    var dx = b.x - a.x, dy = b.y - a.y;
    var t = Math.max(0.0, Math.min(1.0,
      ((point.x - a.x) * dx + (point.y - a.y) * dy) / (dx * dx + dy * dy)));
    return Math.sqrt(Math.pow(point.x - a.x - t * dx, 2) +
      Math.pow(point.y - a.y - t * dy, 2));
  }
}
