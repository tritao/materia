import camkit.CamContour;
import camkit.CamGCodeWriter;
import camkit.CamJob;
import cadkit.modeling.Curve;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cnckit.CncCompiler;
import cnckit.CncMachine;
import toolpathkit.tool.Tool;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.ToolpathOp;

/** CAD fixtures authored locally so CamKit tests have no example-project dependency. */
class CamGeneratedFixtures {
  public static function run(check:Bool->String->Void):Void {
    roundedPlate(check);
    plateWithHole(check);
    concavePlate(check);
    concavePockets(check);
  }

  static function concavePockets(check:Bool->String->Void):Void {
    var l = new CamContour([
      new toolpathkit.path.Point3(0, 0, 0),
      new toolpathkit.path.Point3(0.03, 0, 0),
      new toolpathkit.path.Point3(0.03, 0.01, 0),
      new toolpathkit.path.Point3(0.01, 0.01, 0),
      new toolpathkit.path.Point3(0.01, 0.03, 0),
      new toolpathkit.path.Point3(0, 0.03, 0)
    ]);
    var u = new CamContour([
      new toolpathkit.path.Point3(0, 0, 0),
      new toolpathkit.path.Point3(0.03, 0, 0),
      new toolpathkit.path.Point3(0.03, 0.03, 0),
      new toolpathkit.path.Point3(0.02, 0.03, 0),
      new toolpathkit.path.Point3(0.02, 0.01, 0),
      new toolpathkit.path.Point3(0.01, 0.01, 0),
      new toolpathkit.path.Point3(0.01, 0.03, 0),
      new toolpathkit.path.Point3(0, 0.03, 0)
    ]);
    var tool = new Tool(8, 0, 0.002);
    var lProgram = new CamJob(0.005, 10000)
      .pocket(l, tool, -0.003, 0.005, 0.001, 0.001).finish();
    var uProgram = new CamJob(0.005, 10000)
      .pocket(u, tool, -0.001, 0.005, 0.001).finish();
    var levels = new Map<String, Bool>();
    for (op in lProgram.ops) switch op {
      case Move(Cut, Line(start, end), _, _, _)
        if (Math.abs(start.z - end.z) < 1e-9):
        levels.set(Std.string(Math.round(start.z * 1000000)), true);
      case _:
    }
    var levelCount = 0;
    for (_ in levels.keys()) levelCount++;
    check(levelCount == 3, "L pocket clears at three stepped depths");
    checkPocketCoverage(l, lProgram, -0.003, check);
    checkPocketCoverage(u, uProgram, -0.001, check);

    var leftIndex = -1, rightIndex = -1;
    for (index in 0...uProgram.ops.length) switch uProgram.ops[index] {
      case Move(Cut, Line(a, b), _, _, _) if (Math.abs(a.z + 0.001) < 1e-9 &&
          Math.abs(b.z + 0.001) < 1e-9 && Math.abs(a.y - 0.02) < 1e-9 &&
          Math.abs(b.y - 0.02) < 1e-9):
        if (a.x < 0.01 && b.x < 0.01) leftIndex = index;
        if (a.x > 0.02 && b.x > 0.02) rightIndex = index;
      case _:
    }
    check(leftIndex >= 0 && rightIndex > leftIndex,
      "U pocket has separate left and right clearing passes");
    var retracted = false;
    for (index in (leftIndex + 1)...rightIndex) switch uProgram.ops[index] {
      case Move(Retract, Line(a, b), _, _, _) if (a.z < 0 && b.z >= 0.005 - 1e-9):
        retracted = true;
      case _:
    }
    check(retracted, "U pocket retracts before crossing its open notch");
    var rejected = false;
    try new CamJob(0.005, 10000).pocket(u,
      new Tool(9, 0, 0.012), -0.001, 0.005, 0.004)
    catch (_:Dynamic) rejected = true;
    check(rejected, "U pocket rejects a tool wider than its arms");
    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    machine.setTool(tool);
    for (program in [lProgram, uProgram]) {
      check(CamTestLowering.lower(program, machine).diagnostics.length == 0,
        "concave pocket lowers through MotionKit");
      var imported = new CncCompiler(machine).compileDetailed(
        CamGCodeWriter.write(program, CamTestSetup.standard(), machine));
      check(imported.diagnostics.length == 0 &&
        imported.ops.length == program.ops.length,
        "concave pocket G-code preserves operation count");
      for (index in 0...program.ops.length) switch [program.ops[index], imported.ops[index]] {
        case [Move(Cut, a, _, _, _), Move(Cut, b, _, _, _)],
             [Move(_, a, _, _, _), Move(_, b, _, _, _)]:
          for (fraction in [0.0, 0.5, 1.0]) {
            var original = GeometryTools.pointAt(a,
              GeometryTools.length(a) * fraction);
            var reparsed = GeometryTools.pointAt(b,
              GeometryTools.length(b) * fraction);
            check(original.distanceTo(reparsed) < 1e-8,
              "concave pocket G-code preserves toolpath geometry");
          }
        case _:
      }
    }
  }

  static function checkPocketCoverage(contour:CamContour,
      program:camkit.CamProgram, depth:Float,
      check:Bool->String->Void):Void {
    for (op in program.ops) switch op {
      case Move(Cut, Line(a, b), _, _, _) if (Math.abs(a.z - depth) < 1e-9 &&
          Math.abs(b.z - depth) < 1e-9):
        for (fraction in [0.0, 0.5, 1.0]) {
          var point = new toolpathkit.path.Point3(a.x + (b.x - a.x) * fraction,
            a.y + (b.y - a.y) * fraction, depth);
          var clearance = Math.POSITIVE_INFINITY;
          for (i in 0...contour.vertices.length)
            clearance = Math.min(clearance, segmentDistance(point,
              contour.vertices[i],
              contour.vertices[(i + 1) % contour.vertices.length]));
          check(inside(point, contour.vertices) &&
            clearance >= 0.001 - 1e-8,
            "concave pocket centreline stays inside with cutter clearance");
        }
      case _:
    }
    for (ix in 0...30) for (iy in 0...30) {
      var point = new toolpathkit.path.Point3((ix + 0.5) * 0.001,
        (iy + 0.5) * 0.001, depth);
      if (!inside(point, contour.vertices)) continue;
      var best = Math.POSITIVE_INFINITY;
      for (op in program.ops) switch op {
        case Move(Cut, geometry, _, _, _):
          var start = GeometryTools.pointAt(geometry, 0);
          var end = GeometryTools.pointAt(geometry,
            GeometryTools.length(geometry));
          if (Math.abs(start.z - depth) > 1e-9 ||
              Math.abs(end.z - depth) > 1e-9) continue;
          switch geometry {
            case Line(a, b): best = Math.min(best, segmentDistance(point, a, b));
            case Arc(_, _, _, _):
              for (sample in 0...65) {
                var center = GeometryTools.pointAt(geometry,
                  GeometryTools.length(geometry) * sample / 64);
                best = Math.min(best, point.distanceTo(center));
              }
            case _:
          }
        case _:
      }
      check(best <= 0.001 + 0.00001,
        'concave pocket covers reachable grid point ${point.x},${point.y}: $best');
    }
  }

  static function inside(point:toolpathkit.path.Point3,
      polygon:Array<toolpathkit.path.Point3>):Bool {
    var hit = false;
    for (i in 0...polygon.length) {
      var a = polygon[i], b = polygon[(i + 1) % polygon.length];
      if ((a.y > point.y) != (b.y > point.y) &&
          point.x < a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y))
        hit = !hit;
    }
    return hit;
  }

  static function concavePlate(check:Bool->String->Void):Void {
    var contour = new CamContour([
      new toolpathkit.path.Point3(0, 0, 0),
      new toolpathkit.path.Point3(0.03, 0, 0),
      new toolpathkit.path.Point3(0.03, 0.01, 0),
      new toolpathkit.path.Point3(0.01, 0.01, 0),
      new toolpathkit.path.Point3(0.01, 0.03, 0),
      new toolpathkit.path.Point3(0, 0.03, 0)
    ]);
    var tool = new Tool(5, 0, 0.002);
    var program = new CamJob(0.005, 10000)
      .profile(contour, tool, -0.002, 0.005, "outside").finish();
    var arcs = 0, cornerMeeting = 0;
    for (op in program.ops) switch op {
      case Move(Cut, geometry, _, _, span) if (span.line == 1):
        switch geometry {
          case Arc(_, _, _, _): arcs++;
          case _:
        }
        var start = GeometryTools.pointAt(geometry, 0);
        var end = GeometryTools.pointAt(geometry,
          GeometryTools.length(geometry));
        if (Math.abs(start.z - end.z) > 1e-9) continue;
        for (fraction in [0.0, 0.25, 0.5, 0.75, 1.0]) {
          var point = GeometryTools.pointAt(geometry,
            GeometryTools.length(geometry) * fraction);
          var clearance = Math.POSITIVE_INFINITY;
          for (i in 0...contour.vertices.length)
            clearance = Math.min(clearance, segmentDistance(point,
              contour.vertices[i], contour.vertices[(i + 1) % contour.vertices.length]));
          check(clearance >= 0.001 - 1e-8,
            "concave outside cutter path clears its authored contour");
        }
        if (Math.abs(end.x - 0.011) < 1e-8 &&
            Math.abs(end.y - 0.011) < 1e-8) cornerMeeting++;
      case _:
    }
    check(arcs == 5, "L plate rounds five outside corners and trims the inward corner");
    check(cornerMeeting == 1, "inward corner meets at the two shifted-edge intersection");
    var inside = new CamJob(0.005, 10000)
      .profile(contour, tool, -0.002, 0.005, "inside").finish();
    var insideArcs = 0;
    for (op in inside.ops) switch op {
      case Move(Cut, Arc(center, radius, _, sweep), _, _, _):
        insideArcs++;
        check(Math.abs(center.x - 0.01) < 1e-9 &&
          Math.abs(center.y - 0.01) < 1e-9 &&
          Math.abs(radius - 0.001) < 1e-9 && sweep < 0.0,
          "inside L profile rounds its reflex corner into the hole");
      case _:
    }
    check(insideArcs == 1, "inside L profile has one rounded inward corner");
    var reversedVertices = contour.vertices.copy();
    reversedVertices.reverse();
    var reversed = new CamContour(reversedVertices);
    for (side in ["outside", "inside"]) {
      var reversedProgram = new CamJob(0.005, 10000)
        .profile(reversed, tool, -0.002, 0.005, side).finish();
      var reversedArcs = 0;
      for (op in reversedProgram.ops) switch op {
        case Move(Cut, Arc(_, _, _, _), _, _, _): reversedArcs++;
        case _:
      }
      check(reversedArcs == (side == "outside" ? 5 : 1),
        "concave offset follows the contour's winding");
    }
    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    machine.setTool(tool);
    check(CamTestLowering.lower(program, machine).diagnostics.length == 0,
      "concave profile lowers through MotionKit");
    check(CamTestLowering.lower(inside, machine).diagnostics.length == 0,
      "concave inside profile lowers through MotionKit");
    var imported = new CncCompiler(machine).compileDetailed(CamGCodeWriter.write(program, CamTestSetup.standard(), machine));
    check(imported.diagnostics.length == 0 && imported.ops.length == program.ops.length,
      "concave profile G-code recompiles with the same operations");
    for (index in 0...program.ops.length) switch [program.ops[index], imported.ops[index]] {
      case [Move(Cut, a, _, _, _), Move(Cut, b, _, _, _)],
           [Move(_, a, _, _, _), Move(_, b, _, _, _)]:
        for (fraction in [0.0, 0.5, 1.0]) {
          var original = GeometryTools.pointAt(a,
            GeometryTools.length(a) * fraction);
          var reparsed = GeometryTools.pointAt(b,
            GeometryTools.length(b) * fraction);
          check(original.distanceTo(reparsed) < 1e-8,
            "concave G-code preserves the cutter path");
        }
      case _:
    }
    var insideImported = new CncCompiler(machine).compileDetailed(
      CamGCodeWriter.write(inside, CamTestSetup.standard(), machine));
    check(insideImported.diagnostics.length == 0 &&
      insideImported.ops.length == inside.ops.length,
      "concave inside G-code recompiles with the same operations");
    for (index in 0...inside.ops.length) switch [inside.ops[index], insideImported.ops[index]] {
      case [Move(Cut, a, _, _, _), Move(Cut, b, _, _, _)],
           [Move(_, a, _, _, _), Move(_, b, _, _, _)]:
        for (fraction in [0.0, 0.5, 1.0]) {
          var original = GeometryTools.pointAt(a,
            GeometryTools.length(a) * fraction);
          var reparsed = GeometryTools.pointAt(b,
            GeometryTools.length(b) * fraction);
          check(original.distanceTo(reparsed) < 1e-8,
            "concave inside G-code preserves the cutter path");
        }
      case _:
    }
    var rejected = false;
    try new CamJob(0.005, 10000).profile(contour,
      new Tool(6, 0, 0.05), -0.002, 0.005, "outside")
    catch (_:Dynamic) rejected = true;
    check(rejected, "oversized cutter is rejected at the narrow concave feature");
    var narrowNotch = new CamContour([
      new toolpathkit.path.Point3(0, 0, 0),
      new toolpathkit.path.Point3(0.03, 0, 0),
      new toolpathkit.path.Point3(0.03, 0.03, 0),
      new toolpathkit.path.Point3(0.02, 0.03, 0),
      new toolpathkit.path.Point3(0.02, 0.01, 0),
      new toolpathkit.path.Point3(0.01, 0.01, 0),
      new toolpathkit.path.Point3(0.01, 0.03, 0),
      new toolpathkit.path.Point3(0, 0.03, 0)
    ]);
    rejected = false;
    try new CamJob(0.005, 10000).profile(narrowNotch,
      new Tool(7, 0, 0.012), -0.002, 0.005, "outside")
    catch (_:Dynamic) rejected = true;
    check(rejected, "outside cutter wider than a U notch is rejected");
  }

  static function segmentDistance(point:toolpathkit.path.Point3,
      a:toolpathkit.path.Point3, b:toolpathkit.path.Point3):Float {
    var dx = b.x - a.x, dy = b.y - a.y;
    var t = Math.max(0.0, Math.min(1.0,
      ((point.x - a.x) * dx + (point.y - a.y) * dy) / (dx * dx + dy * dy)));
    return Math.sqrt(Math.pow(point.x - a.x - t * dx, 2) +
      Math.pow(point.y - a.y - t * dy, 2));
  }

  static function roundedPlate(check:Bool->String->Void):Void {
    var r = 5.0, diagonal = r / Math.sqrt(2.0);
    var pieces = [
      Curve.line(new Vector(5, 0), new Vector(35, 0)),
      Curve.arc(new Vector(35, 0), new Vector(35 + diagonal, 5 - diagonal), new Vector(40, 5)),
      Curve.line(new Vector(40, 5), new Vector(40, 15)),
      Curve.arc(new Vector(40, 15), new Vector(35 + diagonal, 15 + diagonal), new Vector(35, 20)),
      Curve.line(new Vector(35, 20), new Vector(5, 20)),
      Curve.arc(new Vector(5, 20), new Vector(5 - diagonal, 15 + diagonal), new Vector(0, 15)),
      Curve.line(new Vector(0, 15), new Vector(0, 5)),
      Curve.arc(new Vector(0, 5), new Vector(5 - diagonal, 5 - diagonal), new Vector(5, 0))
    ];
    var outline = Curve.wire(pieces);
    var sketch = Sketch.face(outline);
    var face = sketch.shape.faces().at(0);
    var tolerance = 0.00002;
    var contour = CamContour.fromFace(face, "mm", tolerance);
    check(contour.vertices.length > 8, "rounded plate samples CAD arcs");
    for (i in 0...contour.vertices.length) {
      var a = contour.vertices[i], b = contour.vertices[(i + 1) % contour.vertices.length];
      var midpointX = (a.x + b.x) * 0.5;
      var midpointY = (a.y + b.y) * 0.5;
      var cx = midpointX < 0.005 ? 0.005 : (midpointX > 0.035 ? 0.035 : midpointX);
      var cy = midpointY < 0.005 ? 0.005 : (midpointY > 0.015 ? 0.015 : midpointY);
      var corner = (midpointX < 0.005 || midpointX > 0.035) &&
        (midpointY < 0.005 || midpointY > 0.015);
      if (corner) {
        var distance = Math.sqrt(Math.pow(midpointX - cx, 2) + Math.pow(midpointY - cy, 2));
        check(Math.abs(distance - 0.005) <= tolerance + 1e-8,
          "sampled rounded corner meets chord tolerance");
      }
    }
    face.close(); sketch.close(); outline.close();
    for (piece in pieces) piece.close();
  }

  static function plateWithHole(check:Bool->String->Void):Void {
    var outer = Curve.polyline([new Vector(0, 0), new Vector(40, 0),
      new Vector(40, 30), new Vector(0, 30)], true);
    var hole = Curve.circle(5.0,
      new Plane(new Vector(20, 15), Vector.X(), Vector.Z()));
    var smallerHole = Curve.circle(3.0,
      new Plane(new Vector(8, 15), Vector.X(), Vector.Z()));
    var sketch = Sketch.face(outer, [hole, smallerHole]);
    var face = sketch.shape.faces().at(0);
    var boundaries = CamContour.fromFaceBoundaries(face, "mm", 0.000001);
    check(boundaries.length == 3, "CAD face exposes outer and both hole contours");
    check(Math.abs(Math.abs(boundaries[0].signedArea) - 0.0012) < 1e-8,
      "outer face boundary is first");
    check(Math.abs(Math.abs(boundaries[1].signedArea) - Math.PI * 0.005 * 0.005) < 1e-8,
      'inner face boundary follows the authored five-millimetre radius: ${boundaries[1].signedArea}');
    check(Math.abs(Math.abs(boundaries[2].signedArea) - Math.PI * 0.003 * 0.003) < 2e-8,
      'second inner boundary follows the authored three-millimetre radius: ${boundaries[2].signedArea}');
    var rejected = false;
    try CamContour.fromFace(face) catch (_:Dynamic) rejected = true;
    check(rejected, "single-contour face adapter rejects a face with holes");

    var invalidJob = new CamJob(0.005, 10000);
    rejected = false;
    try invalidJob.profileFace(face, new Tool(4, 0, 0.008),
      -0.002, 0.005, 0.002, "mm", 0.000001)
    catch (_:Dynamic) rejected = true;
    check(rejected, "face operation rejects a tool too large for an inner hole");
    rejected = false;
    try invalidJob.finish() catch (_:Dynamic) rejected = true;
    check(rejected, "rejected face leaves the CAM job without partial cuts");

    var tool = new Tool(3, 0, 0.002);
    var program = new CamJob(0.005, 10000)
      .profileFace(face, tool, -0.002, 0.005, 0.002, "mm", 0.000001)
      .finish();
    var holeFeeds = 0, lastHoleIndex = -1, firstOutsideIndex = program.ops.length;
    for (index in 0...program.ops.length) switch program.ops[index] {
      case Move(Cut, geometry, _, _, span) if (span.line == 1 || span.line == 2):
        var start = GeometryTools.pointAt(geometry, 0);
        var end = GeometryTools.pointAt(geometry, GeometryTools.length(geometry));
        if (Math.abs(start.z - end.z) > 1e-9) continue;
        holeFeeds++;
        lastHoleIndex = index;
        var cx = span.line == 1 ? 0.02 : 0.008;
        var expectedRadius = span.line == 1 ? 0.004 : 0.002;
        for (fraction in [0.0, 0.5, 1.0]) {
          var point = GeometryTools.pointAt(geometry,
            GeometryTools.length(geometry) * fraction);
          var radius = Math.sqrt(Math.pow(point.x - cx, 2) +
            Math.pow(point.y - 0.015, 2));
          check(Math.abs(radius - expectedRadius) < 0.00002,
            "hole cutter centre follows the inner offset");
        }
      case Move(Cut, _, _, _, span) if (span.line == 3):
        if (index < firstOutsideIndex) firstOutsideIndex = index;
      case _:
    }
    check(holeFeeds > 16, "both holes get separate inside profiles");
    check(lastHoleIndex < firstOutsideIndex && firstOutsideIndex < program.ops.length,
      "face operation finishes both holes before its outer profile");
    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    machine.setTool(tool);
    check(CamTestLowering.lower(program, machine).diagnostics.length == 0,
      "holed plate lowers to MotionKit");
    var imported = new CncCompiler(machine).compileDetailed(CamGCodeWriter.write(program, CamTestSetup.standard(), machine));
    check(imported.diagnostics.length == 0 && imported.ops.length == program.ops.length,
      "holed plate G-code round trips through CncKit");
    for (index in 0...program.ops.length) switch [program.ops[index], imported.ops[index]] {
      case [Move(Cut, a, _, _, _), Move(Cut, b, _, _, _)],
           [Move(_, a, _, _, _), Move(_, b, _, _, _)]:
        for (fraction in [0.0, 0.5, 1.0]) {
          var original = GeometryTools.pointAt(a,
            GeometryTools.length(a) * fraction);
          var reparsed = GeometryTools.pointAt(b,
            GeometryTools.length(b) * fraction);
          check(original.distanceTo(reparsed) < 1e-8,
            "holed plate G-code preserves cutter path geometry");
        }
      case _:
    }
    face.close(); sketch.close(); outer.close(); hole.close(); smallerHole.close();
  }
}
