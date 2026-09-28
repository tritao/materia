import camkit.CamContour;
import camkit.CamGCodeWriter;
import camkit.CamJob;
import cadkit.modeling.Curve;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cnckit.CncCompiler;
import cnckit.CncMachine;
import cnckit.CncTool;
import cnckit.ir.CncGeometryTools;
import cnckit.ir.CncOp;

/** CAD fixtures authored locally so CamKit tests have no example-project dependency. */
class CamGeneratedFixtures {
  public static function run(check:Bool->String->Void):Void {
    roundedPlate(check);
    plateWithHole(check);
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
    try invalidJob.profileFace(face, new CncTool(4, 0, 0.008),
      -0.002, 0.005, 0.002, "mm", 0.000001)
    catch (_:Dynamic) rejected = true;
    check(rejected, "face operation rejects a tool too large for an inner hole");
    rejected = false;
    try invalidJob.finish() catch (_:Dynamic) rejected = true;
    check(rejected, "rejected face leaves the CAM job without partial cuts");

    var tool = new CncTool(3, 0, 0.002);
    var program = new CamJob(0.005, 10000)
      .profileFace(face, tool, -0.002, 0.005, 0.002, "mm", 0.000001)
      .finish();
    var holeFeeds = 0, lastHoleIndex = -1, firstOutsideIndex = program.ops.length;
    for (index in 0...program.ops.length) switch program.ops[index] {
      case Feed(geometry, _, _, span) if (span.line == 1 || span.line == 2):
        var start = CncGeometryTools.pointAt(geometry, 0);
        var end = CncGeometryTools.pointAt(geometry, CncGeometryTools.length(geometry));
        if (Math.abs(start.z - end.z) > 1e-9) continue;
        holeFeeds++;
        lastHoleIndex = index;
        var cx = span.line == 1 ? 0.02 : 0.008;
        var expectedRadius = span.line == 1 ? 0.004 : 0.002;
        for (fraction in [0.0, 0.5, 1.0]) {
          var point = CncGeometryTools.pointAt(geometry,
            CncGeometryTools.length(geometry) * fraction);
          var radius = Math.sqrt(Math.pow(point.x - cx, 2) +
            Math.pow(point.y - 0.015, 2));
          check(Math.abs(radius - expectedRadius) < 0.00002,
            "hole cutter centre follows the inner offset");
        }
      case Feed(_, _, _, span) if (span.line == 3):
        if (index < firstOutsideIndex) firstOutsideIndex = index;
      case _:
    }
    check(holeFeeds > 16, "both holes get separate inside profiles");
    check(lastHoleIndex < firstOutsideIndex && firstOutsideIndex < program.ops.length,
      "face operation finishes both holes before its outer profile");
    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    machine.setTool(tool);
    check(program.lower(machine).diagnostics.length == 0,
      "holed plate lowers to MotionKit");
    var imported = new CncCompiler(machine).compileDetailed(CamGCodeWriter.write(program));
    check(imported.diagnostics.length == 0 && imported.ops.length == program.ops.length,
      "holed plate G-code round trips through CncKit");
    for (index in 0...program.ops.length) switch [program.ops[index], imported.ops[index]] {
      case [Feed(a, _, _, _), Feed(b, _, _, _)],
           [Rapid(a, _), Rapid(b, _)]:
        for (fraction in [0.0, 0.5, 1.0]) {
          var original = CncGeometryTools.pointAt(a,
            CncGeometryTools.length(a) * fraction);
          var reparsed = CncGeometryTools.pointAt(b,
            CncGeometryTools.length(b) * fraction);
          check(original.distanceTo(reparsed) < 1e-8,
            "holed plate G-code preserves cutter path geometry");
        }
      case _:
    }
    face.close(); sketch.close(); outer.close(); hole.close(); smallerHole.close();
  }
}
