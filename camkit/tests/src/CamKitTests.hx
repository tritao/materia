import camkit.CamContour;
import camkit.CamGCodeWriter;
import camkit.CamJob;
import camkit.CamSheetProfiles;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cnckit.CncCompiler;
import cnckit.CncMachine;
import cnckit.CncTool;
import cnckit.ir.CncGeometry;
import cnckit.ir.CncGeometryTools;
import cnckit.ir.CncOp;
import cnckit.ir.CncPoint;

class CamKitTests {
  static var assertions = 0;
  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function near(actual:Float, expected:Float, message:String,
      ?tolerance:Float = 1e-8):Void
    check(Math.abs(actual - expected) <= tolerance,
      '$message: expected $expected, got $actual');

  public static function main():Void {
    var placement:materia.sheet.SheetPartPlacement = {
      id: "gantry-bracket", requirementId: "bracket", x: 10.0, y: 10.0,
      width: 20.0, height: 10.0, rotation: 0, regionId: "sheet-a"
    };
    var contour = CamSheetProfiles.placement(placement, "mm");
    var sheetPlan:materia.sheet.SheetCutPlan = {
      schemaVersion: 1, id: "gantry-sheet", revision: 1,
      stockSpecId: "aluminium", stockSpecRevision: 1,
      stockSpecFingerprint: "fixture", lengthUnit: "mm", kerf: 2.0,
      edgeMargin: 0.0, requirements: [], placements: [placement],
      cuts: [], retainedRegionIds: []
    };
    near(CamSheetProfiles.fromPlan(sheetPlan, "gantry-bracket").signedArea,
      contour.signedArea, "manufacturingkit sheet plan feeds CAM directly");
    near(contour.vertices[0].x, 0.01, "sheet placement X becomes metres");
    near(contour.signedArea, 0.0002, "sheet rectangle area");
    var sketch = new ConstrainedSketch();
    var coordinates = [[10.0, 10.0], [30.0, 10.0],
      [30.0, 20.0], [10.0, 20.0]];
    for (index in 0...4) {
      sketch.addPoint(new SketchPoint('p$index', coordinates[index][0],
        coordinates[index][1]));
      sketch.addConstraint(SketchConstraint.fixed('fixed$index', 'p$index'));
      sketch.addEntity(SketchEntity.line('edge$index', 'p$index',
        'p${(index + 1) % 4}'));
    }
    var fromSketch = CamContour.fromSketch(sketch, sketch.solve());
    near(fromSketch.signedArea, contour.signedArea,
      "solved cadkit sketch becomes a CAM contour");
    var cadFace = Sketch.polygon([new Vector(10, 10), new Vector(30, 10),
      new Vector(30, 20), new Vector(10, 20)]);
    var face = cadFace.shape.faces().at(0);
    var fromFace = CamContour.fromFace(face);
    near(Math.abs(fromFace.signedArea), Math.abs(contour.signedArea),
      "cadkit face edges become a CAM contour");
    face.close(); cadFace.close();

    var tool = new CncTool(2, 0.0, 0.002);
    var job = new CamJob(0.005, 12000.0);
    job.profile(contour, tool, -0.002, 0.01);
    job.pocket(contour, tool, -0.001, 0.01, 0.001);
    job.drill([new CncPoint(0.015, 0.015, 0.0),
      new CncPoint(0.025, 0.015, 0.0)], tool, -0.003, 0.001, 0.005);
    var program = job.finish();
    check(program.ops.length > 30,
      "profile, pocket rings, and drills create ordered CNC geometry");
    var outsideArcs = 0;
    for (op in program.ops) switch op {
      case CncOp.Feed(CncGeometry.Arc(_, radius, _, sweep), _, _, span):
        if (span.line == 1) {
          outsideArcs++;
          near(radius, 0.001, "outside profile joins by cutter radius");
          near(sweep, Math.PI * 0.5, "outside profile corner sweep");
        }
      case _:
    }
    check(outsideArcs == 4,
      "rectangular outside profile has one round join per corner");
    var stepped = new CamJob(0.005, 12000.0)
      .profile(contour, tool, -0.005, 0.01, "outside", 0.002).finish();
    var levels:Array<Float> = [];
    for (op in stepped.ops) switch op {
      case CncOp.Feed(CncGeometry.Arc(center, _, _, _), _, _, _):
        if (levels.length == 0 || Math.abs(center.z - levels[levels.length - 1]) > 1e-9)
          levels.push(center.z);
      case _:
    }
    check(levels.length == 3, "5 mm profile uses three depth levels");
    near(levels[0], -0.002, "first profile depth");
    near(levels[1], -0.004, "second profile depth");
    near(levels[2], -0.005, "final profile depth");
    for (x in 11...30) for (y in 11...20) {
      var sample = new CncPoint(x * 0.001, y * 0.001, -0.001);
      var best = Math.POSITIVE_INFINITY;
      for (op in program.ops) switch op {
        case CncOp.Feed(CncGeometry.Line(a, b), _, _, span):
          if (span.line == 2 && Math.abs(a.z + 0.001) < 1e-9 &&
              Math.abs(b.z + 0.001) < 1e-9)
            best = Math.min(best, distanceToLine(sample, a, b));
        case _:
      }
      check(best <= tool.diameter * 0.5 + 1e-8,
        'pocket clears grid point $x,$y within cutter radius');
    }
    var concave = new CamContour([new CncPoint(0, 0, 0),
      new CncPoint(0.02, 0, 0), new CncPoint(0.02, 0.01, 0),
      new CncPoint(0.01, 0.005, 0), new CncPoint(0, 0.01, 0)]);
    var rejected = "";
    try new CamJob(0.005, 12000.0).pocket(concave, tool, -0.001, 0.01, 0.001)
    catch (error:Dynamic) rejected = Std.string(error);
    check(rejected.indexOf("convex contour") >= 0,
      "concave offset clearing rejects geometry it cannot clear safely");
    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    machine.setTool(tool);
    var lowered = program.lower(machine);
    check(lowered.diagnostics.length == 0 && lowered.program != null,
      "CAM IR lowers directly to executable MotionKit paths");
    var tight = new CncMachine("work", "x", "y", "z", 0.2);
    tight.setTravelEnvelope([0.0, 0.0, -0.01], [0.02, 0.02, 0.01]);
    var overTravel = program.lower(tight);
    check(overTravel.diagnostics.length > 0 &&
      overTravel.diagnostics[0].code == "CNC_TRAVEL" &&
      overTravel.diagnostics[0].span.line == 1,
      "direct CAM lowering checks machine travel with operation source span");
    var hasProfileSpan = false, hasPocketSpan = false, hasDrillSpan = false;
    for (entry in lowered.sourceMap.entries) {
      if (entry.span.line == 1) hasProfileSpan = true;
      if (entry.span.line == 2) hasPocketSpan = true;
      if (entry.span.line == 3) hasDrillSpan = true;
    }
    check(hasProfileSpan && hasPocketSpan && hasDrillSpan,
      "CAM source map identifies each authored operation");
    var gcode = CamGCodeWriter.write(program);
    var parsed = new CncCompiler(machine).compileDetailed(gcode);
    check(parsed.diagnostics.length == 0,
      'CAM G-code recompiles: ${parsed.diagnostics}');
    check(parsed.ops.length == program.ops.length,
      "CAM round trip keeps the operation count");
    for (index in 0...program.ops.length)
      equalOp(program.ops[index], parsed.ops[index], index);
    check(gcode.indexOf("T2 M6") >= 0 &&
      gcode.indexOf("S12000 M3") >= 0 && gcode.indexOf("M5") >= 0 &&
      gcode.indexOf("M2") >= 0,
      "LinuxCNC export includes tool, spindle, and program commands");
    CamGeneratedFixtures.run(check);
    Sys.println('CamKit tests passed ($assertions assertions)');
  }

  static function equalOp(left:CncOp, right:CncOp, index:Int):Void {
    switch [left, right] {
      case [CncOp.Rapid(a, _), CncOp.Rapid(b, _)]: equalGeometry(a, b, index);
      case [CncOp.Feed(a, speedA, blendA, _),
            CncOp.Feed(b, speedB, blendB, _)]:
        equalGeometry(a, b, index);
        near(speedA, speedB, 'feed $index', 1e-8);
        near(blendA, blendB, 'blend $index');
      case [CncOp.ToolChange(a, _), CncOp.ToolChange(b, _)]:
        check(a == b, 'tool change $index');
      case [CncOp.Spindle(channelA, valueA, _),
            CncOp.Spindle(channelB, valueB, _)]:
        check(channelA == channelB, 'spindle channel $index');
        near(valueA, valueB, 'spindle value $index');
      case [CncOp.End(_), CncOp.End(_)]: check(true, 'end $index');
      case _: throw 'CAM round trip changed operation $index';
    }
  }

  static function equalGeometry(a:CncGeometry, b:CncGeometry, index:Int):Void {
    near(CncGeometryTools.length(a), CncGeometryTools.length(b),
      'geometry length $index', 1e-8);
    for (fraction in [0.0, 0.25, 0.5, 0.75, 1.0]) {
      var first = CncGeometryTools.pointAt(a, CncGeometryTools.length(a) * fraction);
      var second = CncGeometryTools.pointAt(b, CncGeometryTools.length(b) * fraction);
      near(first.distanceTo(second), 0.0,
        'geometry sample $index/$fraction', 1e-8);
    }
  }

  static function distanceToLine(point:CncPoint, a:CncPoint, b:CncPoint):Float {
    var dx = b.x - a.x, dy = b.y - a.y;
    var denominator = dx * dx + dy * dy;
    var fraction = denominator == 0.0 ? 0.0 : Math.max(0.0, Math.min(1.0,
      ((point.x - a.x) * dx + (point.y - a.y) * dy) / denominator));
    return point.distanceTo(new CncPoint(a.x + fraction * dx,
      a.y + fraction * dy, point.z));
  }
}
