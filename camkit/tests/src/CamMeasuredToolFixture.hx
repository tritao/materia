import camkit.CamContour;
import camkit.CamJob;
import motionkit.program.MotionOp;
import stockkit.CutMoves;
import toolpathkit.tool.Tool;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Point3;

/**
  Measured tools are programmed at their tips through G43, while the machine
  moves its spindle gauge line the tool length above them.
**/
class CamMeasuredToolFixture {
  public static function run(check:Bool->String->Void):Void {
    var contour = new CamContour([new Point3(0.01, 0.01, 0),
      new Point3(0.03, 0.01, 0),
      new Point3(0.03, 0.02, 0),
      new Point3(0.01, 0.02, 0)]);
    var mill = new Tool(31, 0.04, 0.002);
    var drill = new Tool(32, 0.06, 0.001);
    var unmeasured = new Tool(33, 0.0, 0.002);
    // The spindle starts 50 mm up with no offset active.
    var job = new CamJob(0.005, 10000, new Point3(0.002, 0.002, 0.05));
    var program = job
      .profile(contour, mill, -0.003, 0.005, "outside", 0.002, 0.001)
      .drill([new Point3(0.018, 0.015, 0)], drill, -0.004, 0.001, 0.0008)
      .profile(contour, unmeasured, -0.001, 0.005, "inside", 0.002, 0.001)
      .finish();

    var offsets:Array<String> = [], lowest = 1e9;
    var afterOffset:Null<Point3> = null;
    for (op in program.ops) switch op {
      case ToolLengthOffset(number, length, _): offsets.push('$number:$length');
      case Move(_, geometry, _, _, _):
        if (afterOffset == null && offsets.length == 1)
          afterOffset = GeometryTools.pointAt(geometry, 0.0);
        lowest = Math.min(lowest, GeometryTools.pointAt(geometry,
          GeometryTools.length(geometry)).z);
      case _:
    }
    check(offsets.join(",") == "31:0.04,32:0.06,0:0",
      'measured tools select G43 and an unmeasured one G49: $offsets');
    // The first tool is loaded where the spindle starts; G43 re-expresses that point at its tip.
    check(afterOffset != null && Math.abs(afterOffset.z - (0.05 - 0.04)) < 1e-12,
      "the untouched spindle is re-expressed at the new tool's tip");
    // Before the 20 mm longer drill goes in, the mill retracts 20 mm above safe Z.
    var drillChange = -1, highest = -1e9;
    for (index in 0...program.ops.length) switch program.ops[index] {
      case ToolChange(32, _): drillChange = index;
      case _:
    }
    for (index in 0...drillChange) switch program.ops[index] {
      case Move(_, geometry, _, _, _):
        highest = GeometryTools.pointAt(geometry, GeometryTools.length(geometry)).z;
      case _:
    }
    check(Math.abs(highest - (0.005 + 0.02)) < 1e-12,
      'the mill clears the longer drill\'s extra length before the change, at $highest');
    check(Math.abs(lowest + 0.004) < 1e-12, "drill depth is programmed at the tip");

    var tips = CutMoves.fromProgram(program), lowestTip = 1e9;
    for (move in tips) switch move.motion {
      case Path(geometry):
        lowestTip = Math.min(lowestTip, GeometryTools.pointAt(geometry,
          GeometryTools.length(geometry)).z);
      case _:
    }
    check(Math.abs(lowestTip + 0.004) < 1e-12, "stock simulation cuts at the programmed tip");

    var machine = new CamTestRig();
    for (tool in [mill, drill, unmeasured]) machine.toolLibrary.set(tool);
    var lowered = CamTestLowering.lower(program, machine);
    var motion = lowered.program;
    if (lowered.diagnostics.length > 0 || motion == null)
      throw 'measured-tool job lowers through MotionKit: ${lowered.diagnostics}';
    var gaugeStart = Math.NaN, previousEnd = Math.NaN;
    var ends:Array<Float> = [];
    var continuous = true;
    for (op in motion.ops) switch op {
      case MotionOp.FollowPath(path, _, _, _):
        var start = path.poseAt(0.0).z;
        if (Math.isNaN(gaugeStart)) gaugeStart = start;
        if (!Math.isNaN(previousEnd) && Math.abs(start - previousEnd) > 1e-12)
          continuous = false;
        previousEnd = path.poseAt(path.length()).z;
        // Moves between barriers share a path, so each move ends at a primitive's end.
        for (primitive in path.primitives) ends.push(primitive.waypointAt(primitive.length()).pose.z);
      case _:
    }
    check(continuous, "tool changes keep the spindle continuous in machine space");
    check(Math.abs(gaugeStart - 0.05) < 1e-12,
      "the machine starts from where the spindle stood");
    function reaches(z:Float):Bool {
      for (end in ends) if (Math.abs(end - z) < 1e-12) return true;
      return false;
    }
    check(reaches(-0.003 + 0.04) && reaches(-0.004 + 0.06) && reaches(-0.001),
      "the machine drives the gauge line one tool length above each tip");

    var imported = machine.compileDetailed(
      machine.export(program, CamTestSetup.standard()));
    check(imported.diagnostics.length == 0 &&
      imported.program.ops.length == program.ops.length,
      'measured-tool G-code round trip keeps operation order: ${imported.diagnostics}');
    for (index in 0...program.ops.length) switch [program.ops[index], imported.program.ops[index]] {
      case [Move(_, a, _, _, _), Move(_, b, _, _, _)]:
        check(GeometryTools.pointAt(a, GeometryTools.length(a))
          .distanceTo(GeometryTools.pointAt(b, GeometryTools.length(b))) < 1e-8,
          "measured-tool G-code preserves programmed endpoints");
      case [ToolLengthOffset(a, lengthA, _), ToolLengthOffset(b, lengthB, _)]:
        check(a == b && Math.abs(lengthA - lengthB) < 1e-12,
          "measured-tool G-code preserves tool length offsets");
      case _:
    }

    // Blending lets a contour's cutting moves run on instead of stopping at every vertex: the
    // machine stops only at the sharp corners left in its paths.
    function paths(blend:Float):{count:Int, gcode:String} {
      var blended = new CamJob(0.005, 10000, new Point3(0.002, 0.002, 0.05), blend)
        .pocket(contour, mill, -0.002, 0.005, 0.0008, 0.001, 0.001).finish();
      var lowered = CamTestLowering.lower(blended, machine);
      var motion = lowered.program;
      if (motion == null) throw 'blended pocket lowers: ${lowered.diagnostics}';
      var corners = 0;
      for (op in motion.ops) switch op {
        case MotionOp.FollowPath(path, _, _, _):
          for (k in 1...path.primitives.length) {
            var before = path.primitives[k - 1];
            var arriving = before.derivativesAt(before.length()).linear;
            var leaving = path.primitives[k].derivativesAt(0.0).linear;
            if (Math.abs(arriving[0] - leaving[0]) + Math.abs(arriving[1] - leaving[1]) +
                Math.abs(arriving[2] - leaving[2]) > 1e-6) corners++;
          }
        case _:
      }
      return {count: corners, gcode: machine.export(blended, CamTestSetup.standard())};
    }
    var exact = paths(0.0), smooth = paths(0.00001);
    check(smooth.count < exact.count,
      'a blended pocket stops at fewer corners: ${smooth.count} against ${exact.count}');
    check(smooth.gcode.indexOf("G64 P0.01") >= 0 && exact.gcode.indexOf("G64") < 0,
      "the blend tolerance reaches the G-code as G64 P");
  }
}
