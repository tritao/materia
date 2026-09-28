import toolpathkit.motion.ToolpathMotion;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.ToolpathLowering;
import toolpathkit.motion.ToolpathMotionBinding;
import toolpathkit.path.MoveKind;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import motionkit.program.MotionOp;
import motionkit.path.CornerBlender;
import motionkit.path.GeometricPath;
import motionkit.path.PathPoint;
import motionkit.path.ArcSegment;
import machinekit.assembly.LinearAxis;
import motionkit.robot.MachineKitRobotCompiler;
import haxe.Int64;

class ToolpathMotionTests {
  static function acceptsBinding(binding:ToolpathMotionBinding):Void {}

  static function main():Void {
    acceptsBinding(null);
    var origin = Provenance.cam(7, "face:2");
    var machine = new MachineBinding("work", "x", "y", "z", 0.2);
    var result = ToolpathMotion.lower([
      ToolpathOp.Spindle(Clockwise, 12000, origin),
      ToolpathOp.Move(Cut,
        PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
        0.01, 0, origin),
      ToolpathOp.Coolant(false, false, origin),
      ToolpathOp.End(origin)
    ], machine);
    if (result.program == null || result.diagnostics.length != 0)
      throw "lowering failed";
    var pathIndex = -1;
    for (index in 0...result.program.ops.length) switch result.program.ops[index] {
      case MotionOp.FollowPath(_, _, _, _): pathIndex = index;
      case _:
    }
    if (pathIndex < 0 || result.sourceMap.provenanceAt(pathIndex, 0.005) != origin ||
        result.sourceMap.entriesFor(origin).length < 3)
      throw "path provenance lost";
    machine.setTravelEnvelope([-0.01, -0.01, -0.01],
      [0.005, 0.01, 0.01]);
    var rejected = false;
    try ToolpathMotion.lower([
      ToolpathOp.Move(Cut,
        PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
        0.01, 0, origin)
    ], machine) catch (error:Dynamic) {
      rejected = Std.string(error).indexOf("X travel") >= 0;
    }
    if (!rejected) throw "machine travel was not checked";
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 200), new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 200), 0.1, 0.4);
    var robotBinding = new ToolpathMotionBinding(
      new MachineBinding("work", "x", "y", "z", 0.08), blueprint);
    var compiled = robotBinding.compile([
      ToolpathOp.Move(Cut,
        PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
        0.01, 0, Provenance.cam(8))
    ], [0.0, 0.0, 0.0], Int64.ofInt(1));
    if (compiled.blocks.length == 0) throw "robot binding produced no plans";
    compiled.dispose();
    var corners = CornerBlender.blendPerCorner(GeometricPath.lines([
      new PathPoint(0.0, 0.0), new PathPoint(0.01, 0.0),
      new PathPoint(0.01, 0.01), new PathPoint(0.02, 0.01)
    ]), [0.0001, 0.0005], Math.PI * 5.0 / 6.0);
    if (corners.path.primitives.length != 5)
      throw "both per-corner tolerances must blend";
    var first:ArcSegment = cast corners.path.primitives[1];
    var second:ArcSegment = cast corners.path.primitives[3];
    if (first.radius >= second.radius)
      throw "smaller corner tolerance must create a tighter fillet";
    var eventResult = ToolpathMotion.lower([
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0, 0, 0),
        new Point3(0.01, 0, 0)), 0.01, 0.0002, origin),
      ToolpathOp.Coolant(true, false, origin),
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0.01, 0, 0),
        new Point3(0.01, 0.01, 0)), 0.01, 0.0004, origin)
    ], new MachineBinding("work", "x", "y", "z", 0.2));
    var eventProgram:motionkit.program.MotionProgram = cast eventResult.program;
    if (eventProgram == null || eventProgram.ops.length != 1)
      throw "coolant added a motion stop";
    switch eventProgram.ops[0] {
      case MotionOp.FollowPath(_, _, _, events):
        if (events.length != 2 || events[0].channel != "coolant.mist" ||
            events[0].distance <= 0.0)
          throw "coolant must be a position-tied path event";
      case _: throw "blended path missing";
    }
    Sys.println("ToolpathKit Motion tests passed (8 assertions)");
    MachiningRunTests.run();
    ToolpathScenarioTests.main();
  }
}
