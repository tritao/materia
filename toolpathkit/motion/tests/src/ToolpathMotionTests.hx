import toolpathkit.motion.ToolpathMotion;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.ToolpathLowering;
import toolpathkit.motion.ToolpathMotionBinding;
import toolpathkit.path.MoveKind;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.setup.Setup;
import toolpathkit.setup.TravelEnvelope;
import toolpathkit.tool.ToolLibrary;
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
  static function pathProgram(ops:Array<ToolpathOp>):ToolpathProgram
    return new ToolpathProgram(ops, new ToolLibrary(),
      [new Setup("1", new Point3(0, 0, 0))]);

  static function main():Void {
    acceptsBinding(null);
    var origin = Provenance.cam(7, "face:2");
    var machine = new MachineBinding("work", "x", "y", "z", 0.2);
    var result = ToolpathMotion.lower(pathProgram([
      ToolpathOp.Spindle(Clockwise, 12000, origin),
      ToolpathOp.Move(Cut,
        PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
        0.01, 0, origin),
      ToolpathOp.Coolant(false, false, origin),
      ToolpathOp.End(origin)
    ]), machine);
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
    machine.setTravel(new TravelEnvelope(new Point3(-0.01, -0.01, -0.01),
      new Point3(0.005, 0.01, 0.01)));
    var rejected = false;
    try ToolpathMotion.lower(pathProgram([
      ToolpathOp.Move(Cut,
        PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
        0.01, 0, origin)
    ]), machine) catch (error:Dynamic) {
      rejected = Std.string(error).indexOf("X travel") >= 0;
    }
    if (!rejected) throw "machine travel was not checked";
    machine.setTravel(null);
    var probeOps = [ToolpathOp.Move(Cut,
      PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
      0.01, 0.0, origin)];
    function endX(workOrigin:Point3):Float {
      var placed = new ToolpathProgram(probeOps, new ToolLibrary(),
        [new Setup("1", workOrigin)]);
      var lowered = ToolpathMotion.lower(placed, machine);
      var motion:motionkit.program.MotionProgram = cast lowered.program;
      if (motion == null) throw "probed setup produced no motion";
      for (op in motion.ops) switch op {
        case MotionOp.FollowPath(path, _, _, _):
          return path.poseAt(path.length()).x;
        case _:
      }
      throw "probed setup produced no path";
    }
    if (Math.abs(endX(new Point3(0.003, 0, 0)) -
        endX(new Point3(0, 0, 0)) - 0.003) > 1e-9)
      throw "probed origin must move the lowered path by the probe difference";
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 200), new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 200), 0.1, 0.4);
    var robotBinding = new ToolpathMotionBinding(
      new MachineBinding("work", "x", "y", "z", 0.08), blueprint);
    var compiled = robotBinding.compile(pathProgram([
      ToolpathOp.Move(Cut,
        PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
        0.01, 0, Provenance.cam(8))
    ]), [0.0, 0.0, 0.0], Int64.ofInt(1));
    if (compiled.blocks.length == 0) throw "robot binding produced no plans";
    compiled.dispose();
    // A rapid down that carries straight on as a slower plunge is one motion; the exact
    // corner after it is where the machine stops.
    var entryOps = [
      ToolpathOp.Move(Rapid, PathGeometry.Line(new Point3(0, 0, 0.012), new Point3(0, 0, 0.006)),
        0.0, 0.0, Provenance.cam(9)),
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0, 0, 0.006), new Point3(0, 0, 0.002)),
        0.002, 0.0, Provenance.cam(10)),
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0, 0, 0.002), new Point3(0.01, 0, 0.002)),
        0.01, 0.0, Provenance.cam(11))
    ];
    var entry:motionkit.program.MotionProgram = cast ToolpathMotion.lower(pathProgram(entryOps),
      robotBinding.machine).program;
    if (entry == null || entry.ops.length != 1) throw "moves without a barrier must lower to one path";
    switch entry.ops[0] {
      case MotionOp.FollowPath(path, _, feed, _):
        var speeds = [for (primitive in path.primitives) primitive.speedLimit()];
        if (speeds.join(",") != "0.08,0.002,0.01" || feed != 0.08)
          throw 'each move keeps its own speed, got $speeds under $feed';
      case _: throw "entry path missing";
    }
    var entryPlans = robotBinding.compile(pathProgram(entryOps), [0.0, 0.0, 0.012], Int64.ofInt(1));
    var planCount = 0;
    for (block in entryPlans.blocks) planCount += block.plans.length;
    entryPlans.dispose();
    if (planCount != 2) throw 'the plunge must follow the rapid without stopping, got $planCount plans';
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
    var eventResult = ToolpathMotion.lower(pathProgram([
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0, 0, 0),
        new Point3(0.01, 0, 0)), 0.01, 0.0002, origin),
      ToolpathOp.Coolant(true, false, origin),
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0.01, 0, 0),
        new Point3(0.01, 0.01, 0)), 0.01, 0.0004, origin)
    ]), new MachineBinding("work", "x", "y", "z", 0.2));
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
    Sys.println("ToolpathKit Motion tests passed (12 assertions)");
    Sys.println('Toolpath accuracy tests passed (${ToolpathAccuracyTests.run()} assertions)');
    MachiningRunTests.run();
    ToolpathScenarioTests.main();
  }
}
