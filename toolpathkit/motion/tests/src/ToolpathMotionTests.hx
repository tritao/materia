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
    Sys.println("ToolpathKit Motion tests passed (4 assertions)");
  }
}
