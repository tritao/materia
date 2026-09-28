import camkit.CamContour;

import toolpathkit.path.ToolpathOp;
import camkit.CamJob;
import cnckit.CncCompiler;
import cnckit.CncMachine;
import toolpathkit.tool.Tool;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.tool.CutterProfile;
import stockkit.CutMove;
import stockkit.CutMoves;

class CutMoveTests {
  public static function run():Void {
    tools();
    gcode();
    camProgram();
  }

  static function tools():Void {
    var ball = CutterProfile.ball(0.006, 0.02).withHolder(0.02, 0.03);
    var tool = Tool.shaped(4, 0.08, ball);
    Assert.near(tool.diameter, 0.006, "shaped tool takes its cutting diameter", 1e-15);
    Assert.check(tool.profile() == ball, "shaped tool simulates its own shape");
    Assert.check(tool.withLength(0.09).cutter == ball, "a new length keeps the shape");
    var rejected = false;
    try new Tool(4, 0.08, 0.008, ball) catch (_:Dynamic) rejected = true;
    Assert.check(rejected, "a diameter that contradicts the shape is rejected");

    var plain = new Tool(5, 0.0, 0.004);
    Assert.near(plain.profile().cuttingDiameter(), 0.004,
      "a diameter-only tool simulates as a flat mill", 1e-15);
    Assert.near(plain.profile().height(), Tool.DEFAULT_FLUTE_LENGTH,
      "with the default flute length when its length is unknown", 1e-15);
    Assert.near(new Tool(5, 0.03, 0.004).profile().height(), 0.03,
      "or flutes over its whole length", 1e-15);
    rejected = false;
    try new Tool(6, 0.0, 0.0).profile() catch (_:Dynamic) rejected = true;
    Assert.check(rejected, "a tool with no size has nothing to simulate");

    var machine = new CncMachine("cnc", "x", "y", "z", 0.1);
    machine.toolLibrary.set(tool);
    machine.controller.setToolLength(4, 0.07);
    Assert.check(machine.toolLibrary.tool(4).cutter == ball, "setting a tool length keeps its shape");
  }

  static function gcode():Void {
    var machine = new CncMachine("cnc", "x", "y", "z", 0.1);
    machine.controller.setWorkOffset(54, 0.1, 0.2, 0.3);
    var ball = CutterProfile.ball(0.006, 0.02);
    machine.toolLibrary.set(Tool.shaped(1, 0.05, ball));
    var compiled = new CncCompiler(machine).compileDetailed([
      "G21 G90 G54",
      "G0 X0 Y0 Z50",
      "T1 M6",
      "G43 H1",
      "G0 X10 Y5 Z5",
      "G1 Z-2 F100",
      "G1 X20",
      "G2 X30 Y5 I5 J0",
      "G49",
      "G0 Z10",
      "M2"
    ].join("\n"));
    var moves = CutMoves.fromOps(compiled.ops, machine.toolLibrary.tool);
    Assert.check(moves.length == 5, "moves before the first tool change are skipped");
    Assert.check([for (move in moves) move.span.line].join(",") == "5,6,7,8,10",
      "each move keeps its source line");
    Assert.check(moves[0].rapid && !moves[1].rapid && moves[4].rapid,
      "rapid and feed moves are told apart");
    Assert.check(moves[1].tool.profile() == ball, "moves carry the loaded tool");
    for (move in moves)
      Assert.check(move.opIndex >= 0 && move.opIndex < compiled.ops.length,
        "moves point at their source op");
    var tip = end(moves[1]);
    Assert.near(tip.x, 0.01, "tip X is in the workpiece frame");
    Assert.near(tip.y, 0.005, "tip Y is in the workpiece frame");
    Assert.near(tip.z, -0.002, "G43 tool length is removed from the tip height");
    switch moves[3].motion {
      case Path(Arc(center, radius, _, sweep)):
        Assert.near(center.x, 0.025, "arc centre in the workpiece frame");
        Assert.near(center.z, -0.002, "arc height is the tip height");
        Assert.near(radius, 0.005, "arc radius");
        Assert.check(sweep < 0, "G2 sweeps clockwise");
      case _: Assert.check(false, "G2 becomes an arc move");
    }
    Assert.near(end(moves[4]).z, 0.01, "after G49 the programmed Z is the tip");

    var unresolved = new CncCompiler(machine).compileDetailed("G40");
    Assert.check(unresolved.diagnostics.length > 0,
      "unresolved cutter compensation is rejected before shared toolpaths");
  }

  static function camProgram():Void {
    var contour = new CamContour([
      new Point3(0.01, 0.005, 0), new Point3(0.03, 0.005, 0),
      new Point3(0.03, 0.015, 0), new Point3(0.01, 0.015, 0)
    ]);
    var small = new Tool(2, 0.0, 0.002);
    var large = Tool.shaped(3, 0.0, CutterProfile.bullNose(0.006, 0.001, 0.02));
    var program = new CamJob(0.005, 12000)
      .pocket(contour, small, -0.002, 0.01, 0.0015, 0.001)
      .profile(contour, large, -0.003, 0.01)
      .finish();
    Assert.check(program.tool(3) == large, "CAM programs keep the tools they use");
    // CAM retracts before each tool change; the first retract has no tool loaded.
    var motions = 0, loaded = false;
    for (op in program.ops) switch op {
      case ToolChange(_, _): loaded = true;
      case Move(_, _, _, _, _): if (loaded) motions++;
      case _:
    }
    var moves = CutMoves.fromOps(program.ops, program.tool);
    Assert.check(moves.length == motions, "every CAM motion with a tool loaded becomes a cut move");
    Assert.check(moves[0].tool == small && moves[moves.length - 1].tool == large,
      "moves follow the CAM tool changes");
    var rejected = false;
    try new CamJob(0.005, 12000).pocket(contour, small, -0.002, 0.01, 0.0015, 0.001)
      .profile(contour, new Tool(2, 0.0, 0.004), -0.003, 0.01)
    catch (_:Dynamic) rejected = true;
    Assert.check(rejected, "a CAM job cannot reuse a tool number for another tool");
  }

  static function end(move:CutMove):Point3
    return switch move.motion {
      case Path(geometry):
        toolpathkit.path.GeometryTools.pointAt(geometry,
          toolpathkit.path.GeometryTools.length(geometry));
    };
}
