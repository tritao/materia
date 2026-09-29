import camkit.CamContour;

import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.path.Provenance;
import toolpathkit.tool.ToolLibrary;
import camkit.CamJob;
import toolpathkit.tool.Tool;
import toolpathkit.path.Point3;
import toolpathkit.setup.Setup;
import toolpathkit.tool.CutterProfile;
import stockkit.CutMoves;

class CutMoveTests {
  public static function run():Void {
    tools();
    sharedProgram();
    camProgram();
  }

  static function sharedProgram():Void {
    var tool = new Tool(8, 0, 0.004);
    var library = new ToolLibrary();
    library.set(tool);
    var source = Provenance.cam(7, "face:3");
    var start = new Point3(0.01, 0.01, 0.002);
    var end = new Point3(0.02, 0.01, -0.001);
    var program = new ToolpathProgram([
      Move(Rapid, Line(start, end), 0, 0, source),
      ToolChange(8, source),
      ToolLengthOffset(8, 0.003, source),
      Move(Ramp, Line(start, end), 0.001, 0, source),
      Move(Retract, Line(end, start), 0, 0, source),
      MachineMove(Rapid, Line(start, end), 0, 0, source)
    ], library, [new Setup("1", new Point3(0, 0, 0))]);
    var moves = CutMoves.fromProgram(program);
    Assert.check(moves.length == 2 && moves[0].kind == Ramp &&
      moves[1].kind == Retract && moves[1].rapid,
      "shared move kinds reach stock simulation");
    Assert.check(moves[0].operationId == "cam:7" &&
      moves[0].featureRef == "face:3" && moves[0].toolId == 8 &&
      moves[0].provenance == source,
      "stock move retains operation, feature, tool and provenance");
    switch moves[0].motion {
      case Path(Line(a, b)):
        Assert.near(a.z, -0.001, "tool length shifts the start to its tip", 1e-12);
        Assert.near(b.z, -0.004, "tool length shifts the end to its tip", 1e-12);
      case _: Assert.check(false, "ramp remains a line");
    }
    var placed = new ToolpathProgram([
      ToolChange(8, source),
      Move(Cut, Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
        0.01, 0, source),
      SetSetup("2", source),
      Move(Cut, Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
        0.01, 0, source)
    ], library, [new Setup("1", new Point3(0, 0, 0)),
      new Setup("2", new Point3(0.1, 0, 0))]);
    var placedMoves = CutMoves.fromProgram(placed);
    switch [placedMoves[0].motion, placedMoves[1].motion] {
      case [Path(Line(_, first)), Path(Line(_, second))]:
        Assert.near(second.x - first.x, 0.1,
          "stock moves follow the program's active setup", 1e-12);
      case _: Assert.check(false, "placed stock cuts remain lines");
    }
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

    var library = new toolpathkit.tool.ToolLibrary();
    library.set(tool);
    Assert.check(library.tool(4).cutter == ball, "tool library keeps the shape");
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
    Assert.check(program.tools.tool(3) == large, "CAM programs keep the tools they use");
    // CAM retracts before each tool change; the first retract has no tool loaded.
    var motions = 0, loaded = false;
    for (op in program.ops) switch op {
      case ToolChange(_, _): loaded = true;
      case Move(_, _, _, _, _): if (loaded) motions++;
      case _:
    }
    var moves = CutMoves.fromProgram(program);
    Assert.check(moves.length == motions, "every CAM motion with a tool loaded becomes a cut move");
    Assert.check(moves[0].tool == small && moves[moves.length - 1].tool == large,
      "moves follow the CAM tool changes");
    var rejected = false;
    try new CamJob(0.005, 12000).pocket(contour, small, -0.002, 0.01, 0.0015, 0.001)
      .profile(contour, new Tool(2, 0.0, 0.004), -0.003, 0.01)
    catch (_:Dynamic) rejected = true;
    Assert.check(rejected, "a CAM job cannot reuse a tool number for another tool");
  }

}
