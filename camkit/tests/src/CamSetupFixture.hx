import camkit.CamContour;

import toolpathkit.path.MoveKind;import toolpathkit.setup.Fixture;
import cnckit.CncWriter;
import camkit.CamJob;
import camkit.CamProgram;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Provenance;
import toolpathkit.setup.Setup;
import cnckit.CncMachine;
import toolpathkit.tool.Tool;
import toolpathkit.path.Point3;

/** Generated plate and clamp keep-out, independent of external example files. */
class CamSetupFixture {
  public static function run(check:Bool->String->Void):Void {
    var contour = new CamContour([
      new Point3(0.0, 0.0, 0.0),
      new Point3(0.04, 0.0, 0.0),
      new Point3(0.04, 0.03, 0.0),
      new Point3(0.0, 0.03, 0.0)
    ]);
    var tool = new Tool(7, 0.0, 0.002);
    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    machine.toolLibrary.set(tool);
    var program = new CamJob(0.008, 12000)
      .profile(contour, tool, -0.002, 0.01, "on").finish();
    var clear = new Setup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.003, 0.008);
    var withSetup = new CamJob(0.008, 12000)
      .profile(contour, tool, -0.002, 0.01, "on").finish(clear);
    check(switch withSetup.ops[0] {
      case ToolpathOp.SetSetup("1", _): true;
      case _: false;
    }, "CAM job emits its setup switch");
    check(CncWriter.write(program.ops, clear, machine).indexOf("G1") >= 0,
      "generated plate exports with a clear setup");
    var clamp = new Fixture("edge-clamp", 0.018, 0.022,
      0.0005, 0.005, -0.003, 0.006);
    var occupied = new Setup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.003, 0.008, [clamp]);
    var collision = "";
    try CncWriter.write(program.ops, occupied, machine)
    catch (error:Dynamic) collision = Std.string(error);
    check(collision.indexOf("line 1") >= 0 &&
      collision.indexOf("edge-clamp") >= 0,
      "tool radius catches the edge clamp and names the CAM operation");
    var tooShallow = new Setup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.001, 0.008);
    var depthError = "";
    try CncWriter.write(program.ops, tooShallow, machine)
    catch (error:Dynamic) depthError = Std.string(error);
    check(depthError.indexOf("below stock bottom") >= 0,
      "setup rejects a cut below planned stock depth");
    var highClearance = new Setup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.003, 0.009);
    var rapidError = "";
    var traverse = new CamProgram([
      ToolpathOp.Move(MoveKind.Rapid, PathGeometry.Line(new Point3(0.0, 0.0, 0.008),
        new Point3(0.01, 0.0, 0.008)), 0.0, 0.0, new Provenance(2, 1, 1)),
      ToolpathOp.End(new Provenance(3, 1, 1))
    ]);
    try CncWriter.write(traverse.ops, highClearance, machine)
    catch (error:Dynamic) rapidError = Std.string(error);
    check(rapidError.indexOf("below safe Z") >= 0,
      "setup rejects lateral rapids below its safe Z: " + rapidError);
    var blockedSafeZ = false;
    try new Setup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.003, 0.008,
      [new Fixture("tall-clamp", 0.01, 0.02, 0.01, 0.02,
        0.0, 0.009)])
    catch (_:Dynamic) blockedSafeZ = true;
    check(blockedSafeZ, "setup rejects a safe Z below a fixture top");
    var arcProgram = new CamProgram([
      ToolpathOp.ToolChange(7, new Provenance(4, 1, 1)),
      ToolpathOp.Move(MoveKind.Cut, PathGeometry.Arc(new Point3(0.02, 0.02, -0.001),
        0.01, 0.0, Math.PI), 0.01, 0.0, new Provenance(5, 1, 1)),
      ToolpathOp.End(new Provenance(6, 1, 1))
    ]);
    var arcClamp = new Setup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.003, 0.008,
      [new Fixture("arc-clamp", 0.019, 0.021,
        0.029, 0.031, -0.003, 0.006)]);
    var arcError = "";
    try CncWriter.write(arcProgram.ops, arcClamp, machine)
    catch (error:Dynamic) arcError = Std.string(error);
    check(arcError.indexOf("line 5") >= 0 &&
      arcError.indexOf("arc-clamp") >= 0,
      "setup checks the curved cutter path between arc endpoints");
  }
}
