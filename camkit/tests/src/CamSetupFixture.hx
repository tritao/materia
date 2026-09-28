import camkit.CamContour;
import camkit.CamFixture;
import camkit.CamGCodeWriter;
import camkit.CamJob;
import camkit.CamProgram;
import cnckit.ir.CncGeometry;
import cnckit.ir.CncOp;
import cnckit.parse.CncSpan;
import camkit.CamSetup;
import cnckit.CncMachine;
import cnckit.CncTool;
import cnckit.ir.CncPoint;

/** Generated plate and clamp keep-out, independent of external example files. */
class CamSetupFixture {
  public static function run(check:Bool->String->Void):Void {
    var contour = new CamContour([
      new CncPoint(0.0, 0.0, 0.0),
      new CncPoint(0.04, 0.0, 0.0),
      new CncPoint(0.04, 0.03, 0.0),
      new CncPoint(0.0, 0.03, 0.0)
    ]);
    var tool = new CncTool(7, 0.0, 0.002);
    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    machine.setTool(tool);
    var program = new CamJob(0.008, 12000)
      .profile(contour, tool, -0.002, 0.01, "on").finish();
    var clear = new CamSetup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.003, 0.008);
    check(CamGCodeWriter.write(program, clear, machine).indexOf("G1") >= 0,
      "generated plate exports with a clear setup");
    var clamp = new CamFixture("edge-clamp", 0.018, 0.022,
      0.0005, 0.005, -0.003, 0.006);
    var occupied = new CamSetup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.003, 0.008, [clamp]);
    var collision = "";
    try CamGCodeWriter.write(program, occupied, machine)
    catch (error:Dynamic) collision = Std.string(error);
    check(collision.indexOf("line 1") >= 0 &&
      collision.indexOf("edge-clamp") >= 0,
      "tool radius catches the edge clamp and names the CAM operation");
    var tooShallow = new CamSetup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.001, 0.008);
    var depthError = "";
    try CamGCodeWriter.write(program, tooShallow, machine)
    catch (error:Dynamic) depthError = Std.string(error);
    check(depthError.indexOf("below stock bottom") >= 0,
      "setup rejects a cut below planned stock depth");
    var highClearance = new CamSetup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.003, 0.009);
    var rapidError = "";
    var traverse = new CamProgram([
      CncOp.Rapid(CncGeometry.Line(new CncPoint(0.0, 0.0, 0.008),
        new CncPoint(0.01, 0.0, 0.008)), new CncSpan(2, 1, 1)),
      CncOp.End(new CncSpan(3, 1, 1))
    ]);
    try CamGCodeWriter.write(traverse, highClearance, machine)
    catch (error:Dynamic) rapidError = Std.string(error);
    check(rapidError.indexOf("below safe Z") >= 0,
      "setup rejects lateral rapids below its safe Z: " + rapidError);
    var blockedSafeZ = false;
    try new CamSetup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.003, 0.008,
      [new CamFixture("tall-clamp", 0.01, 0.02, 0.01, 0.02,
        0.0, 0.009)])
    catch (_:Dynamic) blockedSafeZ = true;
    check(blockedSafeZ, "setup rejects a safe Z below a fixture top");
    var arcProgram = new CamProgram([
      CncOp.ToolChange(7, new CncSpan(4, 1, 1)),
      CncOp.Feed(CncGeometry.Arc(new CncPoint(0.02, 0.02, -0.001),
        0.01, 0.0, Math.PI), 0.01, 0.0, new CncSpan(5, 1, 1)),
      CncOp.End(new CncSpan(6, 1, 1))
    ]);
    var arcClamp = new CamSetup(-0.01, 0.05, -0.01, 0.04,
      0.0, -0.003, 0.008,
      [new CamFixture("arc-clamp", 0.019, 0.021,
        0.029, 0.031, -0.003, 0.006)]);
    var arcError = "";
    try CamGCodeWriter.write(arcProgram, arcClamp, machine)
    catch (error:Dynamic) arcError = Std.string(error);
    check(arcError.indexOf("line 5") >= 0 &&
      arcError.indexOf("arc-clamp") >= 0,
      "setup checks the curved cutter path between arc endpoints");
  }
}
