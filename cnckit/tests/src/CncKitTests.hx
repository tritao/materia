import cnckit.CncCompiler;
import cnckit.CncMachine;
import motionkit.program.MotionOp;

class CncKitTests {
  static var assertions = 0;
  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function near(actual:Float, expected:Float, message:String,
      ?tolerance:Float = 1e-9):Void
    check(Math.abs(actual - expected) <= tolerance,
      '$message: expected $expected, got $actual');
  static function rejects(machine:CncMachine, source:String, expected:String):Void {
    var error = "";
    try new CncCompiler(machine).compile(source)
    catch (caught:Dynamic) error = Std.string(caught);
    check(error.indexOf(expected) >= 0, 'expected "$expected" in "$error"');
  }

  public static function main():Void {
    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    machine.setWorkOffset(54, 0.1, 0.2, 0.0);
    machine.setToolLength(2, 0.012);
    var source = "G21 G90 G54 G17\nS12000 M3\nG0 X0 Y0 Z10\nF600 G1 Z0\n" +
      "G1 X20\nG1 Y20\nG2 X0 Y20 I-10 J0\nG1 Y0\nM5 M9\nM2\n";
    var program = new CncCompiler(machine).compile(source);
    check(program.ops.length >= 9, "pocket emits motion and spindle operations");
    check(switch program.ops[0] {
      case MotionOp.SetOutput("spindle.direction", _): true;
      case _: false;
    }, "spindle starts before first motion");
    var firstPath = switch program.ops[2] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "rapid must be a FollowPath";
    };
    near(firstPath.poseAt(firstPath.length()).x, 0.1,
      "G54 applies X offset");
    near(firstPath.poseAt(firstPath.length()).z, 0.01,
      "G0 Z mm converts to metres");
    var arcs = 0;
    for (op in program.ops) switch op {
      case MotionOp.FollowPath(path, _, _, _):
        if (path.primitives[0].length() > 0.025) arcs++;
      case _:
    }
    check(arcs >= 1, "pocket includes an arc path");

    var incremental = new CncCompiler(machine).compile(
      "G20 G91\nG0 X1\nF60 G1 Y1\nG3 X-1 Y-1 I-1 J0\nM2");
    var rapid = switch incremental.ops[0] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "incremental rapid is not a path";
    };
    near(rapid.poseAt(rapid.length()).x, 0.0254,
      "inch incremental rapid converts to metres");
    var feed = switch incremental.ops[1] {
      case MotionOp.FollowPath(_, _, speed, _): speed;
      case _: throw "incremental feed is not a path";
    };
    near(feed, 0.0254, "inch feed converts from units/minute to m/s");
    var modal = new CncCompiler(machine).compile(
      "G21 G90 G55\nF600 G1 X10\nY10\nG91 X5\nM2");
    var modalEnd = switch modal.ops[modal.ops.length - 1] {
      case MotionOp.FollowPath(path, _, _, _): path.poseAt(path.length());
      case _: throw "modal program must end in feed motion";
    };
    near(modalEnd.x, 0.015, "G1 and G91 remain modal across lines");
    near(modalEnd.y, 0.01, "Y coordinate uses persisted G1 mode");
    var blend = new CncCompiler(machine).compile(
      "G21 G90 G64 P1 F600\nG1 X10\nG1 Y10\nG61\nG1 X20\nM2");
    var blended = switch blend.ops[0] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "G64 must emit a path";
    };
    check(blended.authoredGeometry != null,
      "G64 retains authored geometry for task-space validation");
    near(blended.blendTolerance, 0.001, "G64 P converts to metres");
    check(blend.ops.length == 2, "G61 splits the blended path at an exact stop");
    var inchBlend = new CncCompiler(machine).compile(
      "G64 P0.1 G20 F60 G1 X1\nG1 Y1\nM2");
    var inchPath = switch inchBlend.ops[0] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "inch G64 must emit a path";
    };
    near(inchPath.blendTolerance, 0.00254,
      "G64 P uses the block unit mode regardless of word order");

    var tool = new CncCompiler(machine).compile(
      "G21 G90 G43 H2\nG0 Z10\nT3 M6\nM0\nM2");
    var toolPath = switch tool.ops[0] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "tool-offset rapid is not a path";
    };
    near(toolPath.poseAt(toolPath.length()).z, 0.022,
      "G43 H applies stored tool length");
    check(switch tool.ops[1] {
      case MotionOp.WaitInput("cnc.tool_change.3", _, _): true;
      case _: false;
    }, "M6 is a tool-change barrier");
    rejects(machine, "G18\nG1 X1 F100", "line 1 column 1");
    rejects(machine, "G2 X1 R2", "line 1 column 7");
    rejects(machine, "G41", "line 1 column 1");
    rejects(machine, "G64 P1 G61\nG1 X1 F100", "multiple path-control");
    rejects(machine, "G43 H9", "line 1 column 5");
    Sys.println('CncKit tests passed ($assertions assertions)');
  }
}
