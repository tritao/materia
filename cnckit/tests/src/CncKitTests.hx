import cnckit.CncCompiler;
import cnckit.CncMachine;
import cnckit.CncDialect;
import cnckit.CncDiagnostic.CncSeverity;
import cnckit.ir.CncGeometryTools;
import cnckit.ir.CncOp;
import motionkit.path.ArcSegment;
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
    var ordered = new CncCompiler(machine).compile("T1 M6 M3 G0 Z5\nM2");
    check(switch ordered.ops[0] {
      case MotionOp.WaitInput("cnc.tool_change.1", _, null): true;
      case _: false;
    }, "M6 precedes spindle and has no timeout");
    check(switch ordered.ops[1] {
      case MotionOp.SetOutput("spindle.direction", _): true;
      case _: false;
    }, "M3 follows tool change");
    check(switch ordered.ops[3] {
      case MotionOp.FollowPath(_, _, _, _): true;
      case _: false;
    }, "motion follows spindle start");
    var stopped = new CncCompiler(machine).compile("M3 M8\nM5 M9 G0 X1\nM2");
    check(switch stopped.ops[3] {
      case MotionOp.SetOutput("spindle.speed", _): true;
      case _: false;
    }, "M5 stops spindle before coolant and motion");
    check(switch stopped.ops[5] {
      case MotionOp.SetOutput("coolant.mist", _): true;
      case _: false;
    }, "M9 follows M5 before motion");
    check(switch stopped.ops[7] {
      case MotionOp.FollowPath(_, _, _, _): true;
      case _: false;
    }, "M5 and M9 precede motion");
    var partialCompiler = new CncCompiler(machine);
    var partial = partialCompiler.compile(
      "G21 G90 G64 P1 F600 G1 X10\nG1 Y10\nG1 Y0\nG1 X20\nM2");
    check(partialCompiler.warnings.length > 0, "unblendable corner warns");
    check(partialCompiler.warnings[0].indexOf("G-code line 3") >= 0,
      "blend warning names the corner line");
    check(partial.ops.length == 1, "fallback keeps the combined path");
    var strictMachine = new CncMachine("work", "x", "y", "z", 0.2,
      null, 0.0005, 0.02, CncDialect.LinuxCnc, 0.5);
    var strictCompiler = new CncCompiler(strictMachine);
    strictCompiler.compile("G21 G64 P1 F600 G1 X10\nG1 Y10\nM2");
    check(strictCompiler.warnings.length == 1,
      "machine blend corner limit controls exact stops");
    var circle = new CncCompiler(machine).compile(
      "G21 G90 F600 G0 X10 Y0\nG2 X10 Y0 I-10 J0\nM2");
    var circlePath = switch circle.ops[1] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "full circle missing";
    };
    near(circlePath.length(), 2 * Math.PI * 0.01,
      "G2 full circle circumference", 1e-7);
    var arcMachine = new CncMachine("work", "x", "y", "z", 0.2);
    var cw = new CncCompiler(arcMachine).compile(
      "G21 G91 F600 G2 X10 I5 J0\nM2");
    var ccw = new CncCompiler(arcMachine).compile(
      "G21 G91 F600 G3 X10 I5 J0\nM2");
    var cwPath = switch cw.ops[0] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "CW arc missing";
    };
    var ccwPath = switch ccw.ops[0] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "CCW arc missing";
    };
    var cwPrimitive:cnckit.CncPosePrimitive = cast cwPath.primitives[0];
    var ccwPrimitive:cnckit.CncPosePrimitive = cast ccwPath.primitives[0];
    var cwArc:ArcSegment = cast cwPrimitive.geometry;
    var ccwArc:ArcSegment = cast ccwPrimitive.geometry;
    check(cwArc.sweepAngle < 0.0 && ccwArc.sweepAngle > 0.0,
      "CW and CCW sweeps have opposite signs");
    near(cwArc.center.x, 0.005, "G91 I centre is relative to start");
    near(ccwArc.center.x, 0.005, "G91 CCW centre is relative to start");
    rejects(machine, "G21 G90 F600 G0 X10\nG2 X0 Y10 I-10 J0",
      "arc endpoint is not on its I/J circle");
    var relativeTool = new CncCompiler(machine).compile(
      "G21 G91 G43 H2 G0 Z10\nG49 G0 Z10\nM2");
    var finalRelative = switch relativeTool.ops[relativeTool.ops.length - 1] {
      case MotionOp.FollowPath(path, _, _, _): path.poseAt(path.length());
      case _: throw "relative G49 motion missing";
    };
    near(finalRelative.z, 0.02, "G43/G49 do not shift relative moves");
    var lowercase = new CncCompiler(machine).compile(
      "g21 g90 (comment) f600 g1 x1 ; tail\nm30");
    check(lowercase.ops.length == 1, "lowercase and comments parse");
    rejects(machine, "M30\nG0 X1", "code after M2/M30");
    var detailed = new CncCompiler(arcMachine).compileDetailed(
      "G21 G90 G64 P1 F600 G1 X10\nG1 Y10\nG1 Xoops\nG2 X0 Y0 I0 J0\n" +
      "G1 X20\nM2\nG1 X30");
    check(detailed.diagnostics.length == 3,
      "lexer, interpreter, and program-end errors are all reported");
    check(detailed.diagnostics[0].span.line == 3 &&
      detailed.diagnostics[1].span.line == 4 &&
      detailed.diagnostics[2].span.line == 7,
      "diagnostics retain source order after line recovery");
    check(detailed.diagnostics[0].severity == CncSeverity.Error &&
      detailed.diagnostics[0].code == "CNC_LEX",
      "diagnostics carry severity and stable code");
    check(detailed.program != null && detailed.ops.length >= 4,
      "valid lines still produce preview and executable program");
    var feedGeometry = switch detailed.ops[0] {
      case CncOp.Feed(geometry, _, _, span):
        check(span.line == 1, "IR feed keeps its source span");
        geometry;
      case _: throw "first preview operation must be feed";
    };
    near(CncGeometryTools.length(feedGeometry), 0.01,
      "IR preview geometry uses metres");
    var previewEnd = CncGeometryTools.pointAt(feedGeometry,
      CncGeometryTools.length(feedGeometry));
    near(previewEnd.x, 0.01, "preview endpoint needs no MotionKit lowering");
    var recoveredGeometry = switch detailed.ops[2] {
      case CncOp.Feed(geometry, _, _, _): geometry;
      case _: throw "recovered move missing";
    };
    near(CncGeometryTools.pointAt(recoveredGeometry, 0.0).y, 0.01,
      "failed arc leaves the modal position at the previous valid line");
    var firstMap = detailed.sourceMap.spanAt(0, 0.002);
    var secondMap = detailed.sourceMap.spanAt(0, 0.014);
    check(firstMap != null && firstMap.line == 1 &&
      secondMap != null && secondMap.line == 2,
      "distance along a blended MotionOp maps to the authored lines");
    check(detailed.sourceMap.spanAt(0, 1.0) == null,
      "source map rejects distance outside the path");
    var empty = new CncCompiler(arcMachine).compileDetailed("(nothing)\n");
    check(empty.program == null && empty.diagnostics.length == 1 &&
      empty.diagnostics[0].code == "CNC_EMPTY",
      "empty source has a structured diagnostic");
    var rollback = new CncCompiler(arcMachine).compileDetailed(
      "G21 G90 F600 G1 X10\nF1 G2 X20 I0 J0\nG1 X20");
    check(rollback.diagnostics.length == 1 && rollback.ops.length == 2,
      "failed block is skipped without discarding later motion");
    var recoveredFeed = switch rollback.ops[1] {
      case CncOp.Feed(_, speed, _, _): speed;
      case _: throw "feed after error missing";
    };
    near(recoveredFeed, 0.01,
      "failed block does not commit its feed change");
    Sys.println('CncKit tests passed ($assertions assertions)');
  }
}
