import cnckit.CncWriter;
import CncTestCompiler.CncTestCompileResult;
import toolpathkit.tool.Tool;
import cnckit.CncDialect;
import cnckit.CncDiagnostic.CncSeverity;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ArcPlane;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.setup.Setup;
import toolpathkit.setup.SetupStock;
import toolpathkit.path.ToolpathProgram;
import motionkit.path.ArcSegment;
import motionkit.path.CircularSegment;
import motionkit.kinematics.Pose3;
import motionkit.path.PosePrimitive;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import sys.io.File;

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
  /** Every lowered path primitive, in program order: moves between barriers share one path. */
  static function primitives(program:MotionProgram):Array<PosePrimitive> {
    var result:Array<PosePrimitive> = [];
    for (op in program.ops) switch op {
      case MotionOp.FollowPath(path, _, _, _): for (primitive in path.primitives) result.push(primitive);
      case _:
    }
    return result;
  }
  static function endOf(primitive:PosePrimitive):Pose3
    return primitive.waypointAt(primitive.length()).pose;
  static function rejects(machine:CncTestRig, source:String, expected:String):Void {
    var error = "";
    try new CncTestCompiler(machine).compile(source)
    catch (caught:Dynamic) error = Std.string(caught);
    check(error.indexOf(expected) >= 0, 'expected "$expected" in "$error"');
  }
  static function hasError(result:CncTestCompileResult, line:Int,
      fragment:String):Bool {
    for (diagnostic in result.diagnostics)
      if (diagnostic.severity == Error && diagnostic.span.line == line &&
          diagnostic.message.indexOf(fragment) >= 0) return true;
    return false;
  }

  public static function main():Void {
    var machine = new CncTestRig("work", "x", "y", "z", 0.2);
    machine.controller.setWorkOffset(54, 0.1, 0.2, 0.0);
    machine.controller.setToolLength(2, 0.012);
    var source = "G21 G90 G54 G17\nS12000 M3\nG0 X0 Y0 Z10\nF600 G1 Z0\n" +
      "G1 X20\nG1 Y20\nG2 X0 Y20 I-10 J0\nG1 Y0\nM5 M9\nM2\n";
    var program = new CncTestCompiler(machine).compile(source);
    check(program.ops.length == 4 && primitives(program).length == 6,
      "pocket emits the spindle start and one path of its six moves");
    check(switch program.ops[0] {
      case MotionOp.SetOutput("spindle.direction", _): true;
      case _: false;
    }, "spindle starts before first motion");
    var rapidEnd = endOf(primitives(program)[0]);
    near(rapidEnd.x, 0.1, "G54 applies X offset");
    near(rapidEnd.z, 0.01, "G0 Z mm converts to metres");
    var switchingMachine = new CncTestRig("work", "x", "y", "z", 0.2);
    switchingMachine.controller.setWorkOffset(54, 0.1, 0.0, 0.0);
    switchingMachine.controller.setWorkOffset(55, 0.2, 0.0, 0.0);
    var switched = switchingMachine.compileDetailed(
      "G21 G90 G54 G0 X10\nG55 G0 X10\nM2");
    check(switched.diagnostics.length == 0, "G54/G55 compile without errors");
    near(switched.program.setups[0].workOrigin.x, 0.1,
      "G54 setup position comes from the controller");
    near(switched.program.setups[1].workOrigin.x, 0.2,
      "G55 setup position comes from the controller");
    check(switch switched.program.ops[0] {
      case ToolpathOp.SetSetup("1", _): true;
      case _: false;
    }, "G54 selects setup one");
    check(switch switched.program.ops[2] {
      case ToolpathOp.SetSetup("2", _): true;
      case _: false;
    }, "G55 selects setup two");
    var switchedWork = switch switched.program.ops[3] {
      case ToolpathOp.Move(Rapid, PathGeometry.Line(_, end), _, _, _): end;
      case _: throw "G55 work move missing";
    };
    near(switchedWork.x, 0.01, "G55 move stays in work coordinates");
    var switchedMotion = new CncTestCompiler(switchingMachine).compile(
      "G21 G90 G54 G0 X10\nG55 G0 X10\nM2");
    var switchedMachineX = 0.0;
    for (op in switchedMotion.ops) switch op {
      case MotionOp.FollowPath(path, _, _, _):
        switchedMachineX = path.poseAt(path.length()).x;
      case _:
    }
    near(switchedMachineX, 0.21, "G55 adapter applies its setup offset");
    var arcs = 0;
    for (primitive in primitives(program)) if (primitive.length() > 0.025) arcs++;
    check(arcs >= 1, "pocket includes an arc path");

    var incremental = new CncTestCompiler(machine).compile(
      "G20 G91\nG0 X1\nF60 G1 Y1\nG3 X-1 Y-1 I-1 J0\nM2");
    var incrementalMoves = primitives(incremental);
    near(endOf(incrementalMoves[0]).x, 0.0254, "inch incremental rapid converts to metres");
    near(incrementalMoves[1].speedLimit(), 0.0254, "inch feed converts from units/minute to m/s");
    var modal = new CncTestCompiler(machine).compile(
      "G21 G90 G55\nF600 G1 X10\nY10\nG91 X5\nM2");
    var modalEnd = switch modal.ops[modal.ops.length - 1] {
      case MotionOp.FollowPath(path, _, _, _): path.poseAt(path.length());
      case _: throw "modal program must end in feed motion";
    };
    near(modalEnd.x, 0.015, "G1 and G91 remain modal across lines");
    near(modalEnd.y, 0.01, "Y coordinate uses persisted G1 mode");
    var blend = new CncTestCompiler(machine).compile(
      "G21 G90 G64 P1 F600\nG1 X10\nG1 Y10\nG61\nG1 X20\nM2");
    var blended = switch blend.ops[0] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "G64 must emit a path";
    };
    check(blended.authoredGeometry != null,
      "G64 retains authored geometry for task-space validation");
    near(blended.blendTolerance, 0.001, "G64 P converts to metres");
    var exactLine = blended.primitives[blended.primitives.length - 1];
    check(blend.ops.length == 1 && Math.abs(exactLine.length() - 0.01) < 1e-9,
      "G61 leaves its corner sharp, where the plan stops");
    var inchBlend = new CncTestCompiler(machine).compile(
      "G64 P0.1 G20 F60 G1 X1\nG1 Y1\nM2");
    var inchPath = switch inchBlend.ops[0] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "inch G64 must emit a path";
    };
    near(inchPath.blendTolerance, 0.00254,
      "G64 P uses the block unit mode regardless of word order");

    var tool = new CncTestCompiler(machine).compile(
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
    rejects(machine, "G2 X1 R2", "line 1 column 7");
    rejects(machine, "G41", "line 1 column 1");
    rejects(machine, "G64 P1 G61\nG1 X1 F100", "multiple path-control");
    rejects(machine, "G43 H9", "line 1 column 5");
    var ordered = new CncTestCompiler(machine).compile("T1 M6 M3 G0 Z5\nM2");
    check(switch ordered.ops[0] {
      case MotionOp.WaitInput("cnc.tool_change.1", _, null): true;
      case _: false;
    }, "M6 precedes spindle and has no timeout");
    check(switch ordered.ops[1] {
      case MotionOp.SetOutput("spindle.direction", _): true;
      case _: false;
    }, "M3 follows tool change");
    check(switch ordered.ops[3] {
      case MotionOp.WaitInput("spindle.at_speed", _, _): true;
      case _: false;
    }, "spindle start waits for speed feedback");
    check(switch ordered.ops[4] {
      case MotionOp.FollowPath(_, _, _, _): true;
      case _: false;
    }, "motion follows spindle start");
    var stopped = new CncTestCompiler(machine).compile("M3 M8\nM5 M9 G0 X1\nM2");
    check(switch stopped.ops[3] {
      case MotionOp.FollowPath(_, _, _, events):
        Lambda.exists(events, function(event) return event.channel == "spindle.speed" &&
          event.distance == 0.0);
      case _: false;
    }, "M5 spindle stop is tied to path start");
    check(switch stopped.ops[3] {
      case MotionOp.FollowPath(_, _, _, events):
        Lambda.exists(events, function(event) return event.channel == "coolant.mist" &&
          event.distance == 0.0);
      case _: false;
    }, "M9 coolant change is tied to path start");
    check(stopped.ops.length == 4 && switch stopped.ops[3] {
      case MotionOp.FollowPath(_, _, _, _): true;
      case _: false;
    }, "spindle and coolant changes add no stop before motion");
    var partialCompiler = new CncTestCompiler(machine);
    var partial = partialCompiler.compile(
      "G21 G90 G64 P1 F600 G1 X10\nG1 Y10\nG1 Y0\nG1 X20\nM2");
    check(partialCompiler.warnings.length > 0, "unblendable corner warns");
    check(partialCompiler.warnings[0].indexOf("G-code line 3") >= 0,
      "blend warning names the corner line");
    check(partial.ops.length == 1, "fallback keeps the combined path");
    var strictMachine = new CncTestRig("work", "x", "y", "z", 0.2,
      null, 0.0005, 0.02, CncDialect.LinuxCnc, 0.5);
    var strictCompiler = new CncTestCompiler(strictMachine);
    strictCompiler.compile("G21 G64 P1 F600 G1 X10\nG1 Y10\nM2");
    check(strictCompiler.warnings.length == 1,
      "machine blend corner limit controls exact stops");
    var circle = new CncTestCompiler(machine).compile(
      "G21 G90 F600 G0 X10 Y0\nG2 X10 Y0 I-10 J0\nM2");
    var circleMoves = primitives(circle);
    check(circleMoves.length == 2, "full circle missing");
    near(circleMoves[1].length(), 2 * Math.PI * 0.01,
      "G2 full circle circumference", 1e-7);
    var arcMachine = new CncTestRig("work", "x", "y", "z", 0.2);
    var cw = new CncTestCompiler(arcMachine).compile(
      "G21 G91 F600 G2 X10 I5 J0\nM2");
    var ccw = new CncTestCompiler(arcMachine).compile(
      "G21 G91 F600 G3 X10 I5 J0\nM2");
    var cwPath = switch cw.ops[0] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "CW arc missing";
    };
    var ccwPath = switch ccw.ops[0] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "CCW arc missing";
    };
    var cwPrimitive:toolpathkit.motion.ToolpathPosePrimitive = cast cwPath.primitives[0];
    var ccwPrimitive:toolpathkit.motion.ToolpathPosePrimitive = cast ccwPath.primitives[0];
    var cwArc:ArcSegment = cast cwPrimitive.geometry;
    var ccwArc:ArcSegment = cast ccwPrimitive.geometry;
    check(cwArc.sweepAngle < 0.0 && ccwArc.sweepAngle > 0.0,
      "CW and CCW sweeps have opposite signs");
    near(cwArc.center.x, 0.005, "G91 I centre is relative to start");
    near(ccwArc.center.x, 0.005, "G91 CCW centre is relative to start");
    rejects(machine, "G21 G90 F600 G0 X10\nG2 X0 Y10 I-10 J0",
      "arc endpoint is not on its centre circle");
    var relativeTool = new CncTestCompiler(machine).compile(
      "G21 G91 G43 H2 G0 Z10\nG49 G0 Z10\nM2");
    var finalRelative = switch relativeTool.ops[relativeTool.ops.length - 1] {
      case MotionOp.FollowPath(path, _, _, _): path.poseAt(path.length());
      case _: throw "relative G49 motion missing";
    };
    near(finalRelative.z, 0.02, "G43/G49 do not shift relative moves");
    var lengthOps = new CncTestCompiler(machine).compileDetailed(
      "G21 G90 G43 H2\nG0 Z10\nG49\nM2").ops;
    var offsets = [for (op in lengthOps) switch op {
      case ToolLengthOffset(number, length, span): '$number:$length:${span.line}';
      case _: null;
    }].filter(entry -> entry != null);
    check(offsets.length == 2 && offsets[0] == '2:${machine.controller.toolLength(2)}:1'
      && offsets[1] == "0:0:3", "G43 and G49 record the tool length they apply");
    var lowercase = new CncTestCompiler(machine).compile(
      "g21 g90 (comment) f600 g1 x1 ; tail\nm30");
    check(lowercase.ops.length == 1, "lowercase and comments parse");
    rejects(machine, "M30\nG0 X1", "code after M2/M30");
    var detailed = new CncTestCompiler(arcMachine).compileDetailed(
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
      case ToolpathOp.Move(Cut, geometry, _, _, span):
        check(span.line == 1, "IR feed keeps its source span");
        geometry;
      case _: throw "first preview operation must be feed";
    };
    near(GeometryTools.length(feedGeometry), 0.01,
      "IR preview geometry uses metres");
    var previewEnd = GeometryTools.pointAt(feedGeometry,
      GeometryTools.length(feedGeometry));
    near(previewEnd.x, 0.01, "preview endpoint needs no MotionKit lowering");
    var recoveredGeometry = switch detailed.ops[2] {
      case ToolpathOp.Move(Cut, geometry, _, _, _): geometry;
      case _: throw "recovered move missing";
    };
    near(GeometryTools.pointAt(recoveredGeometry, 0.0).y, 0.01,
      "failed arc leaves the modal position at the previous valid line");
    var firstMap = detailed.sourceMap.provenanceAt(0, 0.002);
    var secondMap = detailed.sourceMap.provenanceAt(0, 0.014);
    check(firstMap != null && firstMap.line == 1 &&
      secondMap != null && secondMap.line == 2,
      "distance along a blended MotionOp maps to the authored lines");
    check(detailed.sourceMap.provenanceAt(0, 1.0) == null,
      "source map rejects distance outside the path");
    var empty = new CncTestCompiler(arcMachine).compileDetailed("(nothing)\n");
    check(empty.program == null && empty.diagnostics.length == 1 &&
      empty.diagnostics[0].code == "CNC_EMPTY",
      "empty source has a structured diagnostic");
    var rollback = new CncTestCompiler(arcMachine).compileDetailed(
      "G21 G90 F600 G1 X10\nF1 G2 X20 I0 J0\nG1 X20");
    check(rollback.diagnostics.length == 1 && rollback.ops.length == 2,
      "failed block is skipped without discarding later motion");
    var recoveredFeed = switch rollback.ops[1] {
      case ToolpathOp.Move(Cut, _, speed, _, _): speed;
      case _: throw "feed after error missing";
    };
    near(recoveredFeed, 0.01,
      "failed block does not commit its feed change");
    var fixtureMachine = new CncTestRig("work", "x", "y", "z", 0.2);
    fixtureMachine.controller.setToolLength(1, 0.0);
    fixtureMachine.controller.setToolLength(11, 0.0);
    fixture("freecad-pocket.ngc", fixtureMachine);
    fixture("fusion-drill.ngc", fixtureMachine);
    var compatibility = new CncTestCompiler(fixtureMachine).compileDetailed(
      "%\nN10 O100\n/ G0 X1\nG40 G94\nM30\n%");
    check(compatibility.diagnostics.length == 1 &&
      compatibility.diagnostics[0].code == "CNC_BLOCK_DELETE_IGNORED",
      "program delimiters, line numbers, and block delete parse");
    rejects(fixtureMachine, "G93 M2", "G93 inverse-time");
    rejects(fixtureMachine, "G95 M2", "G95 units-per-revolution");
    rejects(fixtureMachine, "G92 X1 M2", "G92 persistent offsets");
    rejects(fixtureMachine, "G21 F600 G2 X10 R4", "radius cannot reach");
    var radiusArc = new CncTestCompiler(fixtureMachine).compileDetailed(
      "G21 F600 G2 X10 R6\nM2");
    check(radiusArc.diagnostics.length == 0 && radiusArc.ops.length == 2,
      "reachable R arc compiles");
    var majorArc = new CncTestCompiler(fixtureMachine).compileDetailed(
      "G21 F600 G2 X10 R-6\nM2");
    var minorLength = switch radiusArc.ops[0] {
      case ToolpathOp.Move(Cut, geometry, _, _, _): GeometryTools.length(geometry);
      case _: throw "minor R arc missing";
    };
    var majorLength = switch majorArc.ops[0] {
      case ToolpathOp.Move(Cut, geometry, _, _, _): GeometryTools.length(geometry);
      case _: throw "major R arc missing";
    };
    check(majorLength > minorLength && majorLength > Math.PI * 0.006,
      "negative R selects the major arc");
    fixtureMachine.controller.setHomePosition(28, 0.1, 0.2, 0.3);
    var home = new CncTestCompiler(fixtureMachine).compileDetailed(
      "G21 G90 G0 X10 Y10 Z10\nG28 X0\nM2");
    var homeLast = switch home.ops[home.ops.length - 2] {
      case ToolpathOp.MachineMove(Rapid, geometry, _, _, _): GeometryTools.pointAt(geometry,
        GeometryTools.length(geometry));
      case _: throw "G28 home motion missing";
    };
    near(homeLast.x, 0.1, "G28 uses stored machine X");
    near(homeLast.y, 0.01, "G28 leaves unspecified Y axis alone");
    var xyHelix = new CncTestCompiler(fixtureMachine).compileDetailed(
      "G21 G90 F600 G2 X5 Y5 Z10 I5 J0\nM2");
    check(xyHelix.diagnostics.length == 0, "G17 helix compiles");
    var xyGeometry = switch xyHelix.ops[0] {
      case ToolpathOp.Move(Cut, PathGeometry.Circular(_, _, _, _, ArcPlane.XY, rise), _, _, _):
        near(rise, 0.01, "G17 helix rises on Z");
        switch xyHelix.ops[0] {
          case ToolpathOp.Move(Cut, geometry, _, _, _): geometry;
          case _: throw "G17 helix missing";
        };
      case _: throw "G17 helix needs circular IR";
    };
    var xyEnd = GeometryTools.pointAt(xyGeometry,
      GeometryTools.length(xyGeometry));
    near(xyEnd.x, 0.005, "G17 helix preview X");
    near(xyEnd.y, 0.005, "G17 helix preview Y");
    near(xyEnd.z, 0.01, "G17 helix preview Z");
    var xyProgram:motionkit.program.MotionProgram = cast xyHelix.program;
    var xyPath = switch xyProgram.ops[0] {
      case MotionOp.FollowPath(path, _, _, _): path;
      case _: throw "G17 helix path missing";
    };
    var xyPrimitive:toolpathkit.motion.ToolpathPosePrimitive = cast xyPath.primitives[0];
    var xyCircular:CircularSegment = cast xyPrimitive.geometry;
    near(xyCircular.length(), GeometryTools.length(xyGeometry),
      "G17 helix lowering keeps 3D length");
    var helixSpan = xyHelix.sourceMap.provenanceAt(0, xyCircular.length() * 0.5);
    check(helixSpan != null && helixSpan.line == 1,
      "helical path maps to its G-code line");
    var xzCw = new CncTestCompiler(fixtureMachine).compileDetailed(
      "G21 G90 G18 F600 G2 X5 Z5 I5 K0\nM2");
    var xzCcw = new CncTestCompiler(fixtureMachine).compileDetailed(
      "G21 G90 G18 F600 G3 X5 Z5 I5 K0\nM2");
    var xzCwSweep = switch xzCw.ops[0] {
      case ToolpathOp.Move(Cut, PathGeometry.Circular(_, _, _, sweep, ArcPlane.XZ, _), _, _, _): sweep;
      case _: throw "G18 CW arc missing";
    };
    var xzCcwSweep = switch xzCcw.ops[0] {
      case ToolpathOp.Move(Cut, PathGeometry.Circular(_, _, _, sweep, ArcPlane.XZ, _), _, _, _): sweep;
      case _: throw "G18 CCW arc missing";
    };
    check(xzCwSweep > Math.PI && xzCcwSweep < 0.0 &&
      Math.abs(xzCcwSweep) < Math.PI,
      "G18 direction uses LinuxCNC positive-Y viewpoint");
    var xzHelix = new CncTestCompiler(fixtureMachine).compileDetailed(
      "G21 G91 G18 F600 G3 X5 Y10 Z5 I5 K0\nM2");
    var xzGeometry = switch xzHelix.ops[0] {
      case ToolpathOp.Move(Cut, g, _, _, _): g;
      case _: throw "G18 relative helix missing";
    };
    var xzEnd = GeometryTools.pointAt(xzGeometry,
      GeometryTools.length(xzGeometry));
    near(xzEnd.x, 0.005, "G18 helix preview X");
    near(xzEnd.y, 0.01, "G18 helix rises on Y");
    near(xzEnd.z, 0.005, "G18 helix preview Z");
    var yzHelix = new CncTestCompiler(fixtureMachine).compileDetailed(
      "G21 G90 G19 F600 G2 X10 Y5 Z5 J5 K0\nM2");
    var yzGeometry = switch yzHelix.ops[0] {
      case ToolpathOp.Move(Cut, PathGeometry.Circular(_, _, _, sweep, ArcPlane.YZ, rise), _, _, _):
        check(sweep < 0.0, "G19 CW uses positive-X viewpoint");
        near(rise, 0.01, "G19 helix rises on X");
        switch yzHelix.ops[0] {
          case ToolpathOp.Move(Cut, g, _, _, _): g;
          case _: throw "G19 helix missing";
        };
      case _: throw "G19 helix needs circular IR";
    };
    var yzEnd = GeometryTools.pointAt(yzGeometry,
      GeometryTools.length(yzGeometry));
    near(yzEnd.x, 0.01, "G19 helix preview X");
    near(yzEnd.y, 0.005, "G19 helix preview Y");
    near(yzEnd.z, 0.005, "G19 helix preview Z");
    var xzRadius = new CncTestCompiler(fixtureMachine).compileDetailed(
      "G21 G18 F600 G3 X5 Y10 Z5 R5\nM2");
    check(xzRadius.diagnostics.length == 0 && xzRadius.ops.length == 2,
      "G18 helical R arc compiles");
    rejects(fixtureMachine, "G18 F600 G2 X5 Z5 J1 I5", "G18 arc centre uses I/K");
    rejects(fixtureMachine, "G19 F600 G2 Y5 Z5 I1 J5", "G19 arc centre uses J/K");
    rejects(fixtureMachine, "G17 F600 G2 X5 Y5 K1 I5", "G17 arc centre uses I/J");
    rejects(fixtureMachine, "G18 F600 G2 X10 Z10 I5 K0", "arc endpoint is not on its centre circle");
    var cycles = new CncTestCompiler(fixtureMachine).compileDetailed(
      "G21 G90 G0 Z10\nF600 G99 G81 X10 Z-5 R2\nX20\nG80\n" +
      "G98 G82 X30 Z-4 R2 P0.2\nG80\nG83 X40 Z-4 R2 Q2\n" +
      "G80\nG73 X50 Z-4 R2 Q2\nG80\nM2");
    check(cycles.diagnostics.length == 0, "four drilling cycles compile");
    var feeds = 0, retracts = 0, dwells = 0, cycleLine = 0;
    for (op in cycles.ops) switch op {
      case ToolpathOp.Move(Plunge, _, _, _, span):
        feeds++;
        if (span.line == 2) cycleLine++;
      case ToolpathOp.Move(Retract, _, _, _, _): retracts++;
      case ToolpathOp.Dwell(_, _): dwells++;
      case _:
    }
    check(feeds == 9 && dwells == 1,
      "G81/G82/G83/G73 expand into feeds and dwell");
    check(retracts >= 4, "drilling cycles mark upward retracts");
    check(cycleLine == 2, "repeated G81 hole uses initiating source span");
    rejects(fixtureMachine, "G21 F600 G83 X0 Z-5 R2 M2", "requires positive Q");
    rejects(fixtureMachine, "G21 F600 G82 X0 Z-5 R2 M2", "requires positive P");
    rejects(fixtureMachine, "G21 G91 G53 G0 X1 M2", "G53 requires G90");
    var activeCycleG53 = new CncTestCompiler(machine).compileDetailed(
      "G21 G90 F600 G81 X0 Y0 Z-1 R2\nG53 X0\nG80\nM2");
    check(hasError(activeCycleG53, 2, "G53 requires G80"),
      "G53 in an active cycle reports its own line");
    var secondLineOps = 0;
    for (op in activeCycleG53.ops) switch op {
      case ToolpathOp.Move(_, _, _, _, span):
        if (span.line == 2) secondLineOps++;
      case _:
    }
    check(secondLineOps == 0, "rejected G53 does not drill another hole");
    rejects(fixtureMachine, "G80 G0 X1\nM2", "multiple motion G codes");
    var endOutputs = new CncTestCompiler(fixtureMachine).compileDetailed(
      "S1000 M3 M7 M8\nM30");
    var endOps = endOutputs.ops;
    check(endOps.length >= 3 && switch endOps[endOps.length - 3] {
      case ToolpathOp.Spindle(Off, speed, span) if (speed == 0.0): span.line == 2;
      case _: false;
    }, "M30 stops spindle speed on the end line");
    check(switch endOps[endOps.length - 3] {
      case ToolpathOp.Spindle(Off, _, _): true;
      case _: false;
    }, "M30 stops spindle direction");
    check(switch endOps[endOps.length - 2] {
      case ToolpathOp.Coolant(false, _, _): true;
      case _: false;
    }, "M30 turns off mist coolant");
    check(switch endOps[endOps.length - 2] {
      case ToolpathOp.Coolant(_, false, _): true;
      case _: false;
    }, "M30 turns off flood coolant before End");
    var m2Outputs = new CncTestCompiler(fixtureMachine).compileDetailed(
      "S1000 M3 M8\nM2");
    check(switch m2Outputs.ops[m2Outputs.ops.length - 2] {
      case ToolpathOp.Coolant(_, false, _): true;
      case _: false;
    }, "M2 also turns off active coolant");
    var compensatedMachine = new CncTestRig("work", "x", "y", "z", 0.2);
    compensatedMachine.toolLibrary.set(new Tool(2, 0.012, 0.002));
    var compUnsupported = new CncTestCompiler(compensatedMachine).compileDetailed(
      "G21 G90 F600 T2 M6\nG0 X0 Y0\nG41 D2 G1 X5\nG1 Y5\n" +
      "G28\nG81 X5 Y5 Z-1 R2\nG40 G1 X10\nM2");
    check(hasError(compUnsupported, 5, "G28/G30 require G40"),
      "G28 under cutter compensation reports its own line");
    check(hasError(compUnsupported, 6, "drilling cycles require G40"),
      "drilling cycle under cutter compensation reports its own line");
    near(compensatedMachine.controller.toolLength(2), 0.012,
      "tool table supplies G43 length");
    near(compensatedMachine.toolLibrary.tool(2).diameter, 0.002,
      "tool table stores cutter diameter");
    compensatedMachine.controller.setToolLength(2, 0.013);
    near(compensatedMachine.toolLibrary.tool(2).diameter, 0.002,
      "legacy tool-length setter preserves cutter diameter");
    var inside = new CncTestCompiler(compensatedMachine).compileDetailed(
      File.getContent("fixtures/comp-inside.ngc"));
    check(inside.diagnostics.length == 0, "inside cutter fixture compiles");
    var insideFirst = switch inside.ops[1] {
      case ToolpathOp.Move(Cut, g, _, _, _): GeometryTools.pointAt(g,
        GeometryTools.length(g));
      case _: throw "inside first contour missing";
    };
    near(insideFirst.x, 0.019, "inside corner trims first line X");
    near(insideFirst.y, 0.001, "inside corner trims first line Y");
    var insideSecond = switch inside.ops[2] {
      case ToolpathOp.Move(Cut, g, _, _, _): GeometryTools.pointAt(g, 0.0);
      case _: throw "inside second contour missing";
    };
    near(insideSecond.x, insideFirst.x, "inside corner remains connected X");
    near(insideSecond.y, insideFirst.y, "inside corner remains connected Y");
    var outside = new CncTestCompiler(compensatedMachine).compileDetailed(
      File.getContent("fixtures/comp-outside.ngc"));
    check(outside.diagnostics.length == 0 && outside.ops.length == 6,
      "outside corner adds a round cutter-radius join");
    var outsideJoin = switch outside.ops[2] {
      case ToolpathOp.Move(Cut, PathGeometry.Arc(_, radius, _, sweep), _, _, _):
        near(sweep, -Math.PI * 0.5, "outside join follows corner turn");
        radius;
      case _: throw "outside corner join missing";
    };
    near(outsideJoin, 0.001, "outside join uses cutter radius");
    var rightComp = new CncTestCompiler(compensatedMachine).compileDetailed(
      "G21 F600 G42 D2 G1 X10\nG1 X20\nG40 G1 X30\nM2");
    var rightStart = switch rightComp.ops[1] {
      case ToolpathOp.Move(Cut, g, _, _, _): GeometryTools.pointAt(g, 0.0);
      case _: throw "G42 contour missing";
    };
    near(rightStart.y, -0.001, "G42 offsets to the right");
    var loadedComp = new CncTestCompiler(compensatedMachine).compileDetailed(
      "G21 F600 T2 M6\nG41 G1 X10\nG1 X20\nG40 G1 X30\nM2");
    check(loadedComp.diagnostics.length == 0,
      "G41 uses the declared loaded tool without D");
    var withControls = new CncTestCompiler(compensatedMachine).compileDetailed(
      "G21 F600 G41 D2 G1 X10\nM8\nG1 X20\nG40\nM9\nG1 X30\nM2");
    check(withControls.diagnostics.length == 0 &&
      withControls.ops.length == 6,
      "coolant operations remain ordered around compensated motion");
    var arcComp = new CncTestCompiler(compensatedMachine).compileDetailed(
      File.getContent("fixtures/comp-arc.ngc"));
    check(arcComp.diagnostics.length == 0, "arc compensation fixture compiles");
    var arcRadius = switch arcComp.ops[1] {
      case ToolpathOp.Move(Cut, PathGeometry.Arc(_, radius, _, _), _, _, _): radius;
      case _: throw "compensated arc missing";
    };
    near(arcRadius, 0.009, "G41 offsets CCW arc inward");
    var mixedComp = new CncTestCompiler(compensatedMachine).compileDetailed(
      File.getContent("fixtures/comp-line-arc.ngc"));
    check(mixedComp.diagnostics.length == 0,
      "inside line-to-arc corner offsets without gouging");
    var mixedLineEnd = switch mixedComp.ops[1] {
      case ToolpathOp.Move(Cut, g, _, _, _): GeometryTools.pointAt(g,
        GeometryTools.length(g));
      case _: throw "mixed compensated line missing";
    };
    var mixedArcStart = switch mixedComp.ops[2] {
      case ToolpathOp.Move(Cut, g, _, _, _): GeometryTools.pointAt(g, 0.0);
      case _: throw "mixed compensated arc missing";
    };
    near(mixedLineEnd.distanceTo(mixedArcStart), 0.0,
      "inside line and arc meet after trimming", 1e-8);
    rejects(compensatedMachine,
      "G21 F600 G41 D2 G1 X10\nG1 X10.5\nG1 Y10\nG40 G1 Y20\nM2",
      "gouge at inside corner");
    var xzComp = new CncTestCompiler(compensatedMachine).compileDetailed(
      "G21 G18 F600 G41 D2 G1 X10 Z0\nG3 X15 Z5 I5 K0\n" +
      "G40 G1 X15 Z20\nM2");
    var xzCompRadius = switch xzComp.ops[1] {
      case ToolpathOp.Move(Cut, PathGeometry.Circular(_, radius, _, _, ArcPlane.XZ, _), _, _, _): radius;
      case _: throw "G18 compensated arc missing";
    };
    near(xzCompRadius, 0.004, "G18 G41 follows positive-Y side convention");
    rejects(compensatedMachine, "G19 G41 D2", "unsupported in G19 YZ plane");
    rejects(compensatedMachine,
      "G21 F600 G41 D2 G1 X0.5\nG1 X10\nG40 G1 X20\nM2",
      "lead-in is shorter than tool radius");
    rejects(compensatedMachine,
      "G21 F600 G41 D2 G1 X10\nG3 X20 Y10 I0 J10\nG40 G1 X21 Y10\nM2",
      "lead-out is shorter than tool diameter");
    compensatedMachine.toolLibrary.set(new Tool(3, 0.0, 0.022));
    rejects(compensatedMachine,
      "G21 F600 G41 D3 G1 X30\nG3 X40 Y10 I0 J10\nG40 G1 X40 Y40\nM2",
      "gouges arc radius");
    var travelMachine = new CncTestRig("work", "x", "y", "z", 0.2);
    travelMachine.setTravelEnvelope([0.0, 0.0, 0.0], [0.02, 0.02, 0.02]);
    var travelResult = new CncTestCompiler(travelMachine).compileDetailed(
      "G21 G0 X25\nG0 X10\nM2");
    check(travelResult.diagnostics.length >= 1 &&
      travelResult.diagnostics[0].code == "CNC_TRAVEL" &&
      travelResult.diagnostics[0].span.line == 1,
      "travel error identifies the G-code line");
    rejects(travelMachine, "G21 F600 G3 X10 Y0 I5 J0", "Y travel");
    writerRoundTrip();
    Sys.println('CncKit tests passed ($assertions assertions)');
  }

  static function writerRoundTrip():Void {
    var machine = new CncTestRig("work", "x", "y", "z", 0.2,
      [0.0, 0.0, 0.01]);
    machine.controller.setWorkOffset(54, 0.1, 0.0, 0.0);
    machine.controller.setWorkOffset(55, 0.2, 0.0, 0.0);
    machine.toolLibrary.set(new Tool(1, 0.0, 0.002));
    var p = Provenance.cam(10);
    var authored = [
      ToolpathOp.SetSetup("1", p),
      ToolpathOp.ToolChange(1, p),
      ToolpathOp.Spindle(Clockwise, 12000, p),
      ToolpathOp.Coolant(true, false, p),
      ToolpathOp.Move(Rapid, PathGeometry.Line(
        new Point3(-0.1, 0.0, 0.01), new Point3(0.01, 0.0, 0.01)),
        0.0, 0.0, p),
      ToolpathOp.Move(Cut, PathGeometry.Line(
        new Point3(0.01, 0.0, 0.01), new Point3(0.02, 0.0, 0.01)),
        0.01, 0.0002, p),
      ToolpathOp.Move(Cut, PathGeometry.Arc(
        new Point3(0.02, 0.005, 0.01), 0.005, -Math.PI / 2,
        Math.PI / 2), 0.01, 0.0002, p),
      ToolpathOp.Move(Cut, PathGeometry.Line(
        new Point3(0.025, 0.005, 0.01),
        new Point3(0.03, 0.005, 0.01)), 0.01, 0.0, p),
      ToolpathOp.SetSetup("2", p),
      ToolpathOp.Move(Rapid, PathGeometry.Line(
        new Point3(-0.07, 0.005, 0.01),
        new Point3(0.01, 0.005, 0.01)), 0.0, 0.0, p),
      ToolpathOp.Coolant(false, false, p),
      ToolpathOp.Spindle(Off, 0.0, p),
      ToolpathOp.End(p)
    ];
    var setup = new Setup("1", new Point3(0.1, 0, 0),
      new SetupStock(-0.2, 0.2, -0.1, 0.1, 0.0, -0.01, 0.005));
    var secondSetup = new Setup("2", new Point3(0.2, 0, 0));
    var gcode = CncWriter.write(new ToolpathProgram(authored,
      machine.toolLibrary, [setup, secondSetup]), machine.controller);
    check(gcode.indexOf("G54") >= 0 && gcode.indexOf("G55") >= 0 &&
      gcode.indexOf("G64 P0.2") >= 0 && gcode.split("G61").length == 3,
      "writer maps setups and per-move tolerance to LinuxCNC");
    var back = machine.compileDetailed(gcode);
    check(back.diagnostics.length == 0,
      'hand-built toolpath recompiles: ${back.diagnostics}');
    check(back.program.ops.length == authored.length,
      "hand-built round trip keeps operation count");
    for (index in 0...authored.length) switch [authored[index], back.program.ops[index]] {
      case [ToolpathOp.SetSetup(a, _), ToolpathOp.SetSetup(b, _)]:
        check(a == b, 'setup $index');
      case [ToolpathOp.Move(_, a, feedA, toleranceA, _),
            ToolpathOp.Move(_, b, feedB, toleranceB, _)]:
        var lengthA = GeometryTools.length(a), lengthB = GeometryTools.length(b);
        near(lengthA, lengthB, 'path length $index', 1e-8);
        var endA = GeometryTools.pointAt(a, lengthA);
        var endB = GeometryTools.pointAt(b, lengthB);
        near(endA.x, endB.x, 'path X $index', 1e-8);
        near(endA.y, endB.y, 'path Y $index', 1e-8);
        near(endA.z, endB.z, 'path Z $index', 1e-8);
        near(feedA, feedB, 'path feed $index', 1e-8);
        near(toleranceA, toleranceB, 'path tolerance $index', 1e-9);
      case [ToolpathOp.ToolChange(a, _), ToolpathOp.ToolChange(b, _)]:
        check(a == b, 'tool $index');
      case [ToolpathOp.Spindle(a, rpmA, _), ToolpathOp.Spindle(b, rpmB, _)]:
        check(a == b, 'spindle direction $index');
        near(rpmA, rpmB, 'spindle speed $index');
      case [ToolpathOp.Coolant(mistA, floodA, _),
            ToolpathOp.Coolant(mistB, floodB, _)]:
        check(mistA == mistB && floodA == floodB, 'coolant $index');
      case [ToolpathOp.End(_), ToolpathOp.End(_)]: check(true, "end");
      case _: check(false, 'operation $index round trip');
    }
    // Programmed points survive G43: the writer emits them unchanged and the
    // compiler removes the length again, while the machine still sees it.
    machine.toolLibrary.set(new Tool(2, 0.03, 0.006));
    var measured = machine.compileDetailed(
      "G21 G90 G54 T2 M6 G43 H2\nG0 X5 Y5 Z10\nF300 G1 Z-2\nG49 G0 Z60\nM2");
    check(measured.diagnostics.length == 0, 'G43 program compiles: ${measured.diagnostics}');
    var programmedZ:Array<Float> = [];
    for (op in measured.program.ops) switch op {
      case ToolpathOp.Move(_, Line(_, end), _, _, _): programmedZ.push(end.z);
      case _:
    }
    check(programmedZ.length == 3, "G43 program keeps three moves");
    near(programmedZ[0], 0.01, "G43 moves hold the programmed tip Z", 1e-12);
    near(programmedZ[1], -0.002, "G43 feed holds the programmed tip Z", 1e-12);
    near(programmedZ[2], 0.06, "G49 moves hold the programmed gauge Z", 1e-12);
    var rewritten = CncWriter.write(measured.program, machine.controller);
    check(rewritten.indexOf("Z-2") >= 0 && rewritten.indexOf("Z10") >= 0,
      'writer emits programmed Z under G43: $rewritten');
    var reread = machine.compileDetailed(rewritten);
    var rereadZ:Array<Float> = [];
    for (op in reread.program.ops) switch op {
      case ToolpathOp.Move(_, Line(_, end), _, _, _): rereadZ.push(end.z);
      case _:
    }
    check(rereadZ.length == 3, "G43 round trip keeps three moves");
    for (index in 0...3)
      near(rereadZ[index], programmedZ[index], 'G43 round trip Z $index', 1e-12);
    var rejected = false;
    try CncWriter.write(new ToolpathProgram([ToolpathOp.SetSetup("fixture-only", p)],
      machine.toolLibrary, [new Setup("fixture-only", new Point3(0, 0, 0))]),
      machine.controller)
    catch (error:Dynamic) rejected = Std.string(error).indexOf("cannot map setup") >= 0;
    check(rejected, "writer rejects setups without a controller mapping");
    rejected = false;
    try CncWriter.write(new ToolpathProgram([ToolpathOp.MachineMove(Cut,
      PathGeometry.Arc(new Point3(0.02, 0.005, 0.01), 0.005,
        -Math.PI / 2, Math.PI / 2), 0.01, 0.0, p)], machine.toolLibrary,
      [setup]), machine.controller)
    catch (error:Dynamic) rejected = Std.string(error).indexOf("line for machine move") >= 0;
    check(rejected, "writer rejects machine arcs it cannot express");
    rejected = false;
    try CncWriter.write(new ToolpathProgram([ToolpathOp.ToolLengthOffset(1, 0.001, p)],
      machine.toolLibrary, [setup]), machine.controller)
    catch (error:Dynamic) rejected = Std.string(error).indexOf("disagrees") >= 0;
    check(rejected, "writer rejects a mismatched H tool length");
  }

  static function fixture(name:String, machine:CncTestRig):Void {
    var result = new CncTestCompiler(machine).compileDetailed(
      File.getContent('fixtures/$name'));
    check(result.diagnostics.length == 0, '$name diagnostics: ${result.diagnostics}');
    var program:motionkit.program.MotionProgram = cast result.program;
    var motions = 0, length = 0.0;
    var low = [1e9, 1e9, 1e9], high = [-1e9, -1e9, -1e9];
    for (op in result.ops) {
      var geometry = switch op {
        case ToolpathOp.Move(_, g, _, _, _),
            ToolpathOp.MachineMove(_, g, _, _, _): g;
        case _: null;
      };
      if (geometry == null) continue;
      motions++;
      var distance = GeometryTools.length(geometry);
      length += distance;
      for (point in [GeometryTools.pointAt(geometry, 0.0),
          GeometryTools.pointAt(geometry, distance)]) {
        var coords = [point.x, point.y, point.z];
        for (axis in 0...3) {
          low[axis] = Math.min(low[axis], coords[axis]);
          high[axis] = Math.max(high[axis], coords[axis]);
        }
      }
    }
    var expectedMotions = name == "freecad-pocket.ngc" ? 9 : 13;
    var expectedLength = name == "freecad-pocket.ngc" ?
      0.078 : 0.31149203622419913;
    var expectedLow = name == "freecad-pocket.ngc" ?
      [-0.01, 0.0, -0.004] : [0.0, 0.0, -0.01];
    var expectedHigh = name == "freecad-pocket.ngc" ?
      [0.01, 0.0, 0.0] : [0.09648, 0.068423, 0.01];
    check(motions == expectedMotions, '$name golden motion count');
    near(length, expectedLength, '$name golden total length');
    for (axis in 0...3) {
      near(low[axis], expectedLow[axis], '$name lower bound $axis');
      near(high[axis], expectedHigh[axis], '$name upper bound $axis');
    }
    var loweredMoves = primitives(program), primitiveIndex = 0;
    for (op in result.ops) {
      var geometry = switch op {
        case ToolpathOp.Move(_, g, _, _, _),
            ToolpathOp.MachineMove(_, g, _, _, _): g;
        case _: null;
      };
      if (geometry == null) continue;
      check(primitiveIndex < loweredMoves.length, '$name lowered move missing');
      var primitive = loweredMoves[primitiveIndex++];
      var distance = GeometryTools.length(geometry);
      near(primitive.length(), distance, '$name lowered move length', 1e-8);
      for (fraction in [0.0, 0.5, 1.0]) {
        var authored = GeometryTools.pointAt(geometry, distance * fraction);
        var lowered = primitive.waypointAt(primitive.length() * fraction).pose;
        check(Math.abs(authored.x - lowered.x) < 1e-8 &&
          Math.abs(authored.y - lowered.y) < 1e-8 &&
          Math.abs(authored.z - lowered.z) < 1e-8,
          '$name lowered path stays on authored geometry');
      }
    }
  }
}
