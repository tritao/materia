package cnckit.lower;

import cnckit.CncChannels;
import cnckit.CncDiagnostic;
import cnckit.CncDiagnostic.CncSeverity;
import cnckit.CncMachine;
import cnckit.CncPosePrimitive;
import cnckit.CncSourceMap;
import cnckit.ir.CncGeometry;
import cnckit.ir.CncOp;
import cnckit.parse.CncSpan;
import motionkit.event.EventValue;
import motionkit.path.ArcSegment;
import motionkit.path.CircularPlane;
import motionkit.path.CircularSegment;
import motionkit.path.CornerBlender;
import motionkit.path.GeometricPath;
import motionkit.path.LineSegment;
import motionkit.path.PathPoint;
import motionkit.path.PathPrimitive;
import motionkit.path.PosePrimitive;
import motionkit.path.PosePath;
import motionkit.program.InputPredicate;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;

/** Converts plain CNC operations to MotionKit paths, barriers, and source map. */
class CncLowering {
  public final machine:CncMachine;
  var motionOps:Array<MotionOp> = [];
  var sourceMap:CncSourceMap = new CncSourceMap();
  var diagnostics:Array<CncDiagnostic> = [];
  var pending:Array<PathPrimitive> = [];
  var pendingSpans:Array<CncSpan> = [];
  var pendingFeed:Float = 0.0;
  var pendingBlend:Float = 0.0;

  public function new(machine:CncMachine) {
    this.machine = machine;
  }

  public function lower(ops:Array<CncOp>):CncLoweringResult {
    motionOps = [];
    sourceMap = new CncSourceMap();
    diagnostics = [];
    pending = []; pendingSpans = [];
    pendingFeed = 0.0; pendingBlend = 0.0;
    for (op in ops) switch op {
      case Rapid(geometry, span):
        addMove(primitive(geometry), machine.rapidSpeed, 0.0, span);
      case Feed(geometry, speed, blend, span):
        addMove(primitive(geometry), speed, blend, span);
      case Dwell(seconds, span):
        flush(); add(MotionOp.Dwell(seconds), span);
      case Spindle(channel, value, span):
        flush(); add(MotionOp.SetOutput(channel, EventValue.Analog(value)), span);
      case Coolant(channel, enabled, span):
        flush(); add(MotionOp.SetOutput(channel, EventValue.Digital(enabled)), span);
      case ToolChange(number, span):
        flush(); add(MotionOp.WaitInput(CncChannels.toolChange(number),
          InputPredicate.Equals(EventValue.Digital(true)), null), span);
      case OptionalStop(span), ProgramStop(span):
        flush(); add(MotionOp.WaitInput(CncChannels.OperatorResume,
          InputPredicate.Equals(EventValue.Digital(true)), null), span);
      case End(_): flush();
    }
    flush();
    var program = motionOps.length == 0 ? null : new MotionProgram(motionOps);
    return new CncLoweringResult(program, sourceMap, diagnostics.copy());
  }

  function addMove(geometry:PathPrimitive, speed:Float, blend:Float,
      span:CncSpan):Void {
    if (pending.length > 0 && (blend == 0.0 || pendingBlend == 0.0 ||
        Math.abs(speed - pendingFeed) > 1e-12 || blend != pendingBlend)) flush();
    pending.push(geometry); pendingSpans.push(span);
    pendingFeed = speed; pendingBlend = blend;
    if (blend == 0.0) flush();
  }

  function flush():Void {
    if (pending.length == 0) return;
    if (pending.length > 1 && pendingBlend > 0.0) {
      var authored = new GeometricPath(pending);
      var blended = CornerBlender.blend(authored, pendingBlend,
        machine.maxBlendTurnAngleRadians);
      var spans = [for (index in blended.sourcePrimitiveIndices)
        pendingSpans[index]];
      for (diagnostic in blended.diagnostics) {
        var corner = Std.parseInt(diagnostic.split(":")[0].substr("corner ".length));
        var index = corner == null || corner < 1 || corner >= pendingSpans.length
          ? 0 : corner;
        diagnostics.push(new CncDiagnostic(Warning, "CNC_EXACT_STOP",
          pendingSpans[index], diagnostic));
      }
      emitPath(blended.path.primitives, spans, pendingFeed, authored,
        pendingBlend);
    } else for (index in 0...pending.length)
      emitPath([pending[index]], [pendingSpans[index]], pendingFeed);
    pending = []; pendingSpans = [];
  }

  function emitPath(geometry:Array<PathPrimitive>, spans:Array<CncSpan>,
      speed:Float, ?authored:GeometricPath, ?blend:Float = 0.0):Void {
    var primitives:Array<PosePrimitive> = [for (primitive in geometry)
      new CncPosePrimitive(primitive, speed, machine.positionTolerance,
        machine.orientationTolerance)];
    var path = new PosePath(machine.frameId, primitives);
    if (authored != null) path.withAuthoredGeometry(authored, blend);
    var index = motionOps.length, distance = 0.0;
    for (i in 0...geometry.length) {
      var end = distance + geometry[i].length();
      sourceMap.add(index, distance, end, spans[i]);
      distance = end;
    }
    motionOps.push(MotionOp.FollowPath(path, machine.frameId, speed, []));
  }

  function add(op:MotionOp, span:CncSpan):Void {
    sourceMap.add(motionOps.length, 0.0, 0.0, span);
    motionOps.push(op);
  }

  static function primitive(geometry:CncGeometry):PathPrimitive return switch geometry {
    case Line(start, end): new LineSegment(point(start), point(end));
    case Arc(center, radius, startAngle, sweep):
      new ArcSegment(point(center), radius, startAngle, sweep);
    case Circular(center, radius, startAngle, sweep, plane, rise):
      new CircularSegment(point(center), radius, startAngle, sweep,
        switch plane {
          case XY: CircularPlane.XY;
          case XZ: CircularPlane.XZ;
          case YZ: CircularPlane.YZ;
        }, rise);
  };

  static function point(value:cnckit.ir.CncPoint):PathPoint
    return new PathPoint(value.x, value.y, value.z);
}
