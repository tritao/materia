package toolpathkit.motion;

import toolpathkit.motion.ToolpathChannels;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.ToolpathPosePrimitive;
import toolpathkit.motion.ToolpathSourceMap;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Provenance;
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
class ToolpathLowering {
  public final machine:MachineBinding;
  var motionOps:Array<MotionOp> = [];
  var sourceMap:ToolpathSourceMap = new ToolpathSourceMap();
  var diagnostics:Array<ToolpathDiagnostic> = [];
  var pending:Array<PathPrimitive> = [];
  var pendingSpans:Array<Provenance> = [];
  var pendingFeed:Float = 0.0;
  var pendingBlend:Float = 0.0;

  public function new(machine:MachineBinding) {
    this.machine = machine;
  }

  public function lower(ops:Array<ToolpathOp>):ToolpathLoweringResult {
    motionOps = [];
    sourceMap = new ToolpathSourceMap();
    diagnostics = [];
    pending = []; pendingSpans = [];
    pendingFeed = 0.0; pendingBlend = 0.0;
    for (op in ops) switch op {
      case SetSetup(_, _):
        flush();
      case MachineMove(_, _, _, _, _):
        throw "machine moves must be projected before lowering";
      case Move(Rapid, geometry, _, _, span), Move(Link, geometry, _, _, span),
          Move(Retract, geometry, _, _, span):
        addMove(primitive(geometry), machine.rapidSpeed, 0.0, span);
      case Move(_, geometry, speed, blend, span):
        addMove(primitive(geometry), speed, blend, span);
      case Dwell(seconds, span):
        flush(); add(MotionOp.Dwell(seconds), span);
      case Spindle(direction, rpm, span):
        flush();
        add(MotionOp.SetOutput(ToolpathChannels.SpindleDirection,
          EventValue.Analog(switch direction {
            case Off: 0.0;
            case Clockwise: 1.0;
            case CounterClockwise: -1.0;
          })), span);
        add(MotionOp.SetOutput(ToolpathChannels.SpindleSpeed,
          EventValue.Analog(rpm)), span);
      case Coolant(mist, flood, span):
        flush();
        add(MotionOp.SetOutput(ToolpathChannels.CoolantMist,
          EventValue.Digital(mist)), span);
        add(MotionOp.SetOutput(ToolpathChannels.CoolantFlood,
          EventValue.Digital(flood)), span);
      case ToolChange(number, span):
        flush(); add(MotionOp.WaitInput(ToolpathChannels.toolChange(number),
          InputPredicate.Equals(EventValue.Digital(true)), null), span);
      case ToolLengthOffset(_, _, _):
        // Already applied to Z; the controller has nothing to do.
      case OptionalStop(span), ProgramStop(span):
        flush(); add(MotionOp.WaitInput(ToolpathChannels.OperatorResume,
          InputPredicate.Equals(EventValue.Digital(true)), null), span);
      case End(_): flush();
    }
    flush();
    var program = motionOps.length == 0 ? null : new MotionProgram(motionOps);
    return new ToolpathLoweringResult(program, sourceMap, diagnostics.copy());
  }

  function addMove(geometry:PathPrimitive, speed:Float, blend:Float,
      span:Provenance):Void {
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
      for (warning in 0...blended.diagnostics.length) {
        var index = blended.diagnosticCorners[warning];
        diagnostics.push(new ToolpathDiagnostic( "CNC_EXACT_STOP",
          pendingSpans[index], blended.diagnostics[warning]));
      }
      emitPath(blended.path.primitives, spans, pendingFeed, authored,
        pendingBlend);
    } else for (index in 0...pending.length)
      emitPath([pending[index]], [pendingSpans[index]], pendingFeed);
    pending = []; pendingSpans = [];
  }

  function emitPath(geometry:Array<PathPrimitive>, spans:Array<Provenance>,
      speed:Float, ?authored:GeometricPath, ?blend:Float = 0.0):Void {
    var primitives:Array<PosePrimitive> = [for (primitive in geometry)
      new ToolpathPosePrimitive(primitive, speed, machine.positionTolerance,
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

  function add(op:MotionOp, span:Provenance):Void {
    sourceMap.add(motionOps.length, 0.0, 0.0, span);
    motionOps.push(op);
  }

  static function primitive(geometry:PathGeometry):PathPrimitive return switch geometry {
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

  static function point(value:toolpathkit.path.Point3):PathPoint
    return new PathPoint(value.x, value.y, value.z);
}
