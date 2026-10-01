package toolpathkit.motion;

import toolpathkit.motion.ToolpathChannels;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.ToolpathPosePrimitive;
import toolpathkit.motion.ToolpathSourceMap;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Provenance;
import motionkit.event.EventValue;
import motionkit.event.PathEvent;
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

private typedef PendingOutputEvent = {
  var boundary:Int;
  var channel:String;
  var value:EventValue;
  var span:Provenance;
}

/** Converts plain CNC operations to MotionKit paths, barriers, and source map. */
class ToolpathLowering {
  public final machine:MachineBinding;
  var motionOps:Array<MotionOp> = [];
  var sourceMap:ToolpathSourceMap = new ToolpathSourceMap();
  var diagnostics:Array<ToolpathDiagnostic> = [];
  var pending:Array<PathPrimitive> = [];
  var pendingSpans:Array<Provenance> = [];
  var pendingTolerances:Array<Float> = [];
  var pendingSpeeds:Array<Float> = [];
  var pendingEvents:Array<PendingOutputEvent> = [];
  var queuedEvents:Array<PendingOutputEvent> = [];
  var pendingBlend:Float = 0.0;
  var spindleOn:Bool = false;

  public function new(machine:MachineBinding) {
    this.machine = machine;
  }

  public function lower(ops:Array<ToolpathOp>):ToolpathLoweringResult {
    motionOps = [];
    sourceMap = new ToolpathSourceMap();
    diagnostics = [];
    pending = []; pendingSpans = []; pendingTolerances = []; pendingSpeeds = [];
    pendingEvents = []; queuedEvents = []; spindleOn = false;
    pendingBlend = 0.0;
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
        flush(); drainQueued(); add(MotionOp.Dwell(seconds), span);
      case Spindle(direction, rpm, span):
        var directionValue = EventValue.Analog(switch direction {
          case Off: 0.0;
          case Clockwise: 1.0;
          case CounterClockwise: -1.0;
        });
        if (direction != Off && !spindleOn) {
          flush(); drainQueued();
          add(MotionOp.SetOutput(ToolpathChannels.SpindleDirection,
            directionValue), span);
          add(MotionOp.SetOutput(ToolpathChannels.SpindleSpeed,
            EventValue.Analog(rpm)), span);
          add(MotionOp.WaitInput(ToolpathChannels.SpindleAtSpeed,
            InputPredicate.Equals(EventValue.Digital(true)), null), span);
        } else {
          outputEvent(ToolpathChannels.SpindleDirection, directionValue, span);
          outputEvent(ToolpathChannels.SpindleSpeed, EventValue.Analog(rpm), span);
        }
        spindleOn = direction != Off;
      case Coolant(mist, flood, span):
        outputEvent(ToolpathChannels.CoolantMist,
          EventValue.Digital(mist), span);
        outputEvent(ToolpathChannels.CoolantFlood,
          EventValue.Digital(flood), span);
      case ToolChange(number, span):
        flush(); drainQueued(); add(MotionOp.WaitInput(ToolpathChannels.toolChange(number),
          InputPredicate.Equals(EventValue.Digital(true)), null), span);
      case ToolLengthOffset(_, _, _):
        // ToolpathMotion already moved Z to the controlled point.
      case OptionalStop(span), ProgramStop(span):
        flush(); drainQueued(); add(MotionOp.WaitInput(ToolpathChannels.OperatorResume,
          InputPredicate.Equals(EventValue.Digital(true)), null), span);
      case End(_): flush(); drainQueued();
    }
    flush(); drainQueued();
    var program = motionOps.length == 0 ? null : new MotionProgram(motionOps);
    return new ToolpathLoweringResult(program, sourceMap, diagnostics.copy());
  }

  /**
    Moves join one path until a barrier: the path keeps each move's speed,
    rounds the corners its tolerances allow, and stops at the others when it
    is planned, so a move that carries straight on, at another speed or
    exactly, does not stop.
  **/
  function addMove(geometry:PathPrimitive, speed:Float, blend:Float,
      span:Provenance):Void {
    if (!Math.isFinite(blend) || blend < 0.0)
      throw "toolpath move needs a nonnegative finite tolerance";
    pending.push(geometry); pendingSpans.push(span);
    pendingTolerances.push(blend); pendingSpeeds.push(speed);
    if (pending.length == 1 && queuedEvents.length > 0) {
      pendingEvents = [for (event in queuedEvents) {
        boundary:0, channel:event.channel, value:event.value, span:event.span
      }];
      queuedEvents = [];
    }
    pendingBlend = Math.max(pendingBlend, blend);
  }

  function flush():Void {
    if (pending.length == 0) return;
    // A blend between moves of different speeds would take one move's speed,
    // so only corners between moves of one speed are rounded.
    var corners = [for (i in 0...(pending.length - 1))
      pendingSpeeds[i] != pendingSpeeds[i + 1] ? 0.0 :
        Math.min(pendingTolerances[i], pendingTolerances[i + 1]) * CornerBlender.GEOMETRY_SHARE];
    var rounded = false;
    for (corner in corners) if (corner > 0.0) rounded = true;
    if (rounded) {
      var authored = new GeometricPath(pending);
      var blended = CornerBlender.blendPerCorner(authored, corners,
        machine.maxBlendTurnAngleRadians);
      var spans = [for (index in blended.sourcePrimitiveIndices)
        pendingSpans[index]];
      for (warning in 0...blended.diagnostics.length) {
        var index = blended.diagnosticCorners[warning];
        diagnostics.push(new ToolpathDiagnostic( "CNC_EXACT_STOP",
          pendingSpans[index], blended.diagnostics[warning]));
      }
      emitPath(blended.path.primitives, spans, [for (index in blended.sourcePrimitiveIndices)
        pendingSpeeds[index]], authored, pendingBlend, blended.sourcePrimitiveIndices);
    } else
      emitPath(pending, pendingSpans, pendingSpeeds, null, 0.0, [for (index in 0...pending.length) index]);
    pending = []; pendingSpans = []; pendingTolerances = []; pendingSpeeds = [];
    pendingEvents = [];
    pendingBlend = 0.0;
  }

  function emitPath(geometry:Array<PathPrimitive>, spans:Array<Provenance>,
      speeds:Array<Float>, ?authored:GeometricPath, ?blend:Float = 0.0,
      ?sourceIndices:Array<Int>):Void {
    var primitives:Array<PosePrimitive> = [for (index in 0...geometry.length)
      new ToolpathPosePrimitive(geometry[index], speeds[index], machine.positionTolerance,
        machine.orientationTolerance)];
    // Each primitive holds its own speed; the op's feed only bounds them.
    var speed = 0.0;
    for (value in speeds) speed = Math.max(speed, value);
    var events:Array<PathEvent> = [];
    for (event in pendingEvents) {
      var eventDistance = 0.0;
      for (i in 0...geometry.length)
        if (sourceIndices[i] < event.boundary)
          eventDistance += geometry[i].length();
      events.push(new PathEvent(eventDistance, event.channel, event.value));
    }
    var path = new PosePath(machine.frameId, primitives);
    if (authored != null) path.withAuthoredGeometry(authored, blend);
    var index = motionOps.length, distance = 0.0;
    for (i in 0...geometry.length) {
      var end = distance + geometry[i].length();
      sourceMap.add(index, distance, end, spans[i]);
      distance = end;
    }
    motionOps.push(MotionOp.FollowPath(path, machine.frameId, speed, events));
  }

  function outputEvent(channel:String, value:EventValue,
      span:Provenance):Void {
    if (pending.length > 0) {
      pendingEvents.push({boundary:pending.length, channel:channel,
        value:value, span:span});
      return;
    }
    var last = motionOps.length - 1;
    if (last >= 0) switch motionOps[last] {
      case FollowPath(path, frame, speed, events):
        var attached = events.copy();
        attached.push(new PathEvent(path.length(), channel, value));
        motionOps[last] = MotionOp.FollowPath(path, frame, speed, attached);
        sourceMap.add(last, path.length(), path.length(), span);
        return;
      case _:
    }
    queuedEvents.push({boundary:0, channel:channel, value:value, span:span});
  }

  function drainQueued():Void {
    for (event in queuedEvents)
      add(MotionOp.SetOutput(event.channel, event.value), event.span);
    queuedEvents = [];
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
