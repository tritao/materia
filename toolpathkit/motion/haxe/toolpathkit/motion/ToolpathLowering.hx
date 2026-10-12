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
import motionkit.MotionOptions;
import motionkit.path.CornerBlender;
import motionkit.path.DistanceDrive;
import motionkit.path.DistanceDrive.DriveInterval;
import motionkit.path.GeometricPath;
import motionkit.path.LineSegment;
import motionkit.path.PathPoint;
import motionkit.path.PathPrimitive;
import motionkit.path.PosePrimitive;
import motionkit.path.PosePath;
import motionkit.program.InputPredicate;
import motionkit.program.MotionOp;
import motionkit.program.Blend;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;

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
  /** Radians of the extruder per metre of each pending move: zero for travels and unfed moves. */
  var pendingRates:Array<Float> = [];
  /** The cross-section of the bead in force (`Deposit`), 0 when none. */
  var bead:Float = 0.0;
  var pendingEvents:Array<PendingOutputEvent> = [];
  var queuedEvents:Array<PendingOutputEvent> = [];
  var pendingBlend:Float = 0.0;
  var spindleOn:Bool = false;
  var stops:Map<String, Int> = new Map();

  public function new(machine:MachineBinding) {
    this.machine = machine;
  }

  public function lower(ops:Array<ToolpathOp>):ToolpathLoweringResult {
    motionOps = [];
    sourceMap = new ToolpathSourceMap();
    diagnostics = [];
    pending = []; pendingSpans = []; pendingTolerances = []; pendingSpeeds = []; pendingRates = [];
    pendingEvents = []; queuedEvents = []; spindleOn = false; bead = 0.0;
    pendingBlend = 0.0;
    stops = new Map();
    for (op in ops) switch op {
      case SetSetup(_, _):
        flush();
      case MachineMove(_, _, _, _, _):
        throw "machine moves must be projected before lowering";
      case Move(Rapid, geometry, speed, blend, span), Move(Link, geometry, speed, blend, span),
          Move(Retract, geometry, speed, blend, span):
        if (machine.rapidsAtFeed) addMove(primitive(geometry), Math.min(speed, machine.rapidSpeed), blend, span);
        else addMove(primitive(geometry), machine.rapidSpeed, 0.0, span);
      case Move(kind, geometry, speed, blend, span):
        var printer = machine.printer;
        if (printer != null && kind == toolpathkit.path.MoveKind.Cut && bead > 0.0) {
          // The hotend melts so much filament a second: a bead is laid no faster than that.
          addMove(primitive(geometry), Math.min(speed, printer.maxFlow / bead), blend, span, printer.radiansPerMetre(bead));
        } else addMove(primitive(geometry), speed, blend, span);
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
          add(MotionOp.SetOutput(machine.channels.spindleDirection,
            directionValue), span);
          add(MotionOp.SetOutput(machine.channels.spindleSpeed,
            EventValue.Analog(rpm)), span);
          add(MotionOp.WaitInput(machine.channels.spindleAtSpeed,
            InputPredicate.Equals(EventValue.Digital(true)), null), span);
        } else {
          outputEvent(machine.channels.spindleDirection, directionValue, span);
          outputEvent(machine.channels.spindleSpeed, EventValue.Analog(rpm), span);
        }
        spindleOn = direction != Off;
      case Coolant(mist, flood, span):
        outputEvent(machine.channels.coolantMist,
          EventValue.Digital(mist), span);
        outputEvent(machine.channels.coolantFlood,
          EventValue.Digital(flood), span);
      case ToolChange(number, span):
        flush(); drainQueued(); add(MotionOp.WaitInput(machine.channels.toolChange(number),
          InputPredicate.Equals(EventValue.Digital(true)), null), span);
      case ToolLengthOffset(_, _, _):
        // ToolpathMotion already moved Z to the controlled point.
      case OptionalStop(span), ProgramStop(span):
        flush(); drainQueued(); add(MotionOp.WaitInput(machine.channels.operatorResume,
          InputPredicate.Equals(EventValue.Digital(true)), null), span);
      case End(_): flush(); drainQueued();
      case Layer(_, _, _), Feature(_, _), PrintObject(_, _, _):
        // Markers for preview, restart and cancel-object: they take no motion.
      case Deposit(area, _, _, span):
        if (machine.printer == null) throw 'toolpath line ${span.line}: printing operations need a printer binding';
        bead = area;
      case DepositOff(span):
        if (machine.printer == null) throw 'toolpath line ${span.line}: printing operations need a printer binding';
        bead = 0.0;
      case FilamentMove(length, speed, span):
        var printer = machine.printer;
        if (printer == null) throw 'toolpath line ${span.line}: printing operations need a printer binding';
        // An extruder-only move: the printer holds still, so the path before it ends and the next begins after.
        flush(); drainQueued();
        add(MotionOp.MoveJ(MoveTarget.JointOffset(printer.extruderIndex, printer.radians(length)),
          new MotionOptions(speed / printer.gearRadius), Blend.ExactStop), span);
      case Heat(heater, celsius, wait, span):
        if (machine.printer == null) throw 'toolpath line ${span.line}: printing operations need a printer binding';
        var value = EventValue.Analog(celsius);
        if (wait && celsius > 0.0) {
          // Later moves wait until it is there: a barrier, like a spindle coming up to speed.
          flush(); drainQueued();
          add(MotionOp.SetOutput(machine.channels.heaterTarget(heater), value), span);
          add(MotionOp.WaitInput(machine.channels.heaterAtTemperature(heater),
            InputPredicate.Equals(EventValue.Digital(true)), null), span);
        } else outputEvent(machine.channels.heaterTarget(heater), value, span);
      case Fan(fan, duty, span):
        if (machine.printer == null) throw 'toolpath line ${span.line}: printing operations need a printer binding';
        outputEvent(machine.channels.fan(fan), EventValue.Analog(duty), span);
    }
    flush(); drainQueued();
    var program = motionOps.length == 0 ? null : new MotionProgram(motionOps);
    return new ToolpathLoweringResult(program, sourceMap, diagnostics.copy(), stops);
  }

  /**
    Moves join one path until a barrier: the path keeps each move's speed,
    rounds the corners its tolerances allow, and stops at the others when it
    is planned, so a move that carries straight on, at another speed or
    exactly, does not stop.
  **/
  function addMove(geometry:PathPrimitive, speed:Float, blend:Float,
      span:Provenance, ?rate:Float = 0.0):Void {
    if (!Math.isFinite(blend) || blend < 0.0)
      throw "toolpath move needs a nonnegative finite tolerance";
    pending.push(geometry); pendingSpans.push(span);
    pendingTolerances.push(blend); pendingSpeeds.push(speed); pendingRates.push(rate);
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
    // Moves of different speeds join in one rounded path: the path keeps each move's speed, and the arc
    // that rounds a corner between them goes at the slower of the two.
    var corners = [for (i in 0...(pending.length - 1))
      Math.min(pendingTolerances[i], pendingTolerances[i + 1]) * CornerBlender.GEOMETRY_SHARE];
    var rounded = false;
    for (corner in corners) if (corner > 0.0) rounded = true;
    for (i in 0...corners.length) if (corners[i] == 0.0 && turns(pending[i], pending[i + 1]))
      count("no blend tolerance");
    if (rounded) {
      var authored = new GeometricPath(pending);
      // A corner is worth rounding only if the machine can take it at a good share of the slower move's speed.
      var minRadii = [for (i in 0...corners.length)
        CornerBlender.minimumRadius(Math.min(pendingSpeeds[i], pendingSpeeds[i + 1]), machine.cornerAcceleration)];
      var blended = CornerBlender.blendPerCorner(authored, corners,
        machine.maxBlendTurnAngleRadians, minRadii);
      var spans = [for (index in blended.sourcePrimitiveIndices)
        pendingSpans[index]];
      for (warning in 0...blended.diagnostics.length) {
        var index = blended.diagnosticCorners[warning];
        diagnostics.push(new ToolpathDiagnostic( "CNC_EXACT_STOP",
          pendingSpans[index], blended.diagnostics[warning]));
        var text = blended.diagnostics[warning], open = text.indexOf("exact stop (");
        // The angle in "turn angle 2.6 exceeds blend limit" is not part of the reason.
        count(~/ [0-9][0-9.eE+-]* /g.replace(open < 0 ? text : text.substring(open + 12, text.lastIndexOf(")")), " "));
      }
      var indices = blended.sourcePrimitiveIndices;
      // A blend arc is listed just before the move it leads into, under that move's index.
      var speeds = [for (k in 0...indices.length)
        k + 1 < indices.length && indices[k + 1] == indices[k] ?
          Math.min(pendingSpeeds[indices[k] - 1], pendingSpeeds[indices[k]]) : pendingSpeeds[indices[k]]];
      // Rounding a corner shortens the path, and a bead is laid per metre of the path the nozzle takes: the filament
      // the trimmed ends would have fed is fed by the arc that replaces them, so the corner feeds what was authored.
      var own = new Map<Int, Int>();
      for (k in 0...indices.length) if (!(k + 1 < indices.length && indices[k + 1] == indices[k])) own.set(indices[k], k);
      function trimmed(source:Int, atEnd:Bool):Float {
        var at = own.get(source);
        if (at == null) throw "blended corner lost a move";
        var kept = blended.path.primitives[at], original = pending[source];
        var from = original.pointAt(atEnd ? original.length() : 0.0);
        var to = kept.pointAt(atEnd ? kept.length() : 0.0);
        return from.distanceTo(to);
      }
      var rates = [for (k in 0...indices.length)
        k + 1 < indices.length && indices[k + 1] == indices[k] ?
          (pendingRates[indices[k] - 1] * trimmed(indices[k] - 1, true) + pendingRates[indices[k]] * trimmed(indices[k], false)) /
            Math.max(blended.path.primitives[k].length(), 1e-12) : pendingRates[indices[k]]];
      emitPath(blended.path.primitives, spans, speeds, authored, pendingBlend, indices, rates);
    } else
      emitPath(pending, pendingSpans, pendingSpeeds, null, 0.0, [for (index in 0...pending.length) index], pendingRates);
    pending = []; pendingSpans = []; pendingTolerances = []; pendingSpeeds = []; pendingRates = [];
    pendingEvents = [];
    pendingBlend = 0.0;
  }

  function count(reason:String):Void stops.set(reason, (stops.exists(reason) ? stops.get(reason) : 0) + 1);

  /** Whether the path changes direction where `before` ends and `after` begins. */
  static function turns(before:PathPrimitive, after:PathPrimitive):Bool {
    var incoming = before.tangentAt(before.length()), outgoing = after.tangentAt(0.0);
    var change = 0.0;
    for (axis in 0...3) change = Math.max(change, Math.abs(incoming[axis] - outgoing[axis]));
    return change > 1e-6;
  }

  function emitPath(geometry:Array<PathPrimitive>, spans:Array<Provenance>,
      speeds:Array<Float>, ?authored:GeometricPath, ?blend:Float = 0.0,
      ?sourceIndices:Array<Int>, ?rates:Array<Float>):Void {
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
    var printer = machine.printer;
    if (printer != null && rates != null) {
      var intervals:Array<DriveInterval> = [];
      var at = 0.0;
      for (i in 0...geometry.length) {
        var length = geometry[i].length();
        if (rates[i] > 0.0) intervals.push({start: at, end: at + length, rate: rates[i]});
        at += length;
      }
      if (intervals.length > 0) path.withDrive(new DistanceDrive(printer.extruderIndex, intervals, path.length(), printer.rampLength));
    }
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
