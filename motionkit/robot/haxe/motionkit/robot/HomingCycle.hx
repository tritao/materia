package motionkit.robot;

import motionkit.robot.HomingDriver.HomingObservation;
import haxe.Int64;

private enum HomingPhase {
  Idle; AwaitScope; Seek; StopAfterSeek; Backoff; StopAfterBackoff; Approach; AwaitSideHolds; StopAfterLatch; AwaitRelease; AwaitCalibration; AwaitScopeEnd; Return; Complete; Fault;
}

/** Sensor-driven homing with controlled stops between every change of direction. */
class HomingCycle {
  final driver:HomingDriver;
  final axes:Array<HomingAxis>;
  final sides:Null<robotkit.runtime.HomingSideControl>;
  var index:Int = 0;
  var phase:HomingPhase = Idle;
  var origin:Float = 0.0;
  var elapsed:Float = 0.0;
  var releasePosition:Null<Float> = null;
  var captures:Array<Null<Float>> = [];
  var approachEdges:Array<Null<Int>> = [];
  var sequences:Map<String, Int64> = new Map();
  var lastTimestamp:Null<Int64> = null;
  var observationClock:Null<String> = null;
  public var fault(default, null):Null<String> = null;

  public function new(driver:HomingDriver, axes:Array<HomingAxis>, ?sides:robotkit.runtime.HomingSideControl) {
    if (driver == null || axes == null || axes.length == 0) throw "Homing requires a driver and physical axes";
    this.driver = driver; this.axes = axes.copy();
    this.sides = sides;
    var ids = new Map<String, Bool>(), joints = new Map<Int, Bool>();
    for (axis in this.axes) {
      if (axis == null || ids.exists(axis.id) || joints.exists(axis.joint))
        throw "Homing requires distinct non-null axes and joints";
      ids.set(axis.id, true); joints.set(axis.joint, true);
      if (axis.switches.length > 1) {
        if (sides == null) throw "Multi-side homing requires independent motor holds";
        var drives = new Map<String, Bool>();
        for (contact in axis.switches) {
          if (contact.driveJoint == null || drives.exists(contact.driveJoint))
            throw "Multi-side homing requires distinct motor bindings";
          drives.set(contact.driveJoint, true);
        }
      }
    }
    // Retract the downward slide before any horizontal carriage movement.
    this.axes.sort((a, b) -> {
      var nameA = a.id.split("/").pop(), nameB = b.id.split("/").pop();
      var first = nameA == "z" ? 0 : nameA == "x" ? 1 : nameA == "y" ? 2 : 3;
      var second = nameB == "z" ? 0 : nameB == "x" ? 1 : nameB == "y" ? 2 : 3;
      return first == second ? Reflect.compare(a.id, b.id) : first - second;
    });
  }

  public function status():String return Std.string(phase);
  public function isComplete():Bool return phase == Complete;
  public function isActive():Bool return phase != Idle && phase != Complete && phase != Fault;

  public function start():Void {
    if (phase != Idle) throw "Homing cycle has already started";
    try beginAxis() catch (error:Dynamic) {
      fault = Std.string(error); phase = Fault;
      try driver.stop(axes[index].joint, axes[index].acceleration) catch (_:Dynamic) {}
      try releaseSides() catch (_:Dynamic) {}
      throw error;
    }
  }

  function beginAxis():Void {
    var axis = axes[index], observation = driver.observe(axis.joint);
    lastTimestamp = null; observationClock = null;
    validateFresh(axis, observation);
    if (Math.abs(observation.velocity) > axis.latchSpeed * 0.01)
      throw "Homing must start with the axis at rest";
    captures = [for (_ in axis.switches) null]; releasePosition = null;
    if (axis.switches.length > 1) sides.beginSquaring([for (contact in axis.switches) contact.id]);
    if (!controlsReady()) enter(AwaitScope, observation);
    else startSeeking(axis, observation);
  }

  public function update(dt:Float):Bool {
    if (!Math.isFinite(dt) || dt <= 0) throw "Homing update requires a finite positive interval";
    if (!isActive()) return false;
    var axis = axes[index];
    try {
      var observation = driver.observe(axis.joint);
      // Stop/hold/counter acknowledgements can precede their confirming state.
      // Progress the controls, but do not consume cached captures during that barrier.
      elapsed += dt;
      var controlsConfirmed = controlsReady();
      var timeout = axis.maximumTravel / axis.latchSpeed + 10 * axis.seekSpeed / axis.acceleration;
      if (elapsed > timeout || Math.abs(observation.position - origin) > axis.maximumTravel)
        throw 'Homing axis "${axis.id}" did not reach its switch within physical travel';
      if (!controlsConfirmed && !observationAdvanced(axis, observation)) return true;
      validateFresh(axis, observation);
      var stopped = Math.abs(observation.velocity) <= axis.latchSpeed * 0.01;
      switch phase {
        case AwaitScope:
          if (controlsReady()) startSeeking(axis, observation);
        case AwaitSideHolds:
          if (controlsReady()) enter(StopAfterLatch, observation);
        case AwaitRelease:
          if (controlsReady()) latchAndCalibrate(axis);
        case AwaitCalibration:
          if (controlsReady()) finishCalibration(axis);
        case AwaitScopeEnd:
          if (controlsReady()) enter(Return, driver.observe(axis.joint));
        case Seek:
          if (anyActive(axis, observation)) enter(StopAfterSeek, observation);
        case StopAfterSeek:
          if (stopped && observation.calibrationReady && controlsReady()) enter(Backoff, observation);
        case Backoff:
          if (anyActive(axis, observation)) releasePosition = null;
          else {
            if (releasePosition == null) releasePosition = observation.position;
            if (Math.abs(observation.position - releasePosition) >= axis.releaseDistance)
              enter(StopAfterBackoff, observation);
          }
        case StopAfterBackoff:
          if (stopped && observation.calibrationReady && controlsReady()) {
            captures = [for (_ in axis.switches) null]; enter(Approach, observation);
          }
        case Approach:
          // Progress the first side's asynchronous hold while the second side
          // is still approaching its switch.
          controlsReady();
          var all = true;
          for (i in 0...axis.switches.length) {
            var signal = signalFor(axis.switches[i].id, observation);
            var baseline:Int = -1, count:Int = -1;
            if (approachEdges[i] != null) baseline = cast approachEdges[i];
            if (signal.closingEdges != null) count = cast signal.closingEdges;
            if (count >= 0 && baseline >= 0 && count < baseline)
              throw "Homing edge counter reset during approach";
            if (signal.active && captures[i] == null) {
              if (signal.edgePosition != null && (baseline < 0 || count <= baseline))
                throw "Homing closing-edge capture predates the slow approach";
              if (signal.edgePosition == null && axis.switches[i].repeatability == 0)
                throw "Zero-repeatability homing requires captured switch edges";
              if (signal.edgePosition == null && dt > axis.timestep * (1 + 1e-9))
                throw "Homing sampling interval exceeds its repeatability budget";
              captures[i] = signal.edgePosition == null ? observation.position : signal.edgePosition;
              if (axis.switches.length > 1) sides.hold(axis.switches[i].id);
            }
            if (captures[i] == null) all = false;
          }
          if (all) enter(controlsReady() ? StopAfterLatch : AwaitSideHolds, observation);
        case StopAfterLatch:
          if (stopped && observation.calibrationReady && controlsReady()) {
            if (sides != null) sides.releaseAll();
            if (!controlsReady()) enter(AwaitRelease, observation);
            else latchAndCalibrate(axis);
          }
        case Return:
          if (stopped && observation.calibrationReady && Math.abs(observation.position - axis.home) <= axis.positionTolerance) {
            index++;
            if (index == axes.length) phase = Complete;
            else beginAxis();
          }
        case _:
      }
      return isActive();
    } catch (error:Dynamic) {
      fault = Std.string(error); phase = Fault;
      try driver.stop(axis.joint, axis.acceleration) catch (_:Dynamic) {}
      try releaseSides() catch (_:Dynamic) {}
      throw error;
    }
  }

  public function cancel():Void {
    if (!isActive()) return;
    fault = "Homing cancelled"; phase = Fault;
    var failure:Null<String> = null;
    try driver.stop(axes[index].joint, axes[index].acceleration) catch (error:Dynamic) failure = Std.string(error);
    try releaseSides() catch (error:Dynamic) { if (failure == null) failure = Std.string(error); }
    if (failure != null) throw failure;
  }

  function controlsReady():Bool {
    if (sides == null || !Std.isOfType(sides, robotkit.runtime.HomingControlReadiness)) return true;
    var asynchronous:robotkit.runtime.HomingControlReadiness = cast sides;
    return asynchronous.controlsReady();
  }

  function startSeeking(axis:HomingAxis, observation:HomingObservation):Void {
    enter(anyActive(axis, observation) ? Backoff : Seek, observation);
  }

  function latchAndCalibrate(axis:HomingAxis):Void {
    for (i in 0...axis.switches.length) {
      var capture = captures[i];
      if (capture == null) throw "Homing has no captured latch position";
      var leaderCapture:Null<Float> = axis.switches.length > 1 ? sides.leaderCapture(axis.switches[i].id, capture) : null;
      driver.latch(axis.switches[i].id, capture, leaderCapture);
    }
    if (axis.switches.length > 1) sides.calibrate([for (contact in axis.switches) contact.id]);
    if (!controlsReady()) enter(AwaitCalibration, driver.observe(axis.joint));
    else finishCalibration(axis);
  }

  function finishCalibration(axis:HomingAxis):Void {
    if (axis.switches.length > 1) sides.endSquaring();
    var observation = driver.observe(axis.joint);
    if (!controlsReady()) enter(AwaitScopeEnd, observation);
    else enter(Return, observation);
  }

  function releaseSides():Void {
    if (sides == null) return;
    if (Std.isOfType(sides, robotkit.runtime.HomingCancellation)) {
      var cancellation:robotkit.runtime.HomingCancellation = cast sides;
      cancellation.cancelSquaring();
    } else sides.endSquaring();
  }

  function enter(next:HomingPhase, observation:HomingObservation):Void {
    phase = next; origin = observation.position; elapsed = 0.0;
    var axis = axes[index], side = axis.switches[0].side;
    switch next {
      case Seek: driver.velocity(axis.joint, side * axis.seekSpeed, axis.acceleration);
      case Backoff: driver.velocity(axis.joint, -side * axis.seekSpeed, axis.acceleration);
      case Approach:
        approachEdges = [for (contact in axis.switches) signalFor(contact.id, observation).closingEdges];
        driver.velocity(axis.joint, side * axis.latchSpeed, axis.acceleration);
      case StopAfterSeek, StopAfterBackoff, StopAfterLatch: driver.stop(axis.joint, axis.acceleration);
      case Return: driver.returnHome(axis.joint, axis.home, axis.seekSpeed, axis.acceleration);
      case _:
    }
  }

  function observationAdvanced(axis:HomingAxis, observation:HomingObservation):Bool {
    if (observationClock != null && observation.clockId != observationClock) return true;
    for (contact in axis.switches) {
      var signal = signalFor(contact.id, observation);
      if (signal.clockId != observation.clockId || Int64.compare(signal.timestampNs, observation.timestampNs) > 0) return true;
    }
    if (lastTimestamp != null && Int64.compare(observation.timestampNs, lastTimestamp) <= 0) return false;
    for (contact in axis.switches) {
      var signal = signalFor(contact.id, observation), previous = sequences.get(signal.id);
      if (Int64.toFloat(Int64.sub(observation.timestampNs, signal.timestampNs)) > axis.timestep * 1e9 * (1 + 1e-9) ||
          (previous != null && Int64.compare(signal.sequence, previous) <= 0)) return false;
    }
    return true;
  }

  function validateFresh(axis:HomingAxis, observation:HomingObservation):Void {
    if (observation == null) throw "Homing has no position observation";
    if (observationClock != null && observation.clockId != observationClock)
      throw "Homing observation source clock changed";
    if (lastTimestamp != null && Int64.compare(observation.timestampNs, lastTimestamp) <= 0)
      throw "Homing position observation did not advance";
    for (contact in axis.switches) {
      var signal = signalFor(contact.id, observation);
      var previous = sequences.get(signal.id);
      if (signal.clockId != observation.clockId ||
          Int64.compare(signal.timestampNs, observation.timestampNs) > 0 ||
          Int64.toFloat(Int64.sub(observation.timestampNs, signal.timestampNs)) > axis.timestep * 1e9 * (1 + 1e-9) ||
          (previous != null && Int64.compare(signal.sequence, previous) <= 0))
        throw 'Homing switch "${signal.id}" has a stale or incompatible source observation';
    }
    for (contact in axis.switches) {
      var signal = signalFor(contact.id, observation);
      sequences.set(signal.id, signal.sequence);
    }
    lastTimestamp = observation.timestampNs;
    observationClock = observation.clockId;
  }

  function anyActive(axis:HomingAxis, observation:HomingObservation):Bool {
    var active = false;
    for (contact in axis.switches) if (signalFor(contact.id, observation).active) active = true;
    return active;
  }

  function signalFor(id:String, observation:HomingObservation):motionkit.robot.HomingDriver.HomingSwitchObservation {
    for (signal in observation.switches) if (signal != null && signal.id == id) return signal;
    throw 'Homing switch "$id" has no fresh observation';
  }
}
