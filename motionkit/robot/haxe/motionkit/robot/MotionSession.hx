package motionkit.robot;

import robotkit.core.RobotSnapshot;

/**
 * Motion lifecycle. The runtime owns physical rest; only update() may complete
 * a hold or stop after a snapshot confirms rest.
 *
 * State       Event                   Result
 * Idle        begin/hold              Running/Held
 * Running     hold                    Holding
 * Holding     rest                    Held
 * Holding     resume                  Holding (resume pending)
 * Held        resume                  Running
 * Running     replace/abort          Stopping(Replan/Discard)
 * Stopping    hold/resume             Stopping (no runtime command)
 * Stopping    rest                    Idle, then pending requests may begin
 * Any         runtime fault/rejection Faulted
 * Faulted     reset                   Idle, after runtime safety reset
 * The same transitions govern axis motion and manipulator programs. A program
 * waiting for a controlled stop is owned by its caller, not by this session.
 */
class MotionSession {
  public var state(default, null):SessionState = Idle;
  public var pending(default, null):Array<MotionRequest> = [];
  public var jogAxis(default, null):Null<String> = null;
  public var resumePending(default, null):Bool = false;
  public var faultCode(default, null):Int = 0;

  public function new() {}

  public function isFaulted():Bool return state == Faulted;
  public function isActive():Bool return state != Idle && state != Faulted;
  public function isHolding():Bool return state == Holding || state == Held;
  public function isStopping():Bool return switch state {
    case Stopping(_): true;
    case _: false;
  }
  public function canStream():Bool return state == Running || state == Idle;
  public function hasPending():Bool return pending.length > 0;

  public function begin(?axis:Null<String>):Void {
    requireReady();
    state = Running;
    jogAxis = axis;
  }

  public function hold():Bool {
    requireReady();
    if (state == Idle) {
      state = Held;
      return false;
    }
    if (state != Running) return false;
    state = Holding;
    resumePending = false;
    return true;
  }

  public function resume():Bool {
    requireReady();
    return switch state {
      case Holding: resumePending = true; false;
      case Held: state = Running; resumePending = false; true;
      case _: false;
    }
  }

  public function stop(then:StopDisposition, requests:Array<MotionRequest>):Void {
    requireReady();
    state = Stopping(then);
    pending = requests;
    jogAxis = null;
    resumePending = false;
  }

  public function append(request:MotionRequest):Void {
    requireReady();
    pending.push(request);
  }

  public function observe(snapshot:RobotSnapshot):Void {
    if (snapshot.faultCode != 0) {
      faultCode = snapshot.faultCode;
      state = Faulted;
      pending = [];
    }
  }

  public function reject():Void {
    state = Faulted;
    pending = [];
  }

  /** Called only by MotionSystem.update after runtime rest is observed. */
  public function rest():Array<MotionRequest> {
    if (isFaulted()) return [];
    switch state {
      case Holding:
        state = Held;
        return [];
      case Stopping(then):
        state = Idle;
        var result = then == Replan ? pending : [];
        pending = [];
        return result;
      case _: return [];
    }
  }

  public function completed():Void {
    if (state == Running) state = Idle;
    jogAxis = null;
  }

  public function clear():Void {
    requireReady();
    state = Idle;
    pending = [];
    jogAxis = null;
    resumePending = false;
  }

  public function reset():Void {
    state = Idle;
    pending = [];
    jogAxis = null;
    resumePending = false;
    faultCode = 0;
  }

  public function requireReady():Void
    if (isFaulted()) throw "Motion session is faulted; call reset() before new motion";
}
