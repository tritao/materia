package cnckit.controller;

/** Pure, one-cycle PLC state machine for a robot-tended CNC enclosure and vise. */
class RobotInterface {
  static inline final IDLE = 0;
  static inline final OPENING_DOOR = 1;
  static inline final CLOSING_DOOR = 2;
  static inline final CLAMPING = 3;
  static inline final UNCLAMPING = 4;

  public final requestTimeoutSeconds:Float;
  var state:Int = IDLE;
  var elapsed:Float = 0.0;
  var activeCycle:Bool = false;
  var complete:Bool = false;
  var alarmCode:RobotInterfaceAlarm = RobotInterfaceAlarm.None;
  var previous:Null<RobotInterfaceInput> = null;
  var holdRequested:Bool = false;

  public function new(requestTimeoutSeconds:Float = 5.0) {
    if (!(requestTimeoutSeconds > 0.0) || !Math.isFinite(requestTimeoutSeconds))
      throw "Robot interface timeout must be finite and positive";
    this.requestTimeoutSeconds = requestTimeoutSeconds;
  }

  /** Consume one 10 ms controller image and return the complete output image. */
  public function update(input:RobotInterfaceInput, dt:Float):RobotInterfaceOutputs {
    if (input == null || !(dt > 0.0) || !Math.isFinite(dt))
      throw "Robot interface needs finite positive controller time";
    var old = previous;
    var openEdge = edge(input.requestDoorOpen, old == null ? false : old.requestDoorOpen);
    var closeEdge = edge(input.requestDoorClose, old == null ? false : old.requestDoorClose);
    var clampEdge = edge(input.requestClamp, old == null ? false : old.requestClamp);
    var unclampEdge = edge(input.requestUnclamp, old == null ? false : old.requestUnclamp);
    var startEdge = edge(input.cycleStart, old == null ? false : old.cycleStart);
    var resetEdge = edge(input.faultReset, old == null ? false : old.faultReset);
    var wasActive = activeCycle;
    previous = input;
    holdRequested = false;

    if (alarmCode == RobotInterfaceAlarm.None && activeCycle && !input.doorClosed) fail(DoorOpenedDuringCycle, true);
    else if (alarmCode == RobotInterfaceAlarm.None && activeCycle && input.toolChangeRequired) fail(ToolChangeRequired, true);
    else if (alarmCode == RobotInterfaceAlarm.None && activeCycle && unclampEdge) fail(UnclampInterlock, false);

    if (alarmCode != RobotInterfaceAlarm.None && resetEdge && safeToReset(input)) {
      alarmCode = RobotInterfaceAlarm.None;
      holdRequested = false;
      state = IDLE;
      elapsed = 0.0;
      activeCycle = false;
      complete = false;
    }

    var requestCount = (openEdge ? 1 : 0) + (closeEdge ? 1 : 0) +
      (clampEdge ? 1 : 0) + (unclampEdge ? 1 : 0) + (startEdge ? 1 : 0);
    if (alarmCode == RobotInterfaceAlarm.None && requestCount > 1) fail(ConflictingRequests, false);

    if (alarmCode == RobotInterfaceAlarm.None && state != IDLE) {
      elapsed += dt;
      if (requestReached(input)) {
        state = IDLE;
        elapsed = 0.0;
      } else if (elapsed >= requestTimeoutSeconds) {
        fail(timeoutCode(), false);
      }
    }

    if (alarmCode == RobotInterfaceAlarm.None && state == IDLE && !activeCycle) {
      if (openEdge) {
        if (input.doorOpen) {}
        else if (!input.spindleStopped || !input.axesAtLoad || !input.doorClosed)
          fail(DoorOpenInterlock, false);
        else begin(OPENING_DOOR);
      } else if (closeEdge) {
        if (input.doorClosed) {}
        else if (!input.clear || !input.spindleStopped || !input.doorOpen)
          fail(DoorCloseInterlock, false);
        else begin(CLOSING_DOOR);
      } else if (clampEdge) {
        if (input.clamped && input.partPresent) {}
        else if (!input.doorOpen || !input.partPresent)
          fail(ClampInterlock, false);
        else begin(CLAMPING);
      } else if (unclampEdge) {
        if (input.unclamped) {}
        else if (!input.doorOpen || !input.spindleStopped || !input.axesAtLoad)
          fail(UnclampInterlock, false);
        else begin(UNCLAMPING);
      } else if (startEdge) {
        if (!input.doorClosed || !input.clamped || !input.partPresent ||
            !input.clear || !input.spindleStopped || !input.axesAtLoad)
          fail(CycleInterlock, false);
        else {
          activeCycle = true;
          complete = false;
        }
      }
    }

    var startCycle = activeCycle && !wasActive && alarmCode == RobotInterfaceAlarm.None;
    if (activeCycle && input.cycleComplete) {
      activeCycle = false;
      complete = true;
    }

    return new RobotInterfaceOutputs(alarmCode == RobotInterfaceAlarm.None && state == IDLE &&
		!activeCycle && input.spindleStopped && input.axesAtLoad,
      input.doorOpen, input.doorClosed, input.clamped, input.unclamped,
      input.partPresent, activeCycle, complete, alarmCode,
      alarmCode == RobotInterfaceAlarm.None && state == OPENING_DOOR, alarmCode == RobotInterfaceAlarm.None && state == CLOSING_DOOR,
      alarmCode == RobotInterfaceAlarm.None && state == CLAMPING, alarmCode == RobotInterfaceAlarm.None && state == UNCLAMPING,
      startCycle, holdRequested);
  }

  function begin(next:Int):Void { state = next; elapsed = 0.0; }

  function requestReached(input:RobotInterfaceInput):Bool return switch state {
    case OPENING_DOOR: input.doorOpen;
    case CLOSING_DOOR: input.doorClosed;
    case CLAMPING: input.clamped && input.partPresent;
    case UNCLAMPING: input.unclamped;
    default: false;
  };

  function timeoutCode():RobotInterfaceAlarm return switch state {
    case OPENING_DOOR: DoorOpenTimeout;
    case CLOSING_DOOR: DoorCloseTimeout;
    case CLAMPING: ClampTimeout;
    case UNCLAMPING: UnclampTimeout;
    default: RobotInterfaceAlarm.None;
  };

  function fail(code:RobotInterfaceAlarm, hold:Bool):Void {
    alarmCode = code;
    state = IDLE;
    elapsed = 0.0;
    complete = false;
    holdRequested = hold;
  }

  function safeToReset(input:RobotInterfaceInput):Bool
    return !input.toolChangeRequired && input.spindleStopped && input.axesAtLoad &&
      (alarmCode != RobotInterfaceAlarm.DoorOpenedDuringCycle || input.doorClosed);

  static function edge(current:Bool, prior:Bool):Bool return current && !prior;
}
