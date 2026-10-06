package cnckit.controller;

/** One sampled PLC input image. Times are seconds from the controller cycle. */
class RobotInterfaceInput {
  public final requestDoorOpen:Bool;
  public final requestDoorClose:Bool;
  public final requestClamp:Bool;
  public final requestUnclamp:Bool;
  public final cycleStart:Bool;
  public final clear:Bool;
  public final faultReset:Bool;
  public final doorOpen:Bool;
  public final doorClosed:Bool;
  public final clamped:Bool;
  public final unclamped:Bool;
  public final partPresent:Bool;
  public final spindleStopped:Bool;
  public final axesAtLoad:Bool;
  public final cycleComplete:Bool;
  public final toolChangeRequired:Bool;

  public function new(requestDoorOpen:Bool, requestDoorClose:Bool,
      requestClamp:Bool, requestUnclamp:Bool, cycleStart:Bool, clear:Bool,
      faultReset:Bool, doorOpen:Bool, doorClosed:Bool, clamped:Bool,
      unclamped:Bool, partPresent:Bool, spindleStopped:Bool, axesAtLoad:Bool,
      cycleComplete:Bool, toolChangeRequired:Bool) {
    this.requestDoorOpen = requestDoorOpen;
    this.requestDoorClose = requestDoorClose;
    this.requestClamp = requestClamp;
    this.requestUnclamp = requestUnclamp;
    this.cycleStart = cycleStart;
    this.clear = clear;
    this.faultReset = faultReset;
    this.doorOpen = doorOpen;
    this.doorClosed = doorClosed;
    this.clamped = clamped;
    this.unclamped = unclamped;
    this.partPresent = partPresent;
    this.spindleStopped = spindleStopped;
    this.axesAtLoad = axesAtLoad;
    this.cycleComplete = cycleComplete;
    this.toolChangeRequired = toolChangeRequired;
  }
}
