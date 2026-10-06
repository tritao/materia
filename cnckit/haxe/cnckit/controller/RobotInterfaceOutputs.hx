package cnckit.controller;

/** One complete output image after a RobotInterface update. */
class RobotInterfaceOutputs {
  public final ready:Bool;
  public final doorOpen:Bool;
  public final doorClosed:Bool;
  public final clamped:Bool;
  public final unclamped:Bool;
  public final partPresent:Bool;
  public final inCycle:Bool;
  public final cycleComplete:Bool;
  public final alarm:RobotInterfaceAlarm;
  public final doorCoilA:Bool;
  public final doorCoilB:Bool;
  public final viseCoilA:Bool;
  public final viseCoilB:Bool;
  public final startCycle:Bool;
  public final holdCycle:Bool;

  public function new(ready:Bool, doorOpen:Bool, doorClosed:Bool,
      clamped:Bool, unclamped:Bool, partPresent:Bool, inCycle:Bool,
      cycleComplete:Bool, alarm:RobotInterfaceAlarm, doorCoilA:Bool,
      doorCoilB:Bool, viseCoilA:Bool, viseCoilB:Bool, startCycle:Bool,
      holdCycle:Bool) {
    this.ready = ready;
    this.doorOpen = doorOpen;
    this.doorClosed = doorClosed;
    this.clamped = clamped;
    this.unclamped = unclamped;
    this.partPresent = partPresent;
    this.inCycle = inCycle;
    this.cycleComplete = cycleComplete;
    this.alarm = alarm;
    this.doorCoilA = doorCoilA;
    this.doorCoilB = doorCoilB;
    this.viseCoilA = viseCoilA;
    this.viseCoilB = viseCoilB;
    this.startCycle = startCycle;
    this.holdCycle = holdCycle;
  }
}
