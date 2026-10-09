import cnckit.controller.RobotInterface;
import cnckit.controller.RobotInterfaceAlarm;
import cnckit.controller.RobotInterfaceInput;
import cnckit.controller.RobotInterfaceOutputs;

class RobotInterfaceTests {
  static function input(requestDoorOpen:Bool = false, requestDoorClose:Bool = false,
      requestClamp:Bool = false, requestUnclamp:Bool = false,
      cycleStart:Bool = false, faultReset:Bool = false,
      doorOpen:Bool = false, doorClosed:Bool = true, clamped:Bool = true,
      unclamped:Bool = false, partPresent:Bool = true,
      spindleStopped:Bool = true, axesAtLoad:Bool = true,
      clear:Bool = true, cycleComplete:Bool = false,
      toolChangeRequired:Bool = false):RobotInterfaceInput
    return new RobotInterfaceInput(requestDoorOpen, requestDoorClose,
      requestClamp, requestUnclamp, cycleStart, clear, faultReset,
      doorOpen, doorClosed, clamped, unclamped, partPresent,
      spindleStopped, axesAtLoad, cycleComplete, toolChangeRequired);

  static function check(value:Bool, message:String):Void
    if (!value) throw message;

  static function timesOut(request:RobotInterfaceInput, expected:RobotInterfaceAlarm,
      requestedCoil:RobotInterfaceOutputs->Bool, message:String):Void {
    var fsm = new RobotInterface(0.03);
    var output = fsm.update(request, 0.01);
    check(output.alarm == RobotInterfaceAlarm.None && requestedCoil(output),
      '$message starts the physical request');
    for (_ in 0...3) output = fsm.update(request, 0.01);
    check(output.alarm == expected && !output.doorCoilA && !output.doorCoilB &&
      !output.viseCoilA && !output.viseCoilB,
      '$message times out and de-energizes the valve');
  }

  public static function run():Void {
    timesOut(input(true), RobotInterfaceAlarm.DoorOpenTimeout,
      output -> output.doorCoilA, "door open");
    timesOut(input(false, true, false, false, false, false,
      true, false, true, false, true), RobotInterfaceAlarm.DoorCloseTimeout,
      output -> output.doorCoilB, "door close");
    timesOut(input(false, false, true, false, false, false,
      true, false, false, false, true), RobotInterfaceAlarm.ClampTimeout,
      output -> output.viseCoilA, "vise clamp");
    timesOut(input(false, false, false, true, false, false,
      true, false, true, false, true), RobotInterfaceAlarm.UnclampTimeout,
      output -> output.viseCoilB, "vise unclamp");

    var fsm = new RobotInterface();
    var output = fsm.update(input(true), 0.01);

    fsm = new RobotInterface();
    output = fsm.update(input(true, false, false, false, false, false,
      false, true, true, false, true, false), 0.01);
    check(output.alarm == RobotInterfaceAlarm.DoorOpenInterlock && !output.doorCoilA,
      "door opening is refused while the spindle is running");

    fsm = new RobotInterface();
    output = fsm.update(input(true, false, false, false, false, false,
      false, false, true, false, true, true, true), 0.01);
    check(output.alarm == RobotInterfaceAlarm.None && output.doorCoilA,
      "a safe open request continues a door from a partially open position");

    fsm = new RobotInterface();
    output = fsm.update(input(false, false, true, false, false, false,
      true, false, false, false, false), 0.01);
    check(output.alarm == RobotInterfaceAlarm.ClampInterlock,
      "clamping is refused when no blank is present");

    fsm = new RobotInterface();
    output = fsm.update(input(false, false, false, false, true, false,
      false, false, true, false, true), 0.01);
    check(output.alarm == RobotInterfaceAlarm.CycleInterlock && !output.startCycle,
      "a cycle is refused unless all door, vise, presence and clearance interlocks are true");

    fsm = new RobotInterface();
    output = fsm.update(input(false, true, false, false, false, false,
      false, false, false, false, true, true, true, false), 0.01);
    check(output.alarm == RobotInterfaceAlarm.DoorCloseInterlock,
      "closing is refused unless the robot has cleared the cell");

    fsm = new RobotInterface();
    output = fsm.update(input(false, false, false, false, true), 0.01);
    check(output.startCycle && output.inCycle, "a valid cycle request starts one CNC pass");
    output = fsm.update(input(false, false, false, true), 0.01);
    check(output.alarm == RobotInterfaceAlarm.UnclampInterlock && !output.viseCoilB,
      "unclamping is refused during a CNC cycle");

    fsm = new RobotInterface();
    fsm.update(input(false, false, false, false, true), 0.01);
    output = fsm.update(input(false, false, false, false, false, false,
      false, false, true, true, true, true, true), 0.01);
    check(output.alarm == RobotInterfaceAlarm.DoorOpenedDuringCycle && output.holdCycle,
      "losing the closed-door switch during a cycle requests a hold and alarms");

    fsm = new RobotInterface();
    output = fsm.update(input(false, false, false, false, true), 0.01);
    output = fsm.update(input(false, false, false, false, false, false,
      false, true, true, false, true, true, true, true, false, true), 0.01);
    check(output.alarm == RobotInterfaceAlarm.ToolChangeRequired && output.holdCycle && output.inCycle,
      "a tool-change request during a cut holds the cycle and raises the CNC alarm");
    output = fsm.update(input(false, false, false, false, false, true,
      false, true, true, false, true, true, true, true), 0.01);
    check(output.alarm == RobotInterfaceAlarm.None && output.ready,
      "after the tool-change cause is removed, a safe fault reset clears the CNC alarm");
  }
}
