package cnckit.controller;

/** Stable alarm codes published by a machine's robot interface. */
enum abstract RobotInterfaceAlarm(Int) from Int to Int {
  var None = 0;
  var ConflictingRequests = 1;
  var DoorOpenInterlock = 2;
  var DoorCloseInterlock = 3;
  var ClampInterlock = 4;
  var UnclampInterlock = 5;
  var CycleInterlock = 6;
  var DoorOpenTimeout = 7;
  var DoorCloseTimeout = 8;
  var ClampTimeout = 9;
  var UnclampTimeout = 10;
  var DoorOpenedDuringCycle = 11;
  var ToolChangeRequired = 12;
}
