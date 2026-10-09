package cnckit.controller;

/** Stable machine-cell signal names shared by the CNC and robot controllers. */
class RobotInterfaceChannels {
  public static inline final REQUEST_DOOR_OPEN = "robot.request_door_open";
  public static inline final REQUEST_DOOR_CLOSE = "robot.request_door_close";
  public static inline final REQUEST_CLAMP = "robot.request_clamp";
  public static inline final REQUEST_UNCLAMP = "robot.request_unclamp";
  public static inline final CYCLE_START = "robot.cycle_start";
  public static inline final ROBOT_CLEAR = "robot.clear";
  public static inline final FAULT_RESET = "robot.fault_reset";
  public static inline final CNC_READY = "cnc.ready";
  public static inline final DOOR_OPEN = "cnc.door_open";
  public static inline final DOOR_CLOSED = "cnc.door_closed";
  public static inline final CLAMPED = "cnc.clamped";
  public static inline final UNCLAMPED = "cnc.unclamped";
  public static inline final PART_PRESENT = "cnc.part_present";
  public static inline final IN_CYCLE = "cnc.in_cycle";
  public static inline final CYCLE_COMPLETE = "cnc.cycle_complete";
  public static inline final ALARM = "cnc.alarm";

  /** Four discrete outputs carry the current RobotInterfaceAlarm value to the cell PLC. */
  public static function alarmCodeBits():Array<String> return [
    "cnc.alarm_code.bit0", "cnc.alarm_code.bit1", "cnc.alarm_code.bit2", "cnc.alarm_code.bit3"];

  public static function alarmCodeChannel(scope:String):String
    return scope == null || scope.length == 0 ? "cnc.alarm_code" : scope + "/cnc.alarm_code";

  public static function robotRequests():Array<String> return [REQUEST_DOOR_OPEN,
    REQUEST_DOOR_CLOSE, REQUEST_CLAMP, REQUEST_UNCLAMP, CYCLE_START,
    ROBOT_CLEAR, FAULT_RESET];
  public static function cncOutputs():Array<String> return [CNC_READY, DOOR_OPEN,
    DOOR_CLOSED, CLAMPED, UNCLAMPED, PART_PRESENT, IN_CYCLE,
    CYCLE_COMPLETE, ALARM].concat(alarmCodeBits());
}
