package app;

/** Stable application alarms raised by a failed robot tending mission. */
enum abstract MissionAlarmCode(Int) from Int to Int {
  var EmptyInfeed = 2001;
  var MissedGrasp = 2002;
  var PartDropped = 2003;
  var ClampWithoutPart = 2004;
  var DoorJam = 2005;
  var CncAlarm = 2006;
  var EmergencyStop = 2007;
  var RecoveryTimeout = 2008;
  var MotionFailure = 2009;
  var ViseFailure = 2010;
  var FasteningFailure = 2011;
}
