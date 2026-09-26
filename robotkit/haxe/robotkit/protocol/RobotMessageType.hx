package robotkit.protocol;

/** Stable message identifiers carried by RobotFrame, outside the payload. */
enum abstract RobotMessageType(Int) from Int to Int {
  var Hello = 1;
  var Welcome = 2;
  var RobotDescription = 3;
  var RobotCapabilities = 4;
  var RobotState = 5;
  var JointTarget = 6;
  var FrameTarget = 7;
  var BaseTwist = 8;
  var TrajectoryRequest = 9;
  var ControllerStatus = 10;
  var Fault = 11;
  var Stop = 12;
  var SensorFrame = 13;
  var JointTargets = 14;
  var SafetyReset = 15;
  var ControlHeartbeat = 16;
  var CameraFrame = 17;
}
