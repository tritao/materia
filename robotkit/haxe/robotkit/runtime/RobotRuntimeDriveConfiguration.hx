package robotkit.runtime;

/** Resolved mobile drive roles. Joint numbers follow runtime command ordering. */
enum RobotRuntimeDriveConfiguration {
  Differential(
    leftWheelJoint:Int,
    leftWheelName:String,
    rightWheelJoint:Int,
    rightWheelName:String,
    wheelRadius:Float,
    trackWidth:Float
  );
  Ackermann(
    steeringJoint:Int,
    steeringName:String,
    driveWheelJoint:Int,
    driveWheelName:String,
    wheelBase:Float,
    wheelRadius:Float,
    maxSteeringAngle:Float
  );
}
