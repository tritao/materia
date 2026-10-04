package robotkit.runtime;

/** Resolved mobile drive roles. Joint numbers follow runtime command ordering. */
enum RobotRuntimeDriveConfiguration {
  Differential(
    leftWheelJoint:Int,
    leftWheelName:String,
    rightWheelJoint:Int,
    rightWheelName:String,
    wheelRadius:Float,
    trackWidth:Float,
    /** +1 when a positive rate on that wheel's joint rolls the base forward, -1 when backward. */
    leftDirection:Int,
    rightDirection:Int
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
  Holonomic(
    wheelJoints:Array<Int>,
    wheelNames:Array<String>,
    wheelRadius:Float,
    baseRadius:Float
  );
}
