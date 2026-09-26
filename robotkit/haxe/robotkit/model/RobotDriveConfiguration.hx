package robotkit.model;

/** Authored joint roles and geometry for a supported mobile drive layout. */
enum RobotDriveConfiguration {
  Differential(leftWheelJointId:JointId, rightWheelJointId:JointId, wheelRadius:Float, trackWidth:Float);
  Ackermann(
    steeringJointId:JointId,
    driveWheelJointId:JointId,
    wheelBase:Float,
    wheelRadius:Float,
    maxSteeringAngle:Float
  );
  /** Three wheel joints at 120-degree intervals; see robotkit.mobile.HolonomicDrive. */
  Holonomic(wheelJointIds:Array<JointId>, wheelRadius:Float, baseRadius:Float);
}
