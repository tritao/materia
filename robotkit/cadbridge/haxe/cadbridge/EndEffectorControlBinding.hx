package cadbridge;

/** Digital process channel bound to an actuator inlet in a coupled configuration. */
enum EndEffectorControlBinding {
  Gripper(channel:String, instanceId:String, openPort:String, closePort:String);
  Vacuum(channel:String, instanceId:String, inletPort:String);
  Lock(channel:String, instanceId:String, inletPort:String);
  /** An arc torch's trigger. The arc is worked by the robot's welder (RobotKit's `SimulatedWelder`), not by a tool runtime. */
  Arc(channel:String, instanceId:String, controlPort:String);
}
