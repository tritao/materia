package cadbridge;

/** Digital process channel bound to an actuator inlet in a coupled configuration. */
enum EndEffectorControlBinding {
  Gripper(channel:String, instanceId:String, openPort:String, closePort:String);
  Vacuum(channel:String, instanceId:String, inletPort:String);
  Lock(channel:String, instanceId:String, inletPort:String);
}
