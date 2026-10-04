package robotkit.core;



/** Transport-independent lifecycle state of a logical robot. */
enum RobotStatus {
  Disconnected;
  Connecting;
  Ready;
  Fault;
}
