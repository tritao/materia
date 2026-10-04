package robotkit.runtime;

/** Preserves native status codes so callers can distinguish recoverable races. */
class RobotRuntimeError {
  public final status:Int;
  public final operation:String;

  public function new(status:Int, operation:String) {
    this.status = status;
    this.operation = operation;
  }

  public function toString():String
    return '$operation failed with RobotKit status $status';
}
