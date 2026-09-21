package robotkit.runtime;

/** One actionable semantic error found while compiling a RobotModel. */
class RobotCompileDiagnostic {
  public final code:String;
  public final path:String;
  public final message:String;

  public function new(code:String, path:String, message:String) {
    this.code = code;
    this.path = path;
    this.message = message;
  }

  public function toString():String
    return '$code at $path: $message';
}
