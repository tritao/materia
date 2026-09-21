package robotkit.runtime;

/**
 * Thrown when a RobotModel cannot be lowered to a runtime blueprint.
 *
 * The complete diagnostic list is retained so callers can show one useful
 * report instead of fixing invalid fields one exception at a time.
 */
class RobotCompileException extends haxe.Exception {
  public final diagnostics:Array<RobotCompileDiagnostic>;

  public function new(robotName:String, diagnostics:Array<RobotCompileDiagnostic>) {
    this.diagnostics = diagnostics.copy();
    var details = [for (diagnostic in diagnostics) diagnostic.toString()];
    super('Cannot compile robot "$robotName":\n' + details.join("\n"));
  }
}
