package tests;
import app.ProgramErrorMessage;
class ProgramErrorMessageTests {
  static function check(value:Bool, message:String):Void { if (!value) throw message; }
  public static function main():Void {
    var setup=ProgramErrorMessage.fromFailure("Workholding setup changed: joint opening");
    check(setup.title=="Check the workholding setup"&&setup.recovery.indexOf("blank")>=0,"vise setup failures explain how to recover");
    var failure = "Motion program op 4: path.timing.stream.step failed with MotionKit error -6";
    var timing = ProgramErrorMessage.fromFailure(failure);
    check(timing.title == "Could not plan the next move", "timing failure has an operator explanation");
    check(timing.recovery.indexOf("100%") >= 0, "timing failure suggests a concrete recovery");
    for (raw in [failure, "Clearance violation: tool / fixture", "unreachable pose", "Machine homing: timeout", "unexpected internal failure"]) {
      var result = ProgramErrorMessage.fromFailure(raw);
      check(result.title.length > 0 && result.recovery.length > 0, "every failure has an explanation and recovery");
      check(result.title.indexOf(raw) < 0 && result.recovery.indexOf(raw) < 0, "raw diagnostics stay out of primary messages");
      check(result.title.indexOf("-6") < 0 && result.title.indexOf("op 4") < 0, "internal codes stay out of primary messages");
    }
    check(ProgramErrorMessage.fromFailure("Clearance violation").recovery.indexOf("fixtures") >= 0, "clearance recovery identifies setup checks");
    Sys.println("Program error messages passed");
  }
}
