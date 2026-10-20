package app;

/** Operator-facing explanation; the original diagnostic remains available separately. */
class ProgramErrorMessage {
  public final title:String;
  public final recovery:String;

  function new(title:String, recovery:String) {
    this.title = title;
    this.recovery = recovery;
  }

  public static function fromFailure(reason:String):ProgramErrorMessage {
    var text = reason.toLowerCase();
    if (text.indexOf("workholding setup") >= 0)
      return new ProgramErrorMessage("Check the workholding setup",
        "Seat the blank, close the vise to the blank width, then reset the program.");
    if (text.indexOf("path.timing") >= 0 || text.indexOf("timing requires an adjustment") >= 0)
      return new ProgramErrorMessage("Could not plan the next move",
        "Set speed to 100%, reset the program, then press Play. If it stops again, open technical details.");
    if (text.indexOf("collision") >= 0 || text.indexOf("clearance") >= 0 || text.indexOf("contact") >= 0)
      return new ProgramErrorMessage("Motion stopped for a clearance problem",
        "Check the tool, holder, stock, and fixtures near the stopped tool before resetting the program.");
    if (text.indexOf("unreachable") >= 0 || text.indexOf("joint position") >= 0)
      return new ProgramErrorMessage("The machine cannot reach this move",
        "Check the work offset and machine travel, then reset the program.");
    if (text.indexOf("homing") >= 0)
      return new ProgramErrorMessage("Machine homing could not finish",
        "Check the axis setup and homing sensors, then reset the program.");
    return new ProgramErrorMessage("Program stopped",
      "Review the program and technical details, then reset the program to try again.");
  }
}
