package robotkit.process;

/** Outcome of `ToolpathExecutor.execute`: the steps produced so far, and why it stopped, if it did. */
class ToolpathExecutionResult {
  public final success:Bool;
  public final steps:Array<ToolpathExecutionStep>;
  public final failure:Null<ToolpathExecutionFailure>;

  public function new(success:Bool, steps:Array<ToolpathExecutionStep>, ?failure:ToolpathExecutionFailure) {
    this.success = success;
    this.steps = steps;
    this.failure = failure;
  }
}
