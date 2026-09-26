package robotkit.process;

import robotkit.world.JointTarget;

/** One executed trajectory sample: joint targets, joint solution, and tool state. */
class ToolpathExecutionStep {
  public final time:Float;
  public final targets:Array<JointTarget>;
  public final q:Array<Float>;
  public final processOn:Bool;

  public function new(time:Float, targets:Array<JointTarget>, q:Array<Float>, processOn:Bool) {
    this.time = time;
    this.targets = targets;
    this.q = q;
    this.processOn = processOn;
  }
}
