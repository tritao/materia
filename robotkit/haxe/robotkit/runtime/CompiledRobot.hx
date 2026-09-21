package robotkit.runtime;

import robotkit.model.Robot;

/** Validated semantic robot plus the revision used for runtime compilation. */
class CompiledRobot {
  public final source:Robot;
  public final jointCount:Int;
  public final revision:Int;

  public function new(source:Robot, revision:Int) {
    this.source = source;
    this.jointCount = source.joints.length;
    this.revision = revision;
  }
}
