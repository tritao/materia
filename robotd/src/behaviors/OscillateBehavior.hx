package robotd.behaviors;

import robotkit.behavior.RobotBehavior;
import robotkit.behavior.RobotContext;

/** Small deterministic behavior used to prove the Haxeon behavior path. */
class OscillateBehavior implements RobotBehavior {
  var phase:Float = 0.0;

  public function new() {}

  public function update(context:RobotContext):Void {
    phase += 0.12;
    if (phase > Math.PI * 2.0)
      phase -= Math.PI * 2.0;
    context.jointTarget(0, 1, Math.sin(phase) * 0.5);
  }
}
