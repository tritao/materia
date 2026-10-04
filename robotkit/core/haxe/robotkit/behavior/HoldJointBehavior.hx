package robotkit.behavior;

/** Minimal reusable joint behavior used by simulation and replay tests. */
class HoldJointBehavior implements WorldBehavior {
  final joint:Int;
  final target:Float;

  public function new(joint:Int, target:Float) {
    this.joint = joint;
    this.target = target;
  }

  public function update(context:WorldBehaviorContext):Void
    context.jointPosition(joint, target);
}
