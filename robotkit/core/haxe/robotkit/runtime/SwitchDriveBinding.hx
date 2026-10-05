package robotkit.runtime;

import robotkit.model.JointSwitch;

/** Convert a switch side's observed drive shaft into its monitored axis coordinate. */
class SwitchDriveBinding {
  public final joint:Int;
  public final ratio:Float;
  public final offset:Float;
  function new(joint:Int, ratio:Float, offset:Float) {
    this.joint = joint; this.ratio = ratio; this.offset = offset;
  }
  public function position(shaft:Float):Float return (shaft - offset) / ratio;

  public static function resolve(blueprint:RobotRuntimeBlueprint, contact:JointSwitch):SwitchDriveBinding {
    if (blueprint == null || blueprint.identity == null || contact == null) throw "Switch drive binding requires identity";
    var leader = -1, drive = -1;
    for (i in 0...blueprint.jointCount) {
      var id = blueprint.identity.jointId(i);
      if (id == contact.joint) leader = i;
      if (id == (contact.driveJoint == null ? contact.joint : contact.driveJoint)) drive = i;
    }
    if (leader < 0 || drive < 0) throw "Switch drive binding references an unknown joint";
    var current = drive, ratio = 1.0, offset = 0.0;
    var visited = new Map<Int, Bool>();
    while (current != leader) {
      if (visited.exists(current)) throw "Switch drive coupling is cyclic";
      visited.set(current, true);
      var selected:Null<RobotRuntimeJointCouplingBlueprint> = null;
      for (term in blueprint.couplings) if (term.follower == current) {
        if (selected != null) throw "A switch side needs a single-input motor coupling";
        selected = term;
      }
      if (selected == null) throw "Switch motor does not follow its monitored axis";
      offset += ratio * selected.offset;
      ratio *= selected.ratio;
      if (!Math.isFinite(ratio) || ratio == 0.0 || !Math.isFinite(offset)) throw "Invalid switch drive ratio/zero";
      current = selected.leader;
    }
    return new SwitchDriveBinding(drive, ratio, offset);
  }
}
