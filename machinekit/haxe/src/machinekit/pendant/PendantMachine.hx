package machinekit.pendant;

import cadkit.threads.SurfaceDetail.DisplayBinding;
import machinekit.assembly.MachineAssembly;
import machinekit.operator.PendantDisplaySignals;
import machinekit.operator.ReferenceOperatorProfiles;

/**
 * The machine a pendant belongs to: the operator profile its display serves and the joint each of the profile's axes moves when the
 * wheel jogs it. The pendant is the same on every machine; this is what makes it that machine's.
 */
class PendantMachine {
  /** The id the pendant is included under in a machine's assembly. */
  public static inline var ACCESSORY:String = "pendant";

  /** The name of the operator profile (see `ReferenceOperatorProfiles.named`). */
  public final profile:String;
  /** Which assembly joint a jog of each axis moves, as display bindings named by `PendantDisplaySignals.jog`. */
  public final jogs:Array<DisplayBinding>;
  /** What the pendant's own joints and occurrences are prefixed with in the scene: empty when it is the whole assembly, else `pendant/`. */
  public final prefix:String;

  public function new(profile:String, jogs:Array<DisplayBinding>, prefix:String = "") {
    this.profile = profile;
    this.jogs = jogs;
    this.prefix = prefix;
  }

  /** The reference pendant on its own: the hobber's profile and no machine joints to move. */
  public static function reference():PendantMachine return new PendantMachine("hobber", []);

  /**
   * The pendant of `machine` under the operator profile `profile`: each of the profile's axes jogs the machine's joint of the same id.
   * The pendant is included in the machine, so its own ids are prefixed. Throws when the machine lacks a joint a profile axis names,
   * so a renamed joint is found here and not by a wheel that moves nothing.
   */
  public static function attachedTo(machine:MachineAssembly, profile:String):PendantMachine {
    var operator = ReferenceOperatorProfiles.named(profile);
    if (operator == null) throw 'There is no operator profile called "$profile"';
    var joints = [for (joint in machine.derived().definition.joints) joint.id => joint];
    var jogs:Array<DisplayBinding> = [];
    for (axis in operator.axes) {
      var joint = joints.get(axis.id);
      if (joint == null) throw 'The machine has no joint "${axis.id}" for the axis "${axis.label}" of the "$profile" profile';
      jogs.push(new DisplayBinding(PendantDisplaySignals.jog(axis.id), axis.id));
    }
    return new PendantMachine(profile, jogs, ACCESSORY + "/");
  }
}
