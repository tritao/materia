package machinekit.pendant;

import cadkit.threads.SurfaceDetail.DisplayBinding;
import haxe.io.Bytes;
import machinekit.assembly.MachineAssembly;
import machinekit.operator.OperatorAxis;
import machinekit.operator.OperatorKey;
import machinekit.operator.OperatorProfile;
import machinekit.operator.OperatorProfileCodec;
import machinekit.operator.PendantDisplaySignals;
import machinekit.operator.ReferenceOperatorProfiles;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinitionCodec;

/**
 * The machine a pendant belongs to: the operator profile its display serves and the joint each of the profile's axes moves when the
 * wheel jogs it. The pendant is the same on every machine; this is what makes it that machine's. The profile travels with the scene as
 * data (`profile`), so a machine needs no entry in any registry.
 */
class PendantMachine {
  /** The id the pendant is included under in a machine's assembly. */
  public static inline var ACCESSORY:String = "pendant";
  /** The most axes the pendant's selector switch can name. */
  static inline var POSITIONS:Int = OperatorProfile.PENDANT_POSITIONS;

  public final operator:OperatorProfile;
  /** The operator profile as the scene carries it (see `OperatorProfileCodec`). */
  public final profile:String;
  /** Which assembly joint a jog of each axis moves, as display bindings named by `PendantDisplaySignals.jog`. */
  public final jogs:Array<DisplayBinding>;
  /** What the pendant's own joints and occurrences are prefixed with in the scene: empty when it is the whole assembly, else `pendant/`. */
  public final prefix:String;

  public function new(operator:OperatorProfile, jogs:Array<DisplayBinding>, prefix:String = "") {
    this.operator = operator;
    profile = OperatorProfileCodec.encode(operator).toString();
    this.jogs = jogs;
    this.prefix = prefix;
  }

  /** The reference pendant on its own: the hobber's profile and no machine joints to move. */
  public static function reference():PendantMachine return new PendantMachine(ReferenceOperatorProfiles.hobber(), []);

  /**
   * The pendant of `machine`, included in it (so its own ids are prefixed). Each axis of `operator` jogs the joint `axisJoints` binds it
   * to, or else the joint of the same axis id; with no profile given, the machine's own is derived from its joints (`deriveProfile`).
   * Throws when a profile axis has no joint.
   */
  public static function attachedTo(
    machine:MachineAssembly,
    ?operator:OperatorProfile,
    ?axisJoints:Map<String, String>
  ):PendantMachine {
    var axes = operableJoints(machine);
    var profile = operator == null ? deriveProfile(axes) : operator;
    var jogs:Array<DisplayBinding> = [];
    for (axis in profile.axes) {
      var bound = axisJoints == null ? null : axisJoints.get(axis.id);
      var found = bound != null ? [bound] : [for (a in axes) if (a.axis == axis.id) a.joint];
      if (bound != null && !Lambda.exists(axes, a -> a.joint == bound)) throw 'The machine has no jog-able joint "$bound" for the axis "${axis.id}"';
      if (found.length == 0) throw 'The machine has no joint for the axis "${axis.id}" of the "${profile.name}" profile';
      jogs.push(new DisplayBinding(PendantDisplaySignals.jog(axis.id), found[0]));
    }
    return new PendantMachine(profile, jogs, ACCESSORY + "/");
  }

  /**
   * The joints an operator can jog: actuated tree joints with a coordinate, not passive and not following another (a dual drive's
   * second motor), in the machine's order. An axis is named by its joint's id, with `/` written `.`.
   */
  static function operableJoints(machine:MachineAssembly):Array<{axis:String, joint:String, label:String}> {
    var definition = machine.derived().definition;
    var followers:Map<String, Bool> = [];
    if (definition.couplings != null) for (coupling in definition.couplings) followers.set(coupling.target, true);
    var result = [];
    for (joint in definition.joints) {
      if (joint.role != AssemblyJointRole.Tree || !AssemblyDefinitionCodec.hasCoordinate(joint.type) || joint.passive == true
        || followers.exists(joint.id)) continue;
      var parts = joint.id.split("/");
      var label = parts[parts.length - 1].toUpperCase();
      result.push({axis: StringTools.replace(joint.id, "/", "."), joint: joint.id, label: label.length > OperatorAxis.MAX_LABEL ? label.substr(0, OperatorAxis.MAX_LABEL) : label});
    }
    // The axes an operator expects first, in their usual order; any others after them, in the machine's order.
    var usual = ["x", "y", "z", "a", "b", "c"];
    var rank = (axis:String) -> usual.indexOf(axis) >= 0 ? usual.indexOf(axis) : usual.length;
    var order = [for (i in 0...result.length) i];
    order.sort((a, b) -> rank(result[a].axis) != rank(result[b].axis) ? rank(result[a].axis) - rank(result[b].axis) : a - b);
    return [for (i in order) result[i]];
  }

  /** A profile of just the machine's axes on the pendant's selector positions, in order: jog mode, no keys, door or spindle. */
  static function deriveProfile(axes:Array<{axis:String, joint:String, label:String}>):OperatorProfile {
    if (axes.length == 0) throw "The machine has no joint an operator can jog";
    if (axes.length > POSITIONS) throw 'The machine has ${axes.length} axes; the pendant\'s selector names $POSITIONS';
    var positions:Array<Null<String>> = [for (i in 0...POSITIONS) i < axes.length ? axes[i].axis : null];
    return new OperatorProfile(
      "machine",
      [for (a in axes) new OperatorAxis(a.axis, a.label)],
      positions,
      [new OperatorKey(Unused, ""), new OperatorKey(Unused, ""), new OperatorKey(Unused, "")],
      [Jog],
      false,
      false,
      []
    );
  }
}
