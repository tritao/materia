import machinekit.operator.PendantDisplaySignals;
import machinekit.operator.ReferenceOperatorProfiles;
import machinekit.pendant.ReferencePendant;

/** The pendant beside its demonstration stage: the profile fits the stage and every axis's binding names a joint of the assembly. */
class PendantStageTests {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw message;
  }

  static function main():Void {
    var profile = ReferenceOperatorProfiles.named(JogStage.PROFILE);
    if (profile == null) throw "the stage has an operator profile";
    check(profile.axes.length == 3, "the stage has a profile of three axes");
    var assembly = ReferencePendant.assembly();
    JogStage.addTo(assembly);
    var joints = [for (j in assembly.derived().definition.joints) j.id => j];
    var machine = JogStage.machine();
    check(machine.jogs.length == 3, "one jog binding for each axis");
    for (binding in machine.jogs) {
      var axis = binding.signal.substr(PendantDisplaySignals.jog("").length);
      check(profile.axes.filter(a -> a.id == axis).length == 1, "the profile has axis " + axis);
      var joint = joints.get(binding.joint);
      if (joint == null) throw ("the assembly has the joint " + binding.joint + " in: " + [for (id in joints.keys()) id].join(", "));
      check(joint.type == Prismatic && joint.limits.lower == 0.0 && joint.limits.upper == 120.0, "a slider of 120 mm travel: " + binding.joint);
    }
    // The pendant's display serves the stage's profile and carries the jog bindings.
    var layout = ReferencePendant.layout(null, false, machine);
    check(layout.problems().length == 0, "the pendant's layout is sound: " + layout.problems().join(" | "));
    var scene = PendantPreview.stage();
    check(scene.length > 1000, "the stage scene encodes");
    Sys.println("pendant stage tests passed");
  }
}
