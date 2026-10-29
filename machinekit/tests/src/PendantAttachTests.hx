import machinekit.operator.PendantDisplaySignals;
import machinekit.operator.ReferenceOperatorProfiles;
import machinekit.pendant.PendantMachine;
import machinekit.pendant.ReferencePendant;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

/** The pendant attached to a real machine: its profile's axes jog the machine's own joints, and a machine without them is refused. */
class PendantAttachTests {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw message;
  }

  static function main():Void {
    var stage = new XyzStage();
    var machine = PendantMachine.attachedTo(stage, XyzStagePreview.PROFILE);
    check(machine.prefix == "pendant/", "the pendant is included under its own id");
    var profile = ReferenceOperatorProfiles.named(XyzStagePreview.PROFILE);
    if (profile == null) throw "the xyz profile exists";
    check(machine.jogs.length == profile.axes.length, "one jog binding for each axis of the profile");
    var joints = [for (j in stage.derived().definition.joints) j.id => j];
    for (binding in machine.jogs) {
      var axis = binding.signal.substr(PendantDisplaySignals.jog("").length);
      var joint = joints.get(binding.joint);
      if (joint == null) throw "the stage has the joint " + binding.joint;
      check(binding.joint == axis, "an axis jogs the joint of its id");
      check(joint.type == Prismatic && joint.limits.lower == 0.0, "a prismatic joint from zero: " + binding.joint);
    }
    var x = joints.get("x"), z = joints.get("z");
    if (x == null || z == null) throw "the stage has x and z";
    check(x.limits.upper == XyzStage.TRAVEL_X && z.limits.upper == XyzStage.TRAVEL_Z, "the joints carry the stage's travel");
    // The pendant's own bindings are prefixed to where the pendant sits in the machine.
    var bindings = ReferencePendant.displayBindings(machine);
    check([for (b in bindings) if (b.signal == PendantDisplaySignals.WHEEL) b.joint].join(",") == "pendant/jog/wheel/encoder", "the wheel's joint is prefixed");
    // A machine without the joints a profile names is refused, not silently left with a dead wheel.
    var refused = false;
    try PendantMachine.attachedTo(new machinekit.assembly.MachineAssembly(), "xyz") catch (_:Dynamic) refused = true;
    check(refused, "a machine without x, y and z cannot take the xyz pendant");
    // The whole scene builds with the pendant's joints under the machine's.
    var scene = SceneArtifact.decode(XyzStagePreview.withPendant());
    var definition = scene.assemblyDefinition;
    if (definition == null) throw "the scene has an assembly";
    var ids = [for (j in definition.joints) j.id => true];
    for (id in ["x", "y", "z", "pendant/jog/wheel/encoder", "pendant/estop/press", "pendant/cycle-keys/start/press"]) check(ids.exists(id), "the scene has the joint " + id);
    Sys.println("pendant attach tests passed");
  }
}
