import machinekit.operator.PendantDisplaySignals;
import machinekit.operator.OperatorProfileCodec;
import machinekit.pendant.PendantMachine;
import machinekit.pendant.ReferencePendant;
import materia.project.SceneArtifact;

/** The pendant attached to a real machine: its profile comes from the machine's joints, they are what its wheel jogs, and a machine with none is refused. */
class PendantAttachTests {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw message;
  }

  static function main():Void {
    var stage = new XyzStage();
    var machine = PendantMachine.attachedTo(stage);
    check(machine.prefix == "pendant/", "the pendant is included under its own id");
    // The profile is the machine's own: its three actuated joints, in order, on the selector's first three positions.
    var profile = machine.operator;
    check([for (a in profile.axes) a.id].join(",") == "x,y,z", "the profile's axes are the stage's joints: " + [for (a in profile.axes) a.id].join(","));
    check(profile.pendantAxes[0] == "x" && profile.pendantAxes[1] == "y" && profile.pendantAxes[2] == "z" && profile.pendantAxes[3] == null, "on the first positions of the selector");
    check(!profile.hasSpindle && !profile.hasDoor, "a stage has no spindle or door");
    check(machine.jogs.length == 3, "one jog binding for each axis");
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
    // The profile travels as data, and reads back the same.
    var back = OperatorProfileCodec.decode(haxe.io.Bytes.ofString(machine.profile));
    check(back.name == profile.name && back.axes.length == 3 && back.pendantAxes[2] == "z", "the encoded profile round trips");
    // The pendant's own bindings are prefixed to where the pendant sits in the machine.
    var bindings = ReferencePendant.displayBindings(machine);
    check([for (b in bindings) if (b.signal == PendantDisplaySignals.WHEEL) b.joint].join(",") == "pendant/jog/wheel/encoder", "the wheel's joint is prefixed");
    // A machine with no joint to jog is refused, not left with a dead wheel; so is a profile axis the machine lacks.
    var refused = false;
    try PendantMachine.attachedTo(new machinekit.assembly.MachineAssembly()) catch (_:Dynamic) refused = true;
    check(refused, "a machine without a joint cannot take a pendant");
    refused = false;
    try PendantMachine.attachedTo(stage, machinekit.operator.ReferenceOperatorProfiles.drillTap()) catch (_:Dynamic) refused = true;
    check(refused, "a profile with an axis the stage lacks is refused");
    // The whole scene builds with the pendant's joints under the machine's, and the display carries the machine's profile.
    var scene = SceneArtifact.decode(XyzStagePreview.withPendant());
    var definition = scene.assemblyDefinition;
    if (definition == null) throw "the scene has an assembly";
    var ids = [for (j in definition.joints) j.id => true];
    for (id in ["x", "y", "z", "pendant/jog/wheel/encoder", "pendant/estop/press", "pendant/cycle-keys/start/press"]) check(ids.exists(id), "the scene has the joint " + id);
    // A one-axis stage: the machine's own profile names its joint (a library axis whose carriage joint is not called "x").
    var axis = LinearStage.axis();
    var linear = PendantMachine.attachedTo(axis, LinearStage.operator(), LinearStage.axisJoints());
    check(linear.jogs.length == 1 && linear.jogs[0].joint == LinearStage.JOINT, "the carriage's joint is what the single axis jogs");
    var carriage = [for (j in axis.derived().definition.joints) if (j.id == LinearStage.JOINT) j][0];
    var low = carriage.limits.lower, high = carriage.limits.upper;
    if (low == null || high == null) throw "the carriage joint is limited";
    check(carriage.type == Prismatic && Math.abs(high - low - LinearStage.TRAVEL) < 1e-6, "the carriage travels the stage's stroke");
    var wrong = false;
    try PendantMachine.attachedTo(axis, LinearStage.operator(), ["x" => "no-such-joint"]) catch (_:Dynamic) wrong = true;
    check(wrong, "an axis bound to a joint the machine lacks is refused");
    // Without a binding, the profile's axis id has to be a joint's id.
    var unbound = false;
    try PendantMachine.attachedTo(axis, LinearStage.operator()) catch (_:Dynamic) unbound = true;
    check(unbound, "a profile axis with no binding and no joint of that id is refused");
    var linearScene = SceneArtifact.decode(LinearStagePreview.withPendant());
    var linearDefinition = linearScene.assemblyDefinition;
    if (linearDefinition == null) throw "the linear stage scene has an assembly";
    var linearIds = [for (j in linearDefinition.joints) j.id => true];
    for (id in [LinearStage.JOINT, "pendant/jog/wheel/encoder"]) check(linearIds.exists(id), "the linear stage scene has the joint " + id);
    Sys.println("pendant attach tests passed");
  }
}
