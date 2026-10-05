import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

/** Materia project entrypoint for the belt-driven Cartesian picker. */
class GantryPickerPreview {
	public static function picker():Bytes return build(false);
	public static function yawPicker():Bytes return build(true);

	static function build(yaw:Bool):Bytes {
		var picker = new GantryPicker(yaw ? machinekit.gantry.GantrySpec.GantryHead.C : machinekit.gantry.GantrySpec.GantryHead.None);
		var scene = AssemblyPreview.scene(picker, "gantry-picker");
		scene.robotTools = machinekit.robot.RobotScene.robotTools(picker.tool, "tool");
		var steps:Array<materia.project.SceneArtifact.SceneArtifactMissionStep> = [];
		for (index in 0...GantryPicker.BOX_COUNT) {
			var pick:materia.project.SceneArtifact.SceneArtifactMissionStep = {kind: "pick", at: {occurrence: "box" + index, connector: "top"}};
			var place:materia.project.SceneArtifact.SceneArtifactMissionStep = {kind: "place", at: {occurrence: "slot" + index, connector: "top"}};
			if (yaw) { pick.yaw = 0.0; place.yaw = (index % 2 == 0 ? 1 : -1) * Math.PI / 2; }
			steps.push(pick); steps.push(place);
		}
		scene.mission = {loop: false, steps: steps,
			powerUpSideOffsets: [{homeSwitch: "switchYRighthome", offset: 0.001}]};
		return SceneArtifact.encode(scene);
	}
}

class GantryPickerChecks {
	public static function run():Void {
		var scene = SceneArtifact.decode(GantryPickerPreview.picker());
		var definition = scene.assemblyDefinition;
		if (definition == null || scene.mission == null || scene.mission.steps.length != 12)
			throw "Gantry picker needs six pick/place pairs";
		if (scene.robotTools == null || scene.robotTools.length != 1 || scene.robotTools[0].sensor == null)
			throw "Gantry picker needs a sensed suction tool";
		var axes = [for (joint in definition.joints) if (Std.string(joint.type) == "prismatic") joint.id];
		if (axes.length != 3 || axes.indexOf("x") < 0 || axes.indexOf("y") < 0 || axes.indexOf("z") < 0)
			throw "Gantry picker needs three physical translational leaders";
		var rejected = false;
		scene.machining = {program: "M30", axes: ["x", "y", "z"], spindle: "tool",
			workOffset: [0.0, 0.0, 0.0], tools: []};
		try { var invalid = SceneArtifact.encode(scene); }
		catch (error:Dynamic) rejected = Std.string(error) == "Scene artifact cannot combine machining and a mission";
		if (!rejected) throw "A picker mission must reject simultaneous machining";
		scene.machining = null;
		var yawScene = SceneArtifact.decode(GantryPickerPreview.yawPicker());
		if (yawScene.mission == null || yawScene.mission.steps[1].yaw != Math.PI / 2 ||
			yawScene.mission.steps[3].yaw != -Math.PI / 2)
			throw "The C-head picker must retain its requested pallet headings";
		var yawMission:materia.project.SceneArtifact.SceneArtifactMission = cast yawScene.mission;
		var yawDefinition = yawScene.assemblyDefinition;
		if (yawDefinition == null || [for (joint in yawDefinition.joints) if (joint.id == "head/c") joint].length != 1)
			throw "The yaw picker needs an included C head";
		yawMission.steps[0].yaw = Math.NaN;
		rejected = false;
		try { SceneArtifact.encode(yawScene); } catch (_:Dynamic) { rejected = true; }
		if (!rejected) throw "A pick must reject a non-finite heading";
		Sys.println("gantry picker: 1500 x 1000 x 500 mm, six cartons, dual Y, belt drives");
	}
}

function main():Void GantryPickerChecks.run();
