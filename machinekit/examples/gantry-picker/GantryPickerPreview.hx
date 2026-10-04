import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

/** Materia project entrypoint for the belt-driven Cartesian picker. */
class GantryPickerPreview {
	public static function picker():Bytes {
		var picker = new GantryPicker();
		var scene = AssemblyPreview.scene(picker, "gantry-picker");
		scene.robotTools = AssemblyPreview.robotTools(picker.tool, "tool");
		var steps:Array<materia.project.SceneArtifact.SceneArtifactMissionStep> = [];
		for (index in 0...GantryPicker.BOX_COUNT) {
			steps.push({kind: "pick", at: {occurrence: "box" + index, connector: "top"}});
			steps.push({kind: "place", at: {occurrence: "slot" + index, connector: "top"}});
		}
		scene.mission = {loop: false, steps: steps};
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
		scene.machining = cast {};
		try { var invalid = SceneArtifact.encode(scene); }
		catch (error:Dynamic) rejected = Std.string(error) == "Scene artifact cannot combine machining and a mission";
		if (!rejected) throw "A picker mission must reject simultaneous machining";
		scene.machining = null;
		Sys.println("gantry picker: 1500 x 1000 x 500 mm, six cartons, dual Y, belt drives");
	}
}

function main():Void GantryPickerChecks.run();
