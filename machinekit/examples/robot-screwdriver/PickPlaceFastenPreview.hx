import cadkit.modeling.AssemblyState;
import machinekit.assembly.AssemblyPreview;
import machinekit.robot.RobotScene;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import haxe.io.Bytes;

/** CAD cell and controller ownership for the two-arm assembly cycle. */
class PickPlaceFastenPreview {
	public static function scene():SceneArtifactData {
		var cell = new PickPlaceFastenCell();
		var scene = AssemblyPreview.scene(cell, "pick-place-fasten");
		scene.assemblyState = ScrewdrivingPreview.state(cell, "pick-place-fasten").record();
		scene.robotTools = RobotScene.robotTools(cell.arm.tool, "arm/tool")
			.concat(RobotScene.robotTools(cell.handling.tool, "handling/tool"));
		SceneArtifact.assignToolControllers(scene);
		return scene;
	}
	public static function cell():Bytes return SceneArtifact.encode(scene());

	/** First commissioning stage keeps the plate fixed; the full cycle also loads and unloads it. */
	public static function cycleScene(housingOnly:Bool = false):SceneArtifactData {
		var equipment = new PickPlaceFastenCell();
		var scene = ScrewdrivingAssemblyPreview.scene(false, false, false, false, false, equipment);
		scene.robotTools = scene.robotTools.concat(RobotScene.robotTools(equipment.handling.tool, "handling/tool"));
		SceneArtifact.assignToolControllers(scene);
		var definition:materia.assembly.AssemblyDefinition = cast scene.assemblyDefinition;
		if (definition == null || scene.missions == null || scene.missions.length != 1)
			throw "Assembly cycle lost its fastening program or CAD definition";
		var state = new AssemblyState(definition, scene.assemblyState);
		var fastening = scene.missions[0];
		// A free plate must receive thread reactions through the physical screw joint.
		// These are the SD13 simulation commissioning limits, not a hardware clamp recipe.
		fastening.screwEngagement = {mode:materia.project.SceneArtifact.SceneArtifactScrewEngagementMode.PhysicalThread,
			maximumAxialForceN:200, threadStiffnessNPerM:1.0e7};
		for (step in fastening.steps) if (step.screw != null) switch step.screw {
			case Single(job): step.screw = materia.project.SceneScrewInvocation.ScrewInvocation.Required(job);
			case _: throw "Assembly cycle expects independently qualified single-pass fastenings";
		}
		var screwPark = [for (i in 1...7) {joint:'arm/j$i', position:state.joint('arm/j$i')}];
		fastening.steps.insert(0, {kind:"waitFor", waitFor:{name:PickPlaceFastenCell.PART_READY, value:true, timeout:180}});
		// Park the six arm axes independently of the tool's passive feed and process spindle.
		fastening.steps.push({kind:"moveJoints", at:{occurrence:"arm/toolFlange", connector:"face"},
			joints:screwPark, timeoutSeconds:60});
		fastening.steps.push({kind:"signal", signal:{name:PickPlaceFastenCell.FASTENED, value:true}});
		fastening.steps.push({kind:"waitFor", waitFor:{name:PickPlaceFastenCell.PART_READY, value:false, timeout:180}});
		fastening.steps.push({kind:"signal", signal:{name:PickPlaceFastenCell.FASTENED, value:false}});
		scene.missions.push({controllerId:PickPlaceFastenCell.HANDLING_CONTROLLER, loop:false,
			steps:PickPlaceFastenMission.handling(equipment, housingOnly)});
		var freed = ["work/flange/housing"];
		if (!housingOnly) freed.push("work/plate");
		var poses:Map<String, materia.assembly.AssemblyRecord.AssemblyFrame> = [
			"work/flange/housing" => state.worldConnector("housingTray/seat-r0-c0-x-1-y-1", "seat"),
			"work/plate" => state.worldConnector("plateTray/seat-r0-c0-x-1-y-1", "seat")];
		definition.joints = [for (joint in definition.joints)
			if (freed.indexOf(joint.parent) < 0 && freed.indexOf(joint.child) < 0) joint];
		if (definition.mates != null) definition.mates = [for (mate in definition.mates)
			if (freed.indexOf(mate.first) < 0 && freed.indexOf(mate.second) < 0) mate];
		for (occurrence in definition.occurrences) if (freed.indexOf(occurrence.id) >= 0)
			occurrence.initialPose = poses.get(occurrence.id);
		var previous = state.record();
		var execution = new AssemblyState(definition, {schemaVersion:previous.schemaVersion, definition:definition.id,
			jointCoordinates:previous.jointCoordinates,
			rootPoses:[for (root in previous.rootPoses) if (freed.indexOf(root.occurrence) < 0) root]});
		scene.assemblyState = execution.record();
		return scene;
	}
	public static function cycle():Bytes return SceneArtifact.encode(cycleScene());
	public static function housingOnly():Bytes return SceneArtifact.encode(cycleScene(true));
}

function main():Void {
	var scene = SceneArtifact.decode(PickPlaceFastenPreview.cell());
	if (scene.controllers == null || scene.controllers.length != 2 || scene.robotTools == null || scene.robotTools.length != 3)
		throw "Assembly cell needs its two controllers and three fixed tools";
	for (tool in scene.robotTools) if (tool.controller != (tool.kind == "screwdriver"
		? PickPlaceFastenCell.FASTENING_CONTROLLER : PickPlaceFastenCell.HANDLING_CONTROLLER))
		throw "Every assembly-cell tool must belong to its physical arm's controller";
	if (scene.signals == null || scene.signals.length != 2)
		throw "Assembly cell needs its part-ready and fastened controller handshakes";
	trace("Pick, place and fasten CAD cell passed");
	var cycle = SceneArtifact.decode(PickPlaceFastenPreview.housingOnly());
	if (cycle.missions == null || cycle.missions.length != 2)
		throw "Assembly cycle needs one program per robot controller";
	trace("Pick, place and fasten controller programs passed");
}
