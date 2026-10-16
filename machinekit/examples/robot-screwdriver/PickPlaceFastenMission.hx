import materia.project.SceneArtifact.SceneArtifactMissionStep;

/** Controller-local handling program; the other robot is coordinated only through wired I/O. */
class PickPlaceFastenMission {
	public static function handling(cell:PickPlaceFastenCell, housingOnly:Bool):Array<SceneArtifactMissionStep> {
		var steps:Array<SceneArtifactMissionStep> = [];
		var housingTool = "handling/tool/gripperA/body.close";
		var plateTool = "handling/tool/gripperB/body.close";
		function wait(name:String, value:Bool, timeout:Float = 10):Void
			steps.push({kind:"waitFor", waitFor:{name:name, value:value, timeout:timeout}});
		function signal(name:String, value:Bool):Void
			steps.push({kind:"signal", signal:{name:name, value:value}});
		function pick(part:String, tool:String):Void
			steps.push({kind:"pick", at:{occurrence:part, connector:"grasp"}, tool:tool,
				toolYaw:tool == plateTool ? Math.PI/2 : Math.PI, approachDistance:0.08, timeoutSeconds:60});
		function place(part:String, connector:String, tool:String):Void
			steps.push({kind:"place", at:{occurrence:part, connector:connector}, tool:tool,
				toolYaw:tool == plateTool ? Math.PI/2 : Math.PI, approachDistance:0.08, timeoutSeconds:60});
		function park():Void steps.push({kind:"moveJoints", at:{occurrence:"handling/tool/gripperA/body", connector:"tcp"},
			joints:[for (i in 0...6) {joint:'handling/j${i+1}', position:cell.handlingPark[i]}], timeoutSeconds:60});
		signal(PickPlaceFastenCell.PART_READY, false);
		if (!housingOnly) {
			wait("plate-present", true);
			pick("work/plate", plateTool);
			place("table", "plateSeat", plateTool);
		}
		// The commissioning variant has a mechanically fixed plate, part of the fixture's rigid body.
		// Presence heads observe dynamic, unheld workpieces; production confirms its newly loaded plate.
		if (!housingOnly) wait("fixture-present", true);
		wait("housing-present", true);
		pick("work/flange/housing", housingTool);
		place("work/plate", "housingSeat", housingTool);
		park();
		signal(PickPlaceFastenCell.PART_READY, true);
		wait(PickPlaceFastenCell.FASTENED, true, 300);
		signal(PickPlaceFastenCell.PART_READY, false);
		wait(PickPlaceFastenCell.FASTENED, false);
		if (!housingOnly) {
			wait("outfeed-present", false);
			pick("work/plate", plateTool);
			place("outfeedTray/seat-r0-c0-x-1-y-1", "seat", plateTool);
			park();
			wait("outfeed-present", true);
		}
		return steps;
	}
}
