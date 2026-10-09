import haxe.io.Bytes;
import materia.project.SceneArtifact;
import cnckit.controller.RobotInterfaceChannels;
import cnckit.controller.RobotInterface;
import machinekit.robotics.DualArmGripperTool;

/** The playable four-part machine-tending project. */
class TendingCell {
	public static function preview():Bytes {
		var cell = new TendingControllers();
		var scene = SceneArtifact.decode(cell.previewArtifact());
		scene.mission = {controllerId: TendingMission.ROBOT_CONTROLLER, loop: false,
			steps: TendingMission.generate(cell, [0, 1, 2, 3])};
		return SceneArtifact.encode(scene);
	}
}

class TendingMissionChecks {
	public static function run():Void {
		var scene = SceneArtifact.decode(TendingCell.preview());
		var mission = scene.mission;
		if (mission == null || mission.controllerId != TendingMission.ROBOT_CONTROLLER || mission.loop == true)
			throw "Tending cell needs its finite robot-controlled mission";
		var picks = [for (step in mission.steps) if (step.kind == "pick") step];
		var places = [for (step in mission.steps) if (step.kind == "place") step];
		var cuts:Array<materia.project.SceneArtifact.SceneArtifactMissionStep> = [];
		for (step in mission.steps) {
			var waitFor = step.waitFor;
			if (step.kind == "waitFor" && waitFor != null && waitFor.name == "cnc.cycle_complete") cuts.push(step);
		}
		var via = [for (step in mission.steps) if (step.kind == "moveVia") step];
		var requests = [for (step in mission.steps) if (step.kind == "signal") step];
		if (via.length == 0 || via[0].poses == null || via[0].poses.length != 2)
			throw "The first tray approach needs both the geometry-derived clearance and approach poses";
		function signalCount(name:String, value:Bool):Int {
			var count = 0;
			for (step in requests) if (step.signal != null && step.signal.name == name && step.signal.value == value) count++;
			return count;
		}
		if (picks.length != 8 || places.length != 8 || cuts.length != 4 || via.length < 12 || requests.length == 0)
			throw 'Tending mission must unroll four pick/place/cut cycles; got ${picks.length}/${places.length}/${cuts.length}/${via.length}';
		if (signalCount(RobotInterfaceChannels.REQUEST_DOOR_OPEN, true) != 5 ||
			signalCount(RobotInterfaceChannels.REQUEST_DOOR_CLOSE, true) != 5 ||
			signalCount(RobotInterfaceChannels.REQUEST_CLAMP, true) != 4 ||
			signalCount(RobotInterfaceChannels.REQUEST_UNCLAMP, true) != 4 ||
			signalCount(RobotInterfaceChannels.CYCLE_START, true) != 4)
			throw "Each tending cycle needs door, clamp, unclamp and CNC start outputs";
		for (step in mission.steps) if (step.kind == "waitFor" && step.waitFor != null &&
			StringTools.startsWith(step.waitFor.name, "cnc.") &&
			step.waitFor.name != RobotInterfaceChannels.CYCLE_COMPLETE &&
			step.waitFor.timeout <= RobotInterface.DEFAULT_REQUEST_TIMEOUT_SECONDS)
			throw 'Mission wait for ${step.waitFor.name} must leave a signal round trip after the PLC timeout';
		var expectedPickParts = ["infeed/blank-0-0", "infeed/blank-0-1", "infeed/blank-0-0",
			"infeed/blank-0-2", "infeed/blank-0-1", "infeed/blank-1-0", "infeed/blank-0-2", "infeed/blank-1-0"];
		var expectedPickTools = [DualArmGripperTool.TOOL_CHANNEL_A, DualArmGripperTool.TOOL_CHANNEL_B,
			DualArmGripperTool.TOOL_CHANNEL_A, DualArmGripperTool.TOOL_CHANNEL_A,
			DualArmGripperTool.TOOL_CHANNEL_B, DualArmGripperTool.TOOL_CHANNEL_B,
			DualArmGripperTool.TOOL_CHANNEL_A, DualArmGripperTool.TOOL_CHANNEL_B];
		for (index in 0...picks.length) {
			var pick = picks[index], at = pick.at;
			if (at == null || at.occurrence != expectedPickParts[index] || at.connector != "grasp" ||
				pick.tool != expectedPickTools[index] || pick.yaw != null || pick.toolYaw == null || !Math.isFinite(pick.toolYaw))
				throw 'Tending pick $index does not use the staged blank and alternating gripper';
		}
		var expectedPlaceParts = ["mill/mill/vise/body", "mill/mill/vise/body", "outfeedTray/seat-r0-c0-x-1-y-1",
			"mill/mill/vise/body", "outfeedTray/seat-r0-c1-x-1-y-1", "mill/mill/vise/body",
			"outfeedTray/seat-r0-c2-x-1-y-1", "outfeedTray/seat-r1-c0-x-1-y-1"];
		var expectedPlaceTools = [DualArmGripperTool.TOOL_CHANNEL_A, DualArmGripperTool.TOOL_CHANNEL_B,
			DualArmGripperTool.TOOL_CHANNEL_A, DualArmGripperTool.TOOL_CHANNEL_A,
			DualArmGripperTool.TOOL_CHANNEL_B, DualArmGripperTool.TOOL_CHANNEL_B,
			DualArmGripperTool.TOOL_CHANNEL_A, DualArmGripperTool.TOOL_CHANNEL_B];
		for (index in 0...places.length) {
			var place = places[index], at = place.at;
			var connector = StringTools.startsWith(expectedPlaceParts[index], "mill/") ? "datum" : "seat";
			if (at == null || at.occurrence != expectedPlaceParts[index] || at.connector != connector ||
				place.tool != expectedPlaceTools[index] || place.yaw != null || place.toolYaw == null || !Math.isFinite(place.toolYaw))
				throw 'Tending place $index does not transfer finished and staged blanks on their assigned grippers';
		}
		var viaA = 0, viaB = 0;
		for (step in via) {
			var at = step.at, poses = step.poses;
			if (at == null || poses == null || poses.length == 0)
				throw "Every tending travel must use checked via-points at the mounted TCP";
			if (at.occurrence == "arm/" + DualArmGripperTool.GRIPPER_A_BODY) viaA++;
			if (at.occurrence == "arm/" + DualArmGripperTool.GRIPPER_B_BODY) viaB++;
		}
		if (viaA == 0 || viaB == 0) throw "Tending via-paths must be certified for both physical gripper TCPs";
		for (step in mission.steps) if ((step.kind == "pick" || step.kind == "place" || step.kind == "moveVia") &&
			(step.timeoutSeconds == null || step.timeoutSeconds <= 0))
			throw 'Tending ${step.kind} step needs a simulation-time deadline';
		for (step in mission.steps) if ((step.kind == "pick" || step.kind == "place") &&
			(step.approachDistance == null || step.approachDistance <= 0))
			throw 'Tending ${step.kind} step needs its reach-checked physical approach distance';
		for (index in 0...mission.steps.length) {
			var step = mission.steps[index];
			if (step.kind != "waitFor" || step.waitFor == null) continue;
			var retry = step.waitFor.retryStep;
			if (retry != null) {
				var retryIndex = retry;
				var retryCommand = retryIndex >= 0 && retryIndex < mission.steps.length
					? mission.steps[retryIndex] : null;
				if (retryIndex < 0 || retryIndex >= index || retryCommand == null || retryCommand.kind != "signal" ||
					retryCommand.signal == null || retryCommand.signal.value != true)
					throw 'Tending wait at step $index does not retry a prior asserted output';
			}
		}
		for (cycle in 0...4) {
			var row = Std.int(cycle / 3), column = cycle % 3;
			var present = 'infeed/slot-$row-$column-present';
			var waitIndex = -1, pickIndex = -1;
			for (index in 0...mission.steps.length) {
				var step = mission.steps[index], condition = step.waitFor;
				if (step.kind == "waitFor" && condition != null && condition.name == present && waitIndex < 0)
					waitIndex = index;
				if (step.kind == "pick" && step.at != null && step.at.occurrence == 'infeed/blank-$row-$column' && pickIndex < 0)
					pickIndex = index;
			}
			if (waitIndex < 0 || pickIndex <= waitIndex)
				throw 'Tending cycle $cycle must confirm its physical infeed slot before reaching for the blank';
		}
		var completedCycles = [for (index in 0...mission.steps.length) {
			var step = mission.steps[index], waitFor = step.waitFor;
			step.kind == "waitFor" && waitFor != null && waitFor.name == "cnc.cycle_complete" ? index : -1;
		}].filter(index -> index >= 0);
		if (completedCycles.length != 4) throw "The tending mission needs four CNC-complete handshakes";
		var cycleStarts:Array<Int> = [];
		for (index in 0...mission.steps.length) {
			var signal = mission.steps[index].signal;
			if (mission.steps[index].kind == "signal" && signal != null &&
				signal.name == RobotInterfaceChannels.CYCLE_START && signal.value) cycleStarts.push(index);
		}
		if (cycleStarts.length != 4) throw "Every unattended cut needs one CNC start edge";
		for (cycle in 0...4) {
			if (cycle < 3) {
				var staged = -1;
				for (index in 0...mission.steps.length) {
					var step = mission.steps[index];
					if (staged < 0 && step.kind == "pick" && step.at != null &&
						step.at.occurrence == expectedPickParts[cycle * 2 + 1]) staged = index;
				}
				if (staged <= cycleStarts[cycle] || staged >= completedCycles[cycle])
					throw 'Cycle $cycle must stage the next blank after starting and before the current cut completes';
			}
			if (cycle > 0) {
				var unloaded = -1;
				for (index in 0...mission.steps.length) {
					var step = mission.steps[index];
					if (step.kind == "place" && step.at != null &&
						step.at.occurrence == expectedPlaceParts[cycle * 2]) unloaded = index;
				}
				if (unloaded <= cycleStarts[cycle] || unloaded >= completedCycles[cycle])
					throw 'Cycle $cycle must unload its prior finish while the mill cuts';
			}
		}
		for (cycle in 0...completedCycles.length) {
			var complete = completedCycles[cycle];
			var end = cycle + 1 < completedCycles.length ? completedCycles[cycle + 1] : mission.steps.length;
			var start = complete + 1;
			var ready = -1, door = -1, unclamp = -1, unclamped = -1, pick = -1;
			for (stepIndex in start...end) {
				var step = mission.steps[stepIndex];
				var waitFor = step.waitFor, signal = step.signal;
				if (ready < 0 && step.kind == "waitFor" && waitFor != null && waitFor.name == "cnc.ready") ready = stepIndex;
				if (door < 0 && step.kind == "waitFor" && waitFor != null && waitFor.name == "cnc.door_open") door = stepIndex;
				if (unclamp < 0 && step.kind == "signal" && signal != null && signal.name == "robot.request_unclamp" && signal.value)
					unclamp = stepIndex;
				if (unclamped < 0 && step.kind == "waitFor" && waitFor != null && waitFor.name == "cnc.unclamped") unclamped = stepIndex;
				if (pick < 0 && step.kind == "pick") pick = stepIndex;
			}
			if (ready <= complete || door <= ready || unclamp <= door || unclamped <= unclamp || pick <= unclamped)
				throw 'Tending cycle $cycle must wait for CNC completion, standstill and open door, then unclamp before picking';
		}
		Sys.println('Tending cell mission: ${mission.steps.length} steps, four unattended cycles');
	}
}
