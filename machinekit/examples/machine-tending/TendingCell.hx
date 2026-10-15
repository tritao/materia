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
		scene.missions = [{controllerId: TendingMission.ROBOT_CONTROLLER, loop: false,
			steps: TendingMission.generate(cell)}];
		return SceneArtifact.encode(scene);
	}
}

class TendingMissionChecks {
	public static function run():Void {
		TendingCyclePlan.TendingCyclePlanChecks.run();
		var scene = SceneArtifact.decode(TendingCell.preview());
		check(scene.missions[0]);
	}

	/** Validate the shipped sequence without constructing another CAD cell. */
	public static function check(mission:Null<SceneArtifact.SceneArtifactMission>):Void {
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
		if (via.length == 0)
			throw "The first tray approach needs both the geometry-derived clearance and approach poses";
		var firstViaPoses = via[0].poses;
		if (firstViaPoses == null || firstViaPoses.length < 2)
			throw "The first tray approach needs both the geometry-derived clearance and approach poses";
		function signalCount(name:String, value:Bool):Int {
			var count = 0;
			for (step in requests) {
				var signal = step.signal;
				if (signal != null && signal.name == name && signal.value == value) count++;
			}
			return count;
		}
		if (picks.length != 8 || places.length != 8 || cuts.length != 4 || via.length < 12 || requests.length == 0)
			throw 'Tending mission must unroll four pick/place/cut cycles; got ${picks.length}/${places.length}/${cuts.length}/${via.length}';
		if (signalCount(RobotInterfaceChannels.REQUEST_DOOR_OPEN, true) != 5 ||
			signalCount(RobotInterfaceChannels.REQUEST_DOOR_CLOSE, true) != 5 ||
			signalCount(RobotInterfaceChannels.REQUEST_CLAMP, true) != 4 ||
			signalCount(RobotInterfaceChannels.REQUEST_UNCLAMP, true) != 5 ||
			signalCount(RobotInterfaceChannels.CYCLE_START, true) != 4)
			throw "Each tending cycle needs door, clamp, unclamp and CNC start outputs";
		for (step in mission.steps) {
			var waitFor = step.waitFor;
			if (step.kind == "waitFor" && waitFor != null &&
				StringTools.startsWith(waitFor.name, "cnc.") &&
				waitFor.name != RobotInterfaceChannels.CYCLE_COMPLETE &&
				waitFor.timeout <= RobotInterface.DEFAULT_REQUEST_TIMEOUT_SECONDS)
				throw 'Mission wait for ${waitFor.name} must leave a signal round trip after the PLC timeout';
		}
		var selectedSlots = ["0-0", "0-1", "0-2", "1-0"];
		var outfeedSeats = ["seat-r0-c0-x-1-y-1", "seat-r0-c1-x-1-y-1",
			"seat-r0-c2-x-1-y-1", "seat-r1-c0-x-1-y-1"];
		var inventory = new Map<String, String>(), handled = new Map<String, Int>();
		var sourcePicks = new Map<String, Int>(), outfeeds = new Map<String, Int>();
		var maximumHeld = 0;
		for (index in 0...mission.steps.length) {
			var step = mission.steps[index], at = step.at, tool = step.tool;
			if (step.kind != "pick" && step.kind != "place") continue;
			if (at == null || tool == null || step.yaw != null || step.toolYaw == null || !Math.isFinite(step.toolYaw))
				throw 'Handling step $index needs its own physical TCP and checked roll';
			if (step.kind == "pick") {
				var cycle = selectedSlots.indexOf(at.occurrence.substr("infeed/blank-".length));
				if (!StringTools.startsWith(at.occurrence, "infeed/blank-") || cycle < 0 ||
					at.connector != "grasp" || inventory.exists(tool) ||
					tool != (cycle % 2 == 0 ? DualArmGripperTool.TOOL_CHANNEL_A : DualArmGripperTool.TOOL_CHANNEL_B))
					throw 'Pick $index has the wrong part, occupied gripper or TCP';
				if (!handled.exists(at.occurrence)) sourcePicks.set(at.occurrence, index);
				var count = handled.get(at.occurrence);
				handled.set(at.occurrence, count == null ? 1 : count + 1);
				inventory.set(tool, at.occurrence);
				var held = [for (part in inventory) part].length;
				maximumHeld = Std.int(Math.max(maximumHeld, held));
			} else {
				var part = inventory.get(tool);
				if (part == null) throw 'Place $index has no part in its selected gripper';
				var cycle = selectedSlots.indexOf(part.substr("infeed/blank-".length));
				var load = at.occurrence == "mill/mill/vise/body" && at.connector == "load";
				if (!load && (at.occurrence != "outfeedTray/" + outfeedSeats[cycle] || at.connector != "seat"))
					throw 'Place $index has the wrong destination for $part';
				if (!load) outfeeds.set(part, index);
				inventory.remove(tool);
			}
		}
		if (maximumHeld != 2 || [for (part in inventory) part].length != 0)
			throw "The exchange must carry two distinct parts and finish with both grippers empty";
		for (slot in selectedSlots) if (handled.get("infeed/blank-" + slot) != 2 || !outfeeds.exists("infeed/blank-" + slot))
			throw 'Blank $slot must be loaded, unloaded and delivered to outfeed';
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
		for (cycle in 0...3) {
			var nextPick = sourcePicks.get("infeed/blank-" + selectedSlots[cycle + 1]);
			var finishedOutfeed = outfeeds.get("infeed/blank-" + selectedSlots[cycle]);
			if (nextPick == null || nextPick <= cycleStarts[cycle] || nextPick >= completedCycles[cycle] ||
				finishedOutfeed == null || finishedOutfeed <= cycleStarts[cycle + 1])
				throw 'Cycle $cycle must pre-stage during cutting and deliver its finish after starting the next cut';
		}
		for (cycle in 0...completedCycles.length) {
			var complete = completedCycles[cycle];
			var staging = -1;
			for (index in cycleStarts[cycle]...complete)
				if (mission.steps[index].kind == "moveVia") staging = index;
			var unloadingBody = cycle % 2 == 0 ? DualArmGripperTool.GRIPPER_A_BODY : DualArmGripperTool.GRIPPER_B_BODY;
			var stagingAt = staging < 0 ? null : mission.steps[staging].at;
			if (stagingAt == null || stagingAt.occurrence != "arm/" + unloadingBody)
				throw 'Cycle $cycle must stage its unloading gripper before CNC completion';
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
