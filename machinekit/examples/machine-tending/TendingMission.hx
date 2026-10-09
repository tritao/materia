import cnckit.controller.RobotInterfaceChannels;
import cnckit.controller.RobotInterface;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
import materia.project.SceneArtifact.SceneArtifactMissionWaypoint;
import materia.project.SceneArtifact.SceneArtifactJointTarget;
import machinekit.robotics.DualArmGripperTool;

/** Unrolled, signal-interlocked tending cycles from the cell's checked reach poses. */
class TendingMission {
	public static inline var ROBOT_CONTROLLER:String = "robotController";
	public static inline var CNC_CONTROLLER:String = "mill/cncController";
	static inline var CNC_SIGNAL_TIMEOUT:Float = RobotInterface.DEFAULT_REQUEST_TIMEOUT_SECONDS + 1.0;
	static inline var PRESENCE_TIMEOUT:Float = 5.0;
	static inline var CYCLE_TIMEOUT:Float = 180.0;
	static inline var MOTION_TIMEOUT:Float = 60.0;

	/** Generate complete cycles for distinct tray slot indices in row-major order. */
	public static function generate(cell:TendingControllers, slots:Array<Int>):Array<SceneArtifactMissionStep> {
		if (cell == null || slots == null || slots.length == 0) throw "Tending mission needs a cell and at least one infeed slot";
		var slotCount = cell.infeedTray.rows * cell.infeedTray.columns;
		var seen = new Map<Int, Bool>();
		for (slot in slots) {
			if (slot < 0 || slot >= slotCount || seen.exists(slot)) throw 'Invalid or repeated tending slot $slot';
			seen.set(slot, true);
		}
		var result:Array<SceneArtifactMissionStep> = [];
		function signal(name:String, value:Bool):Int {
			var index = result.length;
			result.push({kind: "signal", signal: {name: name, value: value}});
			return index;
		}
		function wait(name:String, value:Bool, timeout:Float, retryStep:Int = -1):Void {
			var condition:materia.project.SceneArtifact.SceneArtifactMissionWait = {name: name,
				value: value, timeout: timeout};
			if (retryStep >= 0) condition.retryStep = retryStep;
			result.push({kind: "waitFor", waitFor: condition});
		}
		function move(names:Array<String>, tool:String):Void {
			var waypoints:Array<SceneArtifactMissionWaypoint> = [];
			for (name in names) waypoints.push(jointWaypoint(cell, name, tool));
			result.push({kind: "moveVia", at: {occurrence: toolTcp(tool), connector: "tcp"}, poses: waypoints,
				timeoutSeconds: MOTION_TIMEOUT});
		}
		function moveTray(station:String, tool:String):Void
			move([station + "-clearance", station + "-approach"], tool);
		function moveToVise(tool:String):Void
			move(["outside door", "doorway center", "inside doorway", "inside door", "vise-approach"], tool);
		function moveFromVise(tool:String):Void
			move(["vise-approach", "inside door", "inside doorway", "doorway center", "outside door", "outside staging"], tool);
		function pick(occurrence:String, station:String, tool:String):Void result.push({kind: "pick",
			at: {occurrence: occurrence, connector: "grasp"}, tool: tool,
			timeoutSeconds: MOTION_TIMEOUT, approachDistance: cell.reach.handlingApproachMm / 1000,
			toolYaw: TendingReach.toolYaw(cell.reach, station, tool)});
		function place(target:materia.project.SceneArtifact.SceneArtifactPlace, station:String, tool:String):Void
			result.push({kind: "place", at: target, tool: tool, timeoutSeconds: MOTION_TIMEOUT,
				approachDistance: cell.reach.handlingApproachMm / 1000,
				toolYaw: TendingReach.toolYaw(cell.reach, station, tool)});
		function rowOf(slot:Int):Int return Std.int(slot / cell.infeedTray.columns);
		function columnOf(slot:Int):Int return slot % cell.infeedTray.columns;
		function blankOf(slot:Int):String return 'infeed/blank-${rowOf(slot)}-${columnOf(slot)}';
		function waitInfeed(slot:Int):Void wait('infeed/slot-${rowOf(slot)}-${columnOf(slot)}-present', true, PRESENCE_TIMEOUT);
		function openDoor():Void {
			var request = signal(RobotInterfaceChannels.REQUEST_DOOR_OPEN, true);
			wait(RobotInterfaceChannels.DOOR_OPEN, true, CNC_SIGNAL_TIMEOUT, request);
			signal(RobotInterfaceChannels.REQUEST_DOOR_OPEN, false);
		}
		function closeDoor():Void {
			var request = signal(RobotInterfaceChannels.REQUEST_DOOR_CLOSE, true);
			wait(RobotInterfaceChannels.DOOR_CLOSED, true, CNC_SIGNAL_TIMEOUT, request);
			signal(RobotInterfaceChannels.REQUEST_DOOR_CLOSE, false);
		}
		function clampPart():Void {
			var request = signal(RobotInterfaceChannels.REQUEST_CLAMP, true);
			wait(RobotInterfaceChannels.CLAMPED, true, CNC_SIGNAL_TIMEOUT, request);
			wait(RobotInterfaceChannels.PART_PRESENT, true, CNC_SIGNAL_TIMEOUT, request);
			signal(RobotInterfaceChannels.REQUEST_CLAMP, false);
		}
		function startCycle():Int {
			var request = signal(RobotInterfaceChannels.CYCLE_START, true);
			wait(RobotInterfaceChannels.IN_CYCLE, true, CNC_SIGNAL_TIMEOUT, request);
			signal(RobotInterfaceChannels.CYCLE_START, false);
			return request;
		}
		var toolA = DualArmGripperTool.TOOL_CHANNEL_A;
		var toolB = DualArmGripperTool.TOOL_CHANNEL_B;
		function toolFor(cycle:Int):String return cycle % 2 == 0 ? toolA : toolB;
		function otherTool(tool:String):String return tool == toolA ? toolB : toolA;

		// Load the first blank, then cycle with the next blank staged outside the
		// door. While each following cut runs, clear the prior finish and prepare
		// the next blank on the same free gripper.
		var firstSlot = slots[0], firstTool = toolFor(0);
		waitInfeed(firstSlot);
		moveTray('infeed-${rowOf(firstSlot)}-${columnOf(firstSlot)}', firstTool);
		pick(blankOf(firstSlot), 'infeed-${rowOf(firstSlot)}-${columnOf(firstSlot)}', firstTool);
		// Stay at the fully exterior staging pose while the door is closed. The
		// robot only approaches the aperture after the CNC controller opens it.
		move(['infeed-${rowOf(firstSlot)}-${columnOf(firstSlot)}-clearance', "outside staging"], firstTool);
		signal(RobotInterfaceChannels.ROBOT_CLEAR, true);
		openDoor();
		signal(RobotInterfaceChannels.ROBOT_CLEAR, false);
		moveToVise(firstTool);
		place(cell.viseDatumPlace(), "vise", firstTool);
		clampPart();
		moveFromVise(firstTool);
		signal(RobotInterfaceChannels.ROBOT_CLEAR, true);
		closeDoor();
		var cycleStartIndices:Array<Int> = [startCycle()];

		for (cycle in 0...slots.length) {
			var currentTool = toolFor(cycle), stagingTool = otherTool(currentTool);
			if (cycle > 0) {
				var finishedSlot = slots[cycle - 1];
				moveTray('outfeed-${rowOf(finishedSlot)}-${columnOf(finishedSlot)}', stagingTool);
				place(cell.outfeedPlace(rowOf(finishedSlot), columnOf(finishedSlot)),
					'outfeed-${rowOf(finishedSlot)}-${columnOf(finishedSlot)}', stagingTool);
			}
			if (cycle + 1 < slots.length) {
				var nextSlot = slots[cycle + 1];
				waitInfeed(nextSlot);
				moveTray('infeed-${rowOf(nextSlot)}-${columnOf(nextSlot)}', stagingTool);
				pick(blankOf(nextSlot), 'infeed-${rowOf(nextSlot)}-${columnOf(nextSlot)}', stagingTool);
				move(['infeed-${rowOf(nextSlot)}-${columnOf(nextSlot)}-clearance', "outside staging"], stagingTool);
				signal(RobotInterfaceChannels.ROBOT_CLEAR, true);
			}
			var startIndex = cycleStartIndices[cycle];
			wait(RobotInterfaceChannels.CYCLE_COMPLETE, true, CYCLE_TIMEOUT, startIndex);
			wait(RobotInterfaceChannels.CNC_READY, true, CNC_SIGNAL_TIMEOUT);
			openDoor();
			signal(RobotInterfaceChannels.ROBOT_CLEAR, false);
			moveToVise(currentTool);
			signal(RobotInterfaceChannels.REQUEST_UNCLAMP, true);
			wait(RobotInterfaceChannels.UNCLAMPED, true, CNC_SIGNAL_TIMEOUT);
			signal(RobotInterfaceChannels.REQUEST_UNCLAMP, false);
			var finishedBlank = blankOf(slots[cycle]);
			pick(finishedBlank, "vise", currentTool);
			if (cycle + 1 < slots.length) {
				move(["vise-approach"], stagingTool);
				place(cell.viseDatumPlace(), "vise", stagingTool);
				clampPart();
				moveFromVise(stagingTool);
				signal(RobotInterfaceChannels.ROBOT_CLEAR, true);
				closeDoor();
				cycleStartIndices.push(startCycle());
			} else {
				moveFromVise(currentTool);
				signal(RobotInterfaceChannels.ROBOT_CLEAR, true);
				closeDoor();
				moveTray('outfeed-${rowOf(slots[cycle])}-${columnOf(slots[cycle])}', currentTool);
				place(cell.outfeedPlace(rowOf(slots[cycle]), columnOf(slots[cycle])),
					'outfeed-${rowOf(slots[cycle])}-${columnOf(slots[cycle])}', currentTool);
			}
		}
		return result;
	}

	static function jointWaypoint(cell:TendingControllers, name:String, tool:String):SceneArtifactMissionWaypoint {
		var position = TendingReach.toolPose(cell.reach, name, tool);
		if (position.length != cell.arm.modules.length) throw 'Tending reach pose "$name" has the wrong joint count';
		return jointWaypointAt(cell, position);
	}

	static function toolTcp(tool:String):String {
		if (tool == DualArmGripperTool.TOOL_CHANNEL_A) return "arm/" + DualArmGripperTool.GRIPPER_A_BODY;
		if (tool == DualArmGripperTool.TOOL_CHANNEL_B) return "arm/" + DualArmGripperTool.GRIPPER_B_BODY;
		throw 'Tending mission does not know gripper channel "$tool"';
	}

	static function jointWaypointAt(cell:TendingControllers, position:Array<Float>):SceneArtifactMissionWaypoint {
		if (position.length != cell.arm.modules.length) throw "Tending reach waypoint has the wrong joint count";
		var targets:Array<SceneArtifactJointTarget> = [];
		for (joint in 0...cell.arm.modules.length)
			targets.push({joint: 'arm/j${joint + 1}', position: position[joint]});
		return {joints: targets};
	}
}
