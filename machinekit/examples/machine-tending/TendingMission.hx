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
	// Main CollisionKit's conservative joint-space path through the door and vise
	// is bounded by this per-step simulation-time budget, including planning and settling.
	static inline var MOTION_TIMEOUT:Float = 90.0;

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
		function moveToVise(tool:String, station:String = "vise"):Void
			move(TendingReach.viseTransitStations(station).slice(1), tool);
		function moveFromVise(tool:String, station:String = "vise"):Void {
			var path = TendingReach.viseTransitStations(station);
			path.reverse();
			move(path, tool);
		}
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
		function unclampPart():Void {
			var request = signal(RobotInterfaceChannels.REQUEST_UNCLAMP, true);
			wait(RobotInterfaceChannels.UNCLAMPED, true, CNC_SIGNAL_TIMEOUT, request);
			signal(RobotInterfaceChannels.REQUEST_UNCLAMP, false);
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
		// MT10–MT11 handles one blank at a time. Pre-staging and a two-part
		// exchange are MT12 throughput work, after this cell's recovery gate.
		for (cycle in 0...slots.length) {
			var slot = slots[cycle], tool = toolFor(cycle);
			var station = 'infeed-${rowOf(slot)}-${columnOf(slot)}';
			waitInfeed(slot);
			moveTray(station, tool);
			pick(blankOf(slot), station, tool);
			move([station + "-clearance", "outside staging"], tool);
			signal(RobotInterfaceChannels.ROBOT_CLEAR, true);
			openDoor();
			unclampPart();
			signal(RobotInterfaceChannels.ROBOT_CLEAR, false);
			moveToVise(tool, "vise-load");
			place(cell.viseLoadPlace(), "vise-load", tool);
			clampPart();
			moveFromVise(tool, "vise-load");
			signal(RobotInterfaceChannels.ROBOT_CLEAR, true);
			closeDoor();
			var cycleRequest = startCycle();
			wait(RobotInterfaceChannels.CYCLE_COMPLETE, true, CYCLE_TIMEOUT, cycleRequest);
			wait(RobotInterfaceChannels.CNC_READY, true, CNC_SIGNAL_TIMEOUT);
			openDoor();
			signal(RobotInterfaceChannels.ROBOT_CLEAR, false);
			moveToVise(tool);
			unclampPart();
			pick(blankOf(slot), "vise", tool);
			moveFromVise(tool);
			signal(RobotInterfaceChannels.ROBOT_CLEAR, true);
			closeDoor();
			var outfeedStation = 'outfeed-${rowOf(slot)}-${columnOf(slot)}';
			moveTray(outfeedStation, tool);
			place(cell.outfeedPlace(rowOf(slot), columnOf(slot)), outfeedStation, tool);
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
