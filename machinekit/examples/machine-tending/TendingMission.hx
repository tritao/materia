import cnckit.controller.RobotInterfaceChannels;
import cnckit.controller.RobotInterface;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
import materia.project.SceneArtifact.SceneArtifactMissionWaypoint;
import materia.project.SceneArtifact.SceneArtifactJointTarget;
import machinekit.robotics.DualArmGripperTool;
import TendingCyclePlan.TendingOperation;

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

	/** Resolve the production sequence into frames, outputs and certified paths. */
	public static function generate(cell:TendingControllers):Array<SceneArtifactMissionStep> {
		if (cell == null) throw "Tending mission needs its configured cell";
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
		function movePath(positions:Array<Array<Float>>, tool:String):Void {
			result.push({kind: "moveVia", at: {occurrence: toolTcp(tool), connector: "tcp"},
				poses: [for (position in positions) jointWaypointAt(cell, position)], timeoutSeconds: MOTION_TIMEOUT});
		}
		function moveToVise(tool:String, station:String = "vise"):Void
			movePath(TendingReach.transferPositions(cell.reach, station, tool, tool).slice(1), tool);
		function moveFromVise(tool:String, station:String = "vise"):Void {
			var path = TendingReach.transferPositions(cell.reach, station, tool, tool);
			path.reverse();
			movePath(path, tool);
		}
		function pick(occurrence:String, station:String, tool:String):Void result.push({kind: "pick",
			at: {occurrence: occurrence, connector: "grasp"}, tool: tool,
			timeoutSeconds: MOTION_TIMEOUT, approachDistance: cell.reach.handlingApproachMm / 1000,
			toolYaw: TendingReach.toolYaw(cell.reach, station, tool)});
		function place(target:materia.project.SceneArtifact.SceneArtifactPlace, station:String, tool:String):Void
			result.push({kind: "place", at: target, tool: tool, timeoutSeconds: MOTION_TIMEOUT,
				approachDistance: cell.reach.handlingApproachMm / 1000,
				toolYaw: TendingReach.toolYaw(cell.reach, station, tool)});
		function rowOf(slot:Int):Int return cell.production.rowOf(slot);
		function columnOf(slot:Int):Int return cell.production.columnOf(slot);
		function blankOf(slot:Int):String return 'infeed/blank-${rowOf(slot)}-${columnOf(slot)}';
		function waitInfeed(slot:Int):Void wait('infeed/slot-${rowOf(slot)}-${columnOf(slot)}-present', true, PRESENCE_TIMEOUT);
		function openDoor():Void {
			var request = signal(RobotInterfaceChannels.REQUEST_DOOR_OPEN, true);
			wait(RobotInterfaceChannels.DOOR_OPEN, true, CNC_SIGNAL_TIMEOUT, request);
			signal(RobotInterfaceChannels.REQUEST_DOOR_OPEN, false);
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
		function collect(slot:Int, tool:String, fromTool:String, initial:Bool = false):Void {
			var station = 'infeed-${rowOf(slot)}-${columnOf(slot)}';
			var transport = TendingReach.transferPositions(cell.reach, station, tool, fromTool);
			var approach = TendingReach.transferPositions(cell.reach, TendingCyclePlan.approachRoute(station), tool, tool);
			waitInfeed(slot);
			var incoming = initial ? TendingReach.transferPositions(cell.reach, TendingCyclePlan.startupRoute(station), tool, tool) : transport;
			movePath(incoming.slice(1).concat(approach.slice(1)), tool);
			pick(blankOf(slot), station, tool);
			approach.reverse();
			transport.reverse();
			movePath(approach.slice(1).concat(transport.slice(1)), fromTool);
		}
		function load(tool:String, ?fromTool:String):Void {
			if (fromTool != null) {
				movePath(TendingReach.transferPositions(cell.reach, "exchange", tool, fromTool).slice(1), tool);
			}
			else moveToVise(tool, "vise-load");
			place(cell.viseLoadPlace(), "vise-load", tool);
			clampPart();
		}
		var cycleRequest = -1, doorCloseRequest = -1;
		for (operation in cell.production.operations()) switch operation {
			case Collect(slot, tool, fromTool, initial): collect(slot, tool, fromTool, initial);
			case OpenDoor: openDoor();
			case RequestDoorClose:
				if (doorCloseRequest >= 0) throw "Tending production repeats an unfinished door-close request";
				doorCloseRequest = signal(RobotInterfaceChannels.REQUEST_DOOR_CLOSE, true);
			case DoorClosed:
				if (doorCloseRequest < 0) throw "Tending production waits without a door-close request";
				wait(RobotInterfaceChannels.DOOR_CLOSED, true, CNC_SIGNAL_TIMEOUT, doorCloseRequest);
				signal(RobotInterfaceChannels.REQUEST_DOOR_CLOSE, false);
				doorCloseRequest = -1;
			case Unclamp: unclampPart();
			case Load(tool, fromTool): load(tool, fromTool);
			case LeaveVise(tool): moveFromVise(tool, "vise-load");
			case StartCycle: cycleRequest = startCycle();
			case CycleReady:
				if (cycleRequest < 0) throw "Tending production waits without a cycle request";
				wait(RobotInterfaceChannels.CYCLE_COMPLETE, true, CYCLE_TIMEOUT, cycleRequest);
				wait(RobotInterfaceChannels.CNC_READY, true, CNC_SIGNAL_TIMEOUT);
			case Unload(slot, tool):
				moveToVise(tool);
				unclampPart();
				pick(blankOf(slot), "vise", tool);
			case Retreat(tool): moveFromVise(tool);
			case Deliver(slot, tool, fromTool, returnToStage):
				var station = 'outfeed-${rowOf(slot)}-${columnOf(slot)}';
				var outfeed = TendingReach.transferPositions(cell.reach, station, tool, fromTool);
				var approach = TendingReach.transferPositions(cell.reach, TendingCyclePlan.approachRoute(station), tool, tool);
				movePath(outfeed.slice(1).concat(approach.slice(1)), tool);
				place(cell.outfeedPlace(rowOf(slot), columnOf(slot)), station, tool);
				if (returnToStage) { approach.reverse(); outfeed.reverse(); movePath(approach.slice(1).concat(outfeed.slice(1)), fromTool); }
		}
		return result;
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
