package app;

import materia.project.SceneArtifact.SceneArtifactMissionStep;
import cnckit.controller.RobotInterfaceChannels;
import cnckit.controller.RobotInterfaceAlarm;

/** Maps a failed mission step and its retained physical state to an operator alarm. */
class MissionAlarmClassifier {
	/** Convert the CNC PLC's discrete fault code into the alarm shown for the failed mission. */
	public static function fromInterfaceAlarm(alarm:RobotInterfaceAlarm, partPresent:Bool):MissionAlarmCode return switch alarm {
		case RobotInterfaceAlarm.DoorOpenTimeout | RobotInterfaceAlarm.DoorCloseTimeout |
			RobotInterfaceAlarm.DoorOpenedDuringCycle:
			MissionAlarmCode.DoorJam;
		case RobotInterfaceAlarm.ClampInterlock:
			partPresent ? MissionAlarmCode.CncAlarm : MissionAlarmCode.ClampWithoutPart;
		case RobotInterfaceAlarm.ClampTimeout:
			partPresent ? MissionAlarmCode.ViseFailure : MissionAlarmCode.ClampWithoutPart;
		case RobotInterfaceAlarm.UnclampTimeout:
			MissionAlarmCode.ViseFailure;
		case RobotInterfaceAlarm.None | RobotInterfaceAlarm.ConflictingRequests |
			RobotInterfaceAlarm.DoorOpenInterlock | RobotInterfaceAlarm.DoorCloseInterlock |
			RobotInterfaceAlarm.UnclampInterlock | RobotInterfaceAlarm.CycleInterlock |
			RobotInterfaceAlarm.ToolChangeRequired:
			MissionAlarmCode.CncAlarm;
		case _:
			MissionAlarmCode.CncAlarm;
	};

	public static function classify(step:SceneArtifactMissionStep, message:String,
		emergencyStopped:Bool, carryingPart:Bool):MissionAlarmCode {
		if (emergencyStopped) return MissionAlarmCode.EmergencyStop;
		if (step.kind == "waitFor" && step.waitFor != null) {
			var name = step.waitFor.name;
			if (StringTools.startsWith(name, "infeed/")) return MissionAlarmCode.EmptyInfeed;
			switch name {
				case RobotInterfaceChannels.DOOR_OPEN | RobotInterfaceChannels.DOOR_CLOSED:
					return MissionAlarmCode.DoorJam;
				case RobotInterfaceChannels.CLAMPED:
					return MissionAlarmCode.ViseFailure;
				case RobotInterfaceChannels.PART_PRESENT:
					return MissionAlarmCode.ClampWithoutPart;
				case RobotInterfaceChannels.CNC_READY | RobotInterfaceChannels.IN_CYCLE |
					RobotInterfaceChannels.CYCLE_COMPLETE | RobotInterfaceChannels.ALARM:
					return MissionAlarmCode.CncAlarm;
				default:
			}
		}
		if (step.kind == "pick" || message.indexOf("did not confirm a grasp") >= 0)
			return MissionAlarmCode.MissedGrasp;
		if (carryingPart) return MissionAlarmCode.PartDropped;
		return MissionAlarmCode.MotionFailure;
	}
}
