package processkit.pressbrake;

import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;

/** Press-side process outputs. Ram travel is millimetres downward from measured top. */
class PressBrakeChannels {
	public static inline final READY = "press.ready";
	public static inline final BACKGAUGE_IN_POSITION = "press.backgauge_in_position";
	public static inline final AT_TOP = "press.at_top";
	public static inline final CLAMPING = "press.clamping";
	public static inline final AT_BOTTOM = "press.at_bottom";
	public static inline final DECOMPRESSED = "press.decompressed";
	public static inline final ALARM = "press.alarm";
	public static inline final ALARM_CODE = "press.alarm_code";
	public static inline final RAM_POSITION_MM = "press.ram_position_mm";

	public static function declarations():Array<ProcessChannelDeclaration> {
		var result = [for (id in [READY, BACKGAUGE_IN_POSITION, AT_TOP, CLAMPING, AT_BOTTOM, DECOMPRESSED])
			new ProcessChannelDeclaration(id, Digital(false), true)];
		// A stopped controller reports a runtime alarm through the native safe-output image.
		result.push(new ProcessChannelDeclaration(ALARM, Digital(true), true));
		result.push(new ProcessChannelDeclaration(ALARM_CODE, Analog(alarmCode(RuntimeFailure)), true));
		result.push(new ProcessChannelDeclaration(RAM_POSITION_MM, Analog(0), true));
		return result;
	}

	/** Explicit protocol codes; enum declaration order is not a wire convention. */
	public static function alarmCode(alarm:PressBrakeAlarm):Int return switch alarm {
		case None: 0;
		case ConflictingRequests: 1;
		case Busy: 2;
		case StrokeInterlock: 3;
		case OpenInterlock: 4;
		case SafetyInterlock: 5;
		case InvalidFeedback: 6;
		case ProgramChanged: 7;
		case RegistrationLost: 8;
		case RobotHoldLost: 9;
		case PositioningLost: 10;
		case ClampTimeout: 11;
		case WithdrawalTimeout: 12;
		case DepthTimeout: 13;
		case DecompressionTimeout: 14;
		case OpenTimeout: 15;
		case PositionTimeout: 16;
		case AxisMotionFailure: 17;
		case InvalidProgramStep: 18;
		case RuntimeFailure: 19;
		case EventOverflow: 20;
	};
}
