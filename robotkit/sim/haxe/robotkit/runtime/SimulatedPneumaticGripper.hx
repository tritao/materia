package robotkit.runtime;

import RobotKitRuntime;
import haxe.Int64;
import robotkit.execution.ProcessEventValue;

/** Bridges the logical gripper command to short pulses on the bound cylinder's
 * two digital valve coils. The physical jaws, workpiece contact and joint switch
 * remain in the simulation; this class only implements the valve interface. */
class SimulatedPneumaticGripper implements SimulationStepObserver {
	public static inline var VALVE_PULSE_NS:Int = 40_000_000;

	public final runtime:RobotRuntime;
	public final commandChannel:String;
	public final extendChannel:String;
	public final retractChannel:String;
	public final graspSensor:String;

	var lastCommand:Bool = false;
	var pulseChannel:Null<String>;
	var pulseEnds:Null<Int64>;

	public function new(runtime:RobotRuntime, blueprint:RobotRuntimeBlueprint, commandChannel:String,
			graspSensor:String) {
		if (runtime == null || blueprint == null || commandChannel == null || commandChannel.length == 0 ||
			graspSensor == null || graspSensor.length == 0)
			throw "Simulated pneumatic gripper needs a runtime, logical command and jaw switch";
		var sensor = blueprint.sensorById(graspSensor);
		if (sensor == null || sensor.kind != "joint_switch" || sensor.joint < 0)
			throw 'Gripper feedback "$graspSensor" must be a joint switch';
		var drives = [for (drive in blueprint.pneumaticDrives) if (drive.joint == sensor.joint) drive];
		if (drives.length != 1 || drives[0].channelB == null)
			throw 'Gripper switch "$graspSensor" needs one double-solenoid cylinder on its jaw joint';
		this.runtime = runtime;
		this.commandChannel = commandChannel;
		this.extendChannel = drives[0].channelA;
		this.retractChannel = drives[0].channelB;
		this.graspSensor = graspSensor;
	}

	public function afterSimulationStep(sourceTimestampNs:Int64):Void {
		var close = switch runtime.channelValue(commandChannel) {
			case Digital(value): value;
			case _: throw 'Gripper command "$commandChannel" is not digital';
		};
		var safety = runtime.snapshot().safety;
		var stopped = safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP ||
			safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT;
		if (stopped) {
			deenergize();
			lastCommand = close;
			return;
		}
		if (close != lastCommand) {
			deenergize();
			pulseChannel = close ? extendChannel : retractChannel;
			runtime.writeDigitalOutput(pulseChannel, true);
			pulseEnds = Int64.add(sourceTimestampNs, Int64.ofInt(VALVE_PULSE_NS));
			lastCommand = close;
		}
		var end = pulseEnds;
		if (end != null && Int64.compare(sourceTimestampNs, end) >= 0) deenergize();
	}

	/** Reset both outputs to the double valve's de-energized, latched state. */
	public function reset():Void {
		deenergize();
		lastCommand = false;
	}

	function deenergize():Void {
		if (pulseChannel != null) runtime.writeDigitalOutput(pulseChannel, false);
		pulseChannel = null;
		pulseEnds = null;
	}
}
