package robotkit.runtime;

import RobotKitRuntime;
import haxe.Int64;

/** Bridges the logical gripper command to short pulses on the bound cylinder's
 * two digital valve coils. The physical jaws, workpiece contact and joint switch
 * remain in the simulation. Contact observations report the free workpiece
 * between both jaws; they never attach it kinematically. */
class SimulatedPneumaticGripper implements SimulationStepObserver {
	public static inline var VALVE_PULSE_NS:Int = 40_000_000;

	public final runtime:RobotRuntime;
	public final commandChannel:String;
	public final extendChannel:String;
	public final retractChannel:String;
	public final graspSensor:String;
	/** Jaw links derived from the cylinder joint and its mirrored follower. */
	public final contactLinks:Array<Int>;

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
		var drives = [for (drive in blueprint.cylinderDrives) if (drive.control == CylinderControl.PressureEffort && drive.joint == sensor.joint) drive];
		if (drives.length != 1 || drives[0].channelB == null)
			throw 'Gripper switch "$graspSensor" needs one double-solenoid cylinder on its jaw joint';
		this.runtime = runtime;
		this.commandChannel = commandChannel;
		this.extendChannel = drives[0].channelA;
		this.retractChannel = drives[0].channelB;
		this.graspSensor = graspSensor;
		contactLinks = [blueprint.joints[sensor.joint].childLink];
		for (coupling in blueprint.couplings) if (coupling.leader == sensor.joint)
			contactLinks.push(blueprint.joints[coupling.follower].childLink);
		if (contactLinks.length != 2 || contactLinks[0] == contactLinks[1])
			throw 'Parallel gripper "$commandChannel" needs two distinct, mechanically coupled jaw links';
	}

	/** Free objects supported by both jaws with confirmed width feedback, without a kinematic attachment. */
	public function grippedObjects(contacts:Array<RobotContact>):Array<Int> {
		var confirmed = false;
		for (frame in runtime.snapshot().sensors.toArray())
			if (frame.sensorId == graspSensor && frame.values.length > 0 && frame.values.get(0) >= 0.5) confirmed = true;
		if (!confirmed) return [];
		var left:Array<Int> = [], right:Array<Int> = [];
		for (contact in contacts) if (contact.active && contact.otherKind == RobotContactOtherKind.Object) {
			var pad:Null<Array<Int>> = contact.linkIndex == contactLinks[0] ? left : contact.linkIndex == contactLinks[1] ? right : null;
			if (pad != null && pad.indexOf(contact.otherObject) < 0) pad.push(contact.otherObject);
		}
		return [for (object in left) if (right.indexOf(object) >= 0) object];
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
			// Safety clears logical outputs without changing the latched valve.
			// Observe that image so readiness alone cannot issue an opening pulse.
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
