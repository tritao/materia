package machinekit.robotics;

import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.SolenoidValve;

/** Pneumatic two-jaw tool for a cobot flange. The jaws and cylinder remain real
 * assembly members; the controller exposes one close command and the simulator
 * pulses the cylinder valve's two digital coil outputs. */
class ArmGripperTool implements ArmTool {
	public static inline var TCP_PART:String = "tool/gripper/body";
	public final spec:ParallelGripperSpec;

	public function new(size:Int = 32, expectedPartWidth:Float = 60, fingerLength:Float = 60,
			openingTravel:Float = 16, supportStep:Bool = false) {
		this.spec = new ParallelGripperSpec(size, expectedPartWidth, fingerLength, openingTravel,
			supportStep);
	}

	public function expose(arm:MachineAssembly):Void {
		arm.exposeConnector("toolContact", "tool/gripper/body", "tcp");
		arm.exposePort("compressedAir", "tool/valve", "P");
	}

	public function build(flange:RobotFlange):EndEffector {
		var gripper = new ParallelGripper(spec);
		// Port A is cylinder extension (jaw closure). A double valve rests at B,
		// leaving the tool open until its controller requests a grasp.
		var valve = new SolenoidValve(true, false);
		var result = new EndEffector();
		result.addComponent("plate", new EndEffectorPlate(flange));
		result.include("gripper", gripper);
		result.addComponent("valve", valve);
		result.addMate("gripper-mount", "fixed", "plate", "tool", "gripper/body", "mount");
		result.addMate("valve-mount", "fixed", "gripper/body", "valveSeat", "valve", "mount");
		var poses = result.solvedPoses();
		for (outlet in ["A", "B"])
			GripperAirRouting.add(result, poses, "gripper", "valve", spec, outlet, "airHose" + outlet);
		result.exposePort("close", "gripper/body", "close");
		result.exposePort("open", "gripper/body", "open");
		result.exposePort("compressedAir", "valve", "P");
		result.addCylinder("jaw-cylinder", "gripper/jaw-left-slide", "gripper/cylinder", "valve");
		result.workingFrame("contact", "gripper/body", "tcp", true);
		return result;
	}

}
