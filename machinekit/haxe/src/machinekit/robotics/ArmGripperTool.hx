package machinekit.robotics;

import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.PneumaticCylinder;
import machinekit.pneumatic.RoutedHose;
import machinekit.pneumatic.SolenoidValve;
import machinekit.component.PortKind;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Pneumatic two-jaw tool for a cobot flange. The jaws and cylinder remain real
 * assembly members; the controller exposes one close command and the simulator
 * pulses the cylinder valve's two digital coil outputs. */
class ArmGripperTool implements ArmTool {
	public static inline var TCP_PART:String = "tool/gripper/body";
	static inline var HOSE_OUTER_DIAMETER_MM:Float = 6.0;
	static inline var HOSE_CLEARANCE_MM:Float = 5.0;
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
		var cylinder:PneumaticCylinder = spec.cylinder();
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
		addAirHose(result, poses, valve, cylinder, "A");
		addAirHose(result, poses, valve, cylinder, "B");
		result.exposePort("close", "gripper/body", "close");
		result.exposePort("open", "gripper/body", "open");
		result.exposePort("compressedAir", "valve", "P");
		result.addCylinder("jaw-cylinder", "gripper/jaw-left-slide", "gripper/cylinder", "valve");
		result.workingFrame("contact", "gripper/body", "tcp", true);
		return result;
	}

	function addAirHose(result:EndEffector, poses:Map<String, AssemblyFrame>, valve:SolenoidValve,
			cylinder:PneumaticCylinder, outlet:String):Void {
		var valvePose = poses.get("valve");
		var cylinderPose = poses.get("gripper/cylinder");
		if (valvePose == null || cylinderPose == null) throw "Gripper air line has no solved valve or cylinder pose";
		var valveConnector = valve.connector("port" + outlet).frame;
		var cylinderConnector = cylinder.connector("port" + outlet).frame;
		var start = AssemblyFrames.compose(valvePose, valveConnector);
		var target = AssemblyFrames.compose(cylinderPose, cylinderConnector);
		var startPort = Solids.axial(0, 0, 0);
		var hosePose = AssemblyFrames.compose(start, AssemblyFrames.inverse(startPort));
		var localTarget = AssemblyFrames.transformPoint(AssemblyFrames.inverse(hosePose), target.x, target.y, target.z);
		// Route around the cylinder's swept envelope instead of through the moving
		// rod. The clearance allowance is an assumed installation gap.
		var sideOffset = cylinder.bodyWidth / 2 + HOSE_OUTER_DIAMETER_MM / 2 + HOSE_CLEARANCE_MM;
		var route = [new Vector(0, 0, 0), new Vector(localTarget.x / 2, localTarget.y + sideOffset,
			localTarget.z / 2), new Vector(localTarget.x, localTarget.y, localTarget.z)];
		var id = "airHose" + outlet;
		var hose = new RoutedHose('GRIPPER-6MM-$outlet', route, HOSE_OUTER_DIAMETER_MM, 4, 0.03,
			PortKind.Pneumatic);
		result.addComponent(id, hose);
		result.addMate('hose-$outlet-valve', "fixed", "valve", "port" + outlet, id, "start");
		result.connectPorts('valve-hose-$outlet', "valve", outlet, id, "input");
		result.connectPorts('hose-cylinder-$outlet', id, "output", "gripper/cylinder", outlet);
	}
}
