package machinekit.robotics;

import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.PneumaticCylinder;
import machinekit.pneumatic.PneumaticManifold;
import machinekit.pneumatic.RoutedHose;
import machinekit.pneumatic.SolenoidValve;
import machinekit.component.PortKind;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Two independently valved parallel grippers mounted at right angles on one wrist plate. */
class DualArmGripperTool implements ArmTool {
	public static inline var VALVE_COUNT:Int = 2;
	public static inline var VALVE_COIL_OUTPUTS:Int = VALVE_COUNT * 2;
	public static inline var GRIPPER_A_BODY:String = "tool/gripperA/body";
	public static inline var GRIPPER_B_BODY:String = "tool/gripperB/body";
	public static inline var TOOL_CHANNEL_A:String = "arm/tool/gripperA/body.close";
	public static inline var TOOL_CHANNEL_B:String = "arm/tool/gripperB/body.close";
	/** Assumed free edge clearance between the two separately valved gripper envelopes. */
	public static inline var INTER_GRIPPER_CLEARANCE_MM:Float = 15.0;
	static inline var HOSE_OUTER_DIAMETER_MM:Float = 6.0;
	static inline var HOSE_CLEARANCE_MM:Float = 5.0;

	public final spec:ParallelGripperSpec;
	/** Half-spacing of opposing stations on the adapter's tool bolt circle. */
	public final mountOffsetMm:Float;
	/** Conservative projected envelopes including the cylinder and routed air lines, in mm. */
	public final envelopeWidthMm:Float;
	public final envelopeDepthMm:Float;

	public function new(size:Int = 32, expectedPartWidth:Float = 60, fingerLength:Float = 60,
			openingTravel:Float = 16, supportStep:Bool = false, ?fingerDepth:Float, ?fingerWidth:Float) {
		spec = new ParallelGripperSpec(size, expectedPartWidth, fingerLength, openingTravel,
			supportStep, fingerDepth, fingerWidth);
		var valve = new SolenoidValve(true, false);
		var inwardReach = Math.max(spec.openJawCentre() + spec.jawWidth / 2,
			Math.max(spec.bodyWidth / 2, spec.valveSeatX() + valve.bodyWidth / 2));
		var crossAxisReach = Math.max(spec.bodyDepth / 2,
			Math.max(spec.fingerDepth / 2, valve.bodyDepth / 2));
		// The grippers sit opposite one another on one diameter and turn 90 degrees.
		// Their inward X projections clear by the assumed edge gap.
		mountOffsetMm = (inwardReach + crossAxisReach + INTER_GRIPPER_CLEARANCE_MM) / 2;
		var hoseSideEnvelope = spec.bodyWidth / 2 + HOSE_OUTER_DIAMETER_MM + HOSE_CLEARANCE_MM;
		var cylinder = spec.cylinder();
		var jawAxisMin = -spec.openJawCentre() - cylinder.bodyLength - cylinder.stemLength;
		var jawAxisMax = Math.max(spec.bodyWidth / 2, Math.max(spec.openJawCentre() + spec.jawWidth / 2,
			spec.valveSeatX() + valve.bodyWidth / 2));
		var sideMin = -hoseSideEnvelope;
		var sideMax = Math.max(spec.bodyDepth / 2, spec.fingerDepth / 2);
		// A sits at -X with its cylinder outward; B sits at +X turned 90°,
		// with its hose branch outward. Union their projected part envelopes.
		envelopeWidthMm = 2 * mountOffsetMm + hoseSideEnvelope - jawAxisMin;
		envelopeDepthMm = Math.max(sideMax, jawAxisMax) - Math.min(sideMin, jawAxisMin);
	}

	public function expose(arm:MachineAssembly):Void {
		arm.exposeConnector("toolContactA", "tool/gripperA/body", "tcp");
		arm.exposeConnector("toolContactB", "tool/gripperB/body", "tcp");
		arm.exposePort("compressedAir", "tool/manifold", "input");
	}

	public function build(flange:RobotFlange):EndEffector {
		var result = new EndEffector();
		// The opposing tool-bolt holes are also the two gripper mount stations.
		var plate = new EndEffectorPlate(flange, 12, 2 * mountOffsetMm, 4);
		result.addComponent("plate", plate);
		var seatA = plate.connector("toolBolt3").frame;
		var seatB = AssemblyFrames.compose(plate.connector("toolBolt1").frame,
			{x: 0, y: 0, z: 0, qx: 0, qy: Math.sin(Math.PI / 4), qz: 0, qw: Math.cos(Math.PI / 4)});
		result.addMemberConnector("plate", "gripperASeat", seatA);
		result.addMemberConnector("plate", "gripperBSeat", seatB);
		result.include("gripperA", new ParallelGripper(spec));
		result.include("gripperB", new ParallelGripper(spec));
		// Each body's logical open/close inputs are supplied at the tool boundary,
		// just as they are for the single-gripper tool. The runtime binds these
		// independent controls to the corresponding valve coils.
		result.exposePort("gripperA/open", "gripperA/body", "open");
		result.exposePort("gripperA/close", "gripperA/body", "close");
		result.exposePort("gripperB/open", "gripperB/body", "open");
		result.exposePort("gripperB/close", "gripperB/body", "close");
		result.addMate("gripper-a-mount", "fixed", "plate", "gripperASeat", "gripperA/body", "mount");
		result.addMate("gripper-b-mount", "fixed", "plate", "gripperBSeat", "gripperB/body", "mount");
		result.addComponent("valveA", new SolenoidValve(true, false));
		result.addComponent("valveB", new SolenoidValve(true, false));
		result.addMate("valve-a-mount", "fixed", "gripperA/body", "valveSeat", "valveA", "mount");
		result.addMate("valve-b-mount", "fixed", "gripperB/body", "valveSeat", "valveB", "mount");
		result.addComponent("manifold", new PneumaticManifold(2), AssemblyFrames.translation(0, 0, 24));
		result.addMate("manifold-mount", "fixed", "plate", "tool", "manifold", "mount");
		result.connectPorts("manifold-valve-a", "manifold", "out1", "valveA", "P");
		result.connectPorts("manifold-valve-b", "manifold", "out2", "valveB", "P");
		var cylinderA:PneumaticCylinder = spec.cylinder();
		var cylinderB:PneumaticCylinder = spec.cylinder();
		var poses = result.solvedPoses();
		for (name in ["A", "B"]) {
			var valve = name == "A" ? "valveA" : "valveB";
			var gripper = name == "A" ? "gripperA" : "gripperB";
			var cylinder = name == "A" ? cylinderA : cylinderB;
			for (outlet in ["A", "B"]) addAirHose(result, poses, valve, gripper + "/cylinder", cylinder, outlet);
			result.addCylinder('jaw-cylinder-$name', gripper + "/jaw-left-slide", gripper + "/cylinder", valve);
		}
		result.exposePort("compressedAir", "manifold", "input");
		result.workingFrame("contactA", "gripperA/body", "tcp", true);
		result.workingFrame("contactB", "gripperB/body", "tcp");
		return result;
	}

	function addAirHose(result:EndEffector, poses:Map<String, AssemblyFrame>, valve:String,
			cylinderMember:String, cylinder:PneumaticCylinder, outlet:String):Void {
		var valvePose = poses.get(valve), cylinderPose = poses.get(cylinderMember);
		if (valvePose == null || cylinderPose == null) throw "Dual gripper air line has no solved valve or cylinder";
		var start = AssemblyFrames.compose(valvePose, result.memberConnectorFrame(valve, "port" + outlet));
		var target = AssemblyFrames.compose(cylinderPose, cylinder.connector("port" + outlet).frame);
		var hosePose = AssemblyFrames.compose(start, AssemblyFrames.inverse(Solids.axial(0, 0, 0)));
		var localTarget = AssemblyFrames.transformPoint(AssemblyFrames.inverse(hosePose), target.x, target.y, target.z);
		var sideOffset = spec.bodyWidth / 2 + HOSE_OUTER_DIAMETER_MM / 2 + HOSE_CLEARANCE_MM;
		// Each branch loops around the outside of its own gripper. The two valves
		// share the wrist area, so routing both loops inward would cross the other
		// gripper's fingers and service envelope.
		var route = [new Vector(0, 0, 0), new Vector(localTarget.x / 2, localTarget.y - sideOffset,
			localTarget.z / 2), new Vector(localTarget.x, localTarget.y, localTarget.z)];
		var hoseId = 'airHose$valve$outlet';
		result.addComponent(hoseId, new RoutedHose('GRIPPER-6MM-$valve-$outlet', route,
			HOSE_OUTER_DIAMETER_MM, 4, 0.03, PortKind.Pneumatic));
		result.addMate('$hoseId-valve', "fixed", valve, "port" + outlet, hoseId, "start");
		result.connectPorts('$valve-$outlet-hose', valve, outlet, hoseId, "input");
		result.connectPorts('$hoseId-cylinder', hoseId, "output", cylinderMember, outlet);
	}
}
