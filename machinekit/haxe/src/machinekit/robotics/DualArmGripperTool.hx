package machinekit.robotics;

import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.PneumaticManifold;
import machinekit.pneumatic.SolenoidValve;
import materia.assembly.AssemblyFrames;

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

	public final spec:ParallelGripperSpec;
	/** B can handle a different commissioned part width from A. */
	public final specB:ParallelGripperSpec;
	/** Half-spacing of opposing stations on the adapter's tool bolt circle. */
	public final mountOffsetMm:Float;
	/** Actual thickness of the flange adapter plate, including the assumed wrist stand-off. */
	public final adapterThicknessMm:Float;
	/** Conservative projected envelopes including the cylinder and routed air lines, in mm. */
	public final envelopeWidthMm:Float;
	public final envelopeDepthMm:Float;

	public function new(size:Int = 32, expectedPartWidth:Float = 60, fingerLength:Float = 60,
			openingTravel:Float = 16, supportStep:Bool = false, ?fingerDepth:Float, ?fingerWidth:Float,
			?adapterThicknessMm:Float, ?secondSpec:ParallelGripperSpec) {
		spec = new ParallelGripperSpec(size, expectedPartWidth, fingerLength, openingTravel,
			supportStep, fingerDepth, fingerWidth);
		specB = secondSpec == null ? spec : secondSpec;
		this.adapterThicknessMm = adapterThicknessMm == null ? 12.0 : adapterThicknessMm;
		if (!Math.isFinite(this.adapterThicknessMm) || this.adapterThicknessMm <= 0)
			throw "Dual gripper adapter thickness must be finite and positive";
		var valve = new SolenoidValve(true, false);
		var inwardReach = 0.0, crossAxisReach = 0.0;
		for (grip in [spec, specB]) {
			inwardReach = Math.max(inwardReach, Math.max(grip.openJawCentre() + grip.jawWidth / 2,
				Math.max(grip.bodyWidth / 2, grip.valveSeatX() + valve.bodyWidth / 2)));
			crossAxisReach = Math.max(crossAxisReach, grip.supportSideReach());
		}
		// The grippers sit opposite one another on one diameter and turn 90 degrees.
		// Their inward X projections clear by the assumed edge gap.
		mountOffsetMm = (inwardReach + crossAxisReach + INTER_GRIPPER_CLEARANCE_MM) / 2;
		var hoseSideEnvelope = Math.max(GripperAirRouting.sideReach(spec), GripperAirRouting.sideReach(specB));
		var jawAxisMin = 0.0, jawAxisMax = 0.0;
		for (grip in [spec, specB]) {
			jawAxisMin = Math.min(jawAxisMin, grip.cylinderStartX());
			jawAxisMax = Math.max(jawAxisMax, Math.max(grip.bodyWidth / 2, Math.max(grip.openJawCentre() + grip.jawWidth / 2,
				grip.valveSeatX() + valve.bodyWidth / 2)));
		}
		var sideMin = -hoseSideEnvelope;
		var sideMax = Math.max(spec.supportSideReach(), specB.supportSideReach());
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
		var plate = new EndEffectorPlate(flange, adapterThicknessMm, 2 * mountOffsetMm, 4);
		result.addComponent("plate", plate);
		var seatA = plate.connector("toolBolt3").frame;
		var seatB = AssemblyFrames.compose(plate.connector("toolBolt1").frame,
			{x: 0, y: 0, z: 0, qx: 0, qy: Math.sin(Math.PI / 4), qz: 0, qw: Math.cos(Math.PI / 4)});
		result.addMemberConnector("plate", "gripperASeat", seatA);
		result.addMemberConnector("plate", "gripperBSeat", seatB);
		result.include("gripperA", new ParallelGripper(spec));
		result.include("gripperB", new ParallelGripper(specB));
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
		var poses = result.solvedPoses();
		for (name in ["A", "B"]) {
			var valve = name == "A" ? "valveA" : "valveB";
			var gripper = name == "A" ? "gripperA" : "gripperB";
			for (outlet in ["A", "B"]) GripperAirRouting.add(result, poses, gripper, valve,
				name == "A" ? spec : specB, outlet, 'airHose$valve$outlet');
			result.addCylinder('jaw-cylinder-$name', gripper + "/jaw-left-slide", gripper + "/cylinder", valve);
		}
		result.exposePort("compressedAir", "manifold", "input");
		result.workingFrame("contactA", "gripperA/body", "tcp", true);
		result.workingFrame("contactB", "gripperB/body", "tcp");
		return result;
	}

}
