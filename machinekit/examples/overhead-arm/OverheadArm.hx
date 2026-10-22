import RobotArm.ArmTable;
import RobotArm.ArmBlock;
import machinekit.gantry.Gantry;
import machinekit.gantry.GantrySpec;
import machinekit.gantry.GantrySpec.GantryDrive;
import machinekit.gantry.GantryParts.GantryBoredBracket;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** G18: a complete industrial arm suspended from an XYZ gantry's Z carriage.
 * Layout and mounting dimensions are authored assumptions, in millimetres.
 * Guides, rack/screw ratios and drive limits come from the physical recipes. */
class OverheadArm extends Gantry {
	public static inline var TABLE_TOP:Float = 800;
	public static inline var TRAVEL_X:Float = 3000;
	public static inline var TRAVEL_Y:Float = 1500;
	public final arm:RobotArm;
	public final workY:Float;
	public final ready:Array<Float>;

	public function new() {
		var rack:GantryDrive = Rack(2, 20, 20);
		super(new GantrySpec(TRAVEL_X, TRAVEL_Y, 300, rack, rack, null, false,
			"HGH15CA", 34, "HFS5-4040", "HFS5-4040", false, None, 0.5,
			new machinekit.power.DcVoltage(48), 16, 800, 200, 1400, 650));
		arm = new RobotArm(false, null, machinekit.robotics.IndustrialArmClass.Reach900, true);
		var flange:machinekit.robotics.RobotFlange = cast component("flange");
		attach("armAdapter", new GantryBoredBracket("overhead arm adapter", 360, 360, 30,
			AssemblyFrames.identity(), flange.pilotDiameter + 0.5), flangeZero, "flange");
		addMemberConnector("armAdapter", "arm", Solids.axial(0, 0, 30));
		include("arm", arm);
		var floor = arm.connector("floor");
		addMate("arm-mount", "fixed", "armAdapter", "arm", "arm/" + floor.instanceId, floor.connectorName);
		for (name in arm.portNames()) {
			var port = arm.port(name, "arm");
			exposePort("arm/" + name, port.instanceId, port.portName);
		}
		ready = [for (joint in arm.specs) joint.initial];
		workY = flangeZero.y - 550;
		addComponent("table", new ArmTable(TRAVEL_X + 200, TRAVEL_Y + 200, TABLE_TOP),
			AssemblyFrames.translation(TRAVEL_X / 2, workY + TRAVEL_Y / 2, 0));
		var pad = new ArmBlock(80, 80, RobotArm.PAD_HEIGHT, "rubber", "overhead station");
		addComponent("padPick", pad, AssemblyFrames.translation(200, workY, TABLE_TOP));
		addComponent("padPlace", pad, AssemblyFrames.translation(2800, workY + TRAVEL_Y, TABLE_TOP));
		addComponent("workpiece", new ArmBlock(RobotArm.WORKPIECE_WIDTH, RobotArm.WORKPIECE_WIDTH,
			RobotArm.WORKPIECE_HEIGHT, "birch plywood", "overhead workpiece"),
			AssemblyFrames.translation(200, workY, TABLE_TOP + RobotArm.PAD_HEIGHT));
	}
}
