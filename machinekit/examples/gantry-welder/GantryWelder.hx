import RobotArm.ArmTable;
import RobotArm.ArmBlock;
import machinekit.component.BomItem;
import machinekit.component.Solids;
import machinekit.welding.GasCylinder;
import machinekit.welding.WeldingEquipment;
import machinekit.welding.WeldingEquipment.WeldingEquipmentData;
import machinekit.welding.WeldingPowerSource;
import machinekit.welding.Weldment;
import machinekit.welding.WireFeeder;
import machinekit.welding.WorkClamp;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Screw-driven XYZ gantry with a CA head and the shared plate/tube-frame weldment. */
class GantryWelder extends machinekit.gantry.Gantry {
	/** Shared table height, mm. */
	public static inline var TABLE_TOP:Float = 800;
	/** The forward torch reach shifts the useful tool workspace ahead of the Y carriage travel. */
	public static inline var TABLE_CENTRE_Y:Float = 200;
	public static inline var WORK_Y:Float = 200;
	/** The workpiece's origin, the plate's centre, on the table. The plate and the tube frame beside it are both inside the XYZ workspace. */
	public static inline var WORK_X:Float = 450;
	static inline var FIXTURE_SIZE:Float = 20;
	/** The work clamp's seat on the plate's top, in the plate's frame: a back corner, clear of the seams. */
	public static inline var CLAMP_X:Float = -60;
	public static inline var CLAMP_Y:Float = 62;

	public final tool:machinekit.robotics.EndEffector;
	public final feeder = new WireFeeder(150, 240, 180);
	public final source = new WeldingPowerSource();
	public final cylinder = new GasCylinder();
	public final clamp = new WorkClamp();
	public final work:WeldingWorkpiece;

	public function new(?workpiece:WeldingWorkpiece) {
		// Reorienting the bent torch makes XYZ counter its long lever arm.
		// The 10 mm ball-screw lead and 48 V drivers supply that carriage
		// speed while the wire stays above the welder's stable feed range.
		var screw = machinekit.gantry.GantrySpec.GantryDrive.Screw(new machinekit.motion.LeadScrewThread(
			machinekit.motion.LeadScrewThread.LeadScrewThreadFamily.Ball, 16, 10));
		super(new machinekit.gantry.GantrySpec(1200, 1000, 600, screw, screw, screw, true,
			"MGN12C", 23, "HFS5-4040", "HFS5-4040", false, machinekit.gantry.GantrySpec.GantryHead.CA,
			0.5, 48, 16, 200, 100, 500, 250));
		var head:machinekit.gantry.RotaryHead = cast this.head;
		tool = new ArmWeldingTool().build(head.flange);
		include("tool", tool);
		var flange = connector("toolFlange");
		addMate("torch-mount", "fixed", flange.instanceId, flange.connectorName, "tool/plate", "robot");
		exposeConnector("toolTcp", "tool/torch", "tcp");
		work = workpiece == null ? new WeldingWorkpiece() : workpiece;
		function at(x:Float, y:Float, z:Float):AssemblyFrame return AssemblyFrames.translation(x, y, z);
		addComponent("source", source, at(-800, 100, 0));
		addComponent("feeder", feeder, at(-700, 300, 0));
		addComponent("cylinder", cylinder, at(-800, 500, 0));

		addComponent("table", new ArmTable(900, 500, TABLE_TOP), at(600, TABLE_CENTRE_Y, 0));
		// The workpiece sits inside the gantry workspace, and its fixtures hold the plate's long edges.
		var seatY = WORK_Y - TABLE_CENTRE_Y;
		addMemberConnector("table", "weldmentSeat", Solids.axial(WORK_X - 600, seatY, TABLE_TOP));
		var stop = WeldingWorkpiece.PLATE_WIDTH / 2 + FIXTURE_SIZE / 2;
		addMemberConnector("table", "fixtureNearSeat", Solids.axial(WORK_X - 600, seatY - stop, TABLE_TOP));
		addMemberConnector("table", "fixtureFarSeat", Solids.axial(WORK_X - 600, seatY + stop, TABLE_TOP));
		var fixture = new ArmBlock(WeldingWorkpiece.PLATE_LENGTH, FIXTURE_SIZE, FIXTURE_SIZE, "steel", "Fixture");
		addComponent("fixtureNear", fixture);
		addComponent("fixtureFar", fixture);
		addMate("fixture-near-mate", "fixed", "table", "fixtureNearSeat", "fixtureNear", "base");
		addMate("fixture-far-mate", "fixed", "table", "fixtureFarSeat", "fixtureFar", "base");
		include("work", work);
		addMate("work-mate", "fixed", "table", "weldmentSeat", "work/basePlate", "base");
		// The work clamp sits on the plate's top, in a corner away from the seams; the work lead ends there.
		addMemberConnector("work/basePlate", "clampSeat", Solids.axial(CLAMP_X, CLAMP_Y, WeldingWorkpiece.PLATE_THICKNESS));
		addComponent("clamp", clamp);
		addMate("clamp-mate", "fixed", "work/basePlate", "clampSeat", "clamp", "contact");

		function line(partNumber:String, description:String):BomItem
			return {partNumber: partNumber, description: description, quantity: 1, material: null};
		connectPorts("gas-hose", "cylinder", "gas", "source", "gas", line("HOSE-GAS-3M", "Shielding gas hose, 3 m"));
		connectPorts("weld-cable", "source", "weldPositive", "feeder", "power", line("CABLE-WELD-35-3M", "Weld cable 35 mm², 3 m"));
		connectPorts("feeder-gas", "source", "gasOut", "feeder", "gas", line("HOSE-GAS-3M", "Shielding gas hose, 3 m"));
		connectPorts("feeder-control", "source", "feederControl", "feeder", "control",
			line("CABLE-CONTROL-12-3M", "Welder control cable, 12 core, 3 m"));
		connectPorts("work-lead", "source", "weldNegative", "clamp", "lead", line("CABLE-WORK-35-3M", "Work lead 35 mm², 3 m"));
		connectPorts("torch-power", "feeder", "torchPower", "tool/torch", "power");
		connectPorts("torch-gas", "feeder", "torchGas", "tool/torch", "gas");
		connectPorts("torch-wire", "feeder", "torchWire", "tool/torch", "wire");
		connectPorts("torch-control", "feeder", "torchControl", "tool/torch", "control");
		addBomItem(line("HOSEPACK-GANTRY-3M", "Torch hose pack, 3 m: power, gas, wire liner and control"));
		// The power source takes mains power and the gantry controller.
		exposePort("mains", "source", "mains");
		exposePort("control", "source", "control");
	}

	/** The welded joints, for the members of `work` as they are named in the cell. */
	public function weldment():Weldment return work.weldment().prefixed("work/");

	/**
	 * The cell's welding equipment as its connections give it: the supply's limits, the wire, and the work
	 * the circuit returns through (the plate the clamp sits on and everything welded to it).
	 */
	public function equipment():WeldingEquipmentData return WeldingEquipment.of(this, [weldment()]);
}
