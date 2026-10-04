import machinekit.assembly.MachineAssembly;
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

/**
 * A fixed MIG welding cell: the six-axis arm on its pedestal carrying a torch, a wire feeder on
 * its upper arm, the power source and the shielding gas cylinder on the floor behind it, and a
 * welding table in front with the workpiece held on it. The arm is included as `arm` and works
 * the table in front of it (-Y).
 *
 * Services reach the torch the way they do on a real cell. The cylinder feeds the power source,
 * which feeds the feeder with weld current, gas and control; the feeder feeds the torch with those
 * and the wire. The power source's `mains` inlet and its `control` input are the cell's own: they
 * go to the wall and to the robot's controller.
 *
 * The workpiece (`work`, a `WeldingWorkpiece`) is a plate T-joint with a small tube frame beside it. It
 * sits on the table at `WORK_X`, `WORK_Y`, the plate's long edges held by two fixture bars. Its seams are
 * found from its geometry (`weldment()`), not placed here.
 */
class WeldingCell extends MachineAssembly {
	/** The table is the arm's own work table, as tall as its cell's. */
	public static inline var TABLE_TOP:Float = RobotArm.TABLE_TOP;
	/** The table stands in front of the arm (-Y), and the weldment sits on its centre line, in the arm's reach. */
	public static inline var TABLE_CENTRE_Y:Float = -600;
	public static inline var WORK_Y:Float = -480;
	/** The workpiece's origin, the plate's centre, on the table. The plate and the tube frame beside it are both in the arm's reach. */
	public static inline var WORK_X:Float = -155;
	static inline var FIXTURE_SIZE:Float = 20;
	/** The work clamp's seat on the plate's top, in the plate's frame: a back corner, clear of the seams. */
	public static inline var CLAMP_X:Float = -60;
	public static inline var CLAMP_Y:Float = 62;

	public final arm = new RobotArm(false, new ArmWeldingTool());
	public final feeder = new WireFeeder(150, 240, 180);
	public final source = new WeldingPowerSource();
	public final cylinder = new GasCylinder();
	public final clamp = new WorkClamp();
	public final work:WeldingWorkpiece;

	public function new(?workpiece:WeldingWorkpiece) {
		super();
		work = workpiece == null ? new WeldingWorkpiece() : workpiece;
		include("arm", arm);
		// The feeder rides on the upper arm, on the side the tube's centre line faces, and moves with it.
		var tube = 40.0;
		addMemberConnector("arm/upperArm", "feederSeat", AssemblyFrames.alongY(tube, 0, 160, 1, 0, 0));
		addComponent("feeder", feeder);
		addMate("feeder-mate", "fixed", "arm/upperArm", "feederSeat", "feeder", "mount");

		function at(x:Float, y:Float, z:Float):AssemblyFrame return AssemblyFrames.translation(x, y, z);
		addComponent("source", source, at(-1000, 100, 0));
		addComponent("cylinder", cylinder, at(-1000, 500, 0));

		addComponent("table", new ArmTable(900, 500, TABLE_TOP), at(0, TABLE_CENTRE_Y, 0));
		// The workpiece sits at the arm's work position, and its fixtures hold the plate's long edges.
		var seatY = WORK_Y - TABLE_CENTRE_Y;
		addMemberConnector("table", "weldmentSeat", Solids.axial(WORK_X, seatY, TABLE_TOP));
		var stop = WeldingWorkpiece.PLATE_WIDTH / 2 + FIXTURE_SIZE / 2;
		addMemberConnector("table", "fixtureNearSeat", Solids.axial(WORK_X, seatY - stop, TABLE_TOP));
		addMemberConnector("table", "fixtureFarSeat", Solids.axial(WORK_X, seatY + stop, TABLE_TOP));
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
		connectPorts("torch-power", "feeder", "torchPower", "arm/tool/torch", "power");
		connectPorts("torch-gas", "feeder", "torchGas", "arm/tool/torch", "gas");
		connectPorts("torch-wire", "feeder", "torchWire", "arm/tool/torch", "wire");
		connectPorts("torch-control", "feeder", "torchControl", "arm/tool/torch", "control");
		addBomItem(line("HOSEPACK-ROBOT-1.5M", "Torch hose pack, 1.5 m: power, gas, wire liner and control"));
		// The power source takes the wall and the robot's controller.
		exposePort("mains", "source", "mains");
		exposePort("control", "source", "control");
	}

	/** Lift the torch clear of the table before approaching the work. */
	public function readyPose():Array<Float> {
		var shoulder = 0.25, elbow = -1.4;
		return [0.0, shoulder, elbow, 0.0, Math.PI - shoulder + elbow, 0.0];
	}

	/** The welded joints, for the members of `work` as they are named in the cell. */
	public function weldment():Weldment return work.weldment().prefixed("work/");

	/**
	 * The cell's welding equipment as its connections give it: the supply's limits, the wire, and the work
	 * the circuit returns through (the plate the clamp sits on and everything welded to it).
	 */
	public function equipment():WeldingEquipmentData return WeldingEquipment.of(this, [weldment()]);
}
