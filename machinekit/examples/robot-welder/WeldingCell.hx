import machinekit.assembly.MachineAssembly;
import machinekit.component.BomItem;
import machinekit.component.Solids;
import machinekit.welding.GasCylinder;
import machinekit.welding.WeldingPowerSource;
import machinekit.welding.WireFeeder;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A point on a seam in the cell's frame, in millimetres. */
typedef SeamPoint = {x:Float, y:Float, z:Float};

/** The two ends of one fillet seam, and the face of the upright it runs along. */
typedef PlaceholderSeam = {id:String, start:SeamPoint, stop:SeamPoint};

/**
 * A fixed MIG welding cell: the six-axis arm on its pedestal carrying a torch, a wire feeder on
 * its upper arm, the power source and the shielding gas cylinder on the floor behind it, and a
 * welding table in front with a plate T-joint held on it. The arm is included as `arm` and works
 * the table in front of it (-Y).
 *
 * Services reach the torch the way they do on a real cell. The cylinder feeds the power source,
 * which feeds the feeder with weld current, gas and control; the feeder feeds the torch with those
 * and the wire. The power source's `mains` inlet and its `control` input are the cell's own: they
 * go to the wall and to the robot's controller.
 *
 * The weldment is a placeholder for the seams of W1: a base plate and an upright standing on it, so
 * there are two fillet seams along the upright's long sides.
 */
class WeldingCell extends MachineAssembly {
	/** The table is the arm's own work table, as tall as its cell's. */
	public static inline var TABLE_TOP:Float = RobotArm.TABLE_TOP;
	/** The table stands in front of the arm (-Y), and the weldment sits on its centre line, in the arm's reach. */
	public static inline var TABLE_CENTRE_Y:Float = -600;
	public static inline var WORK_Y:Float = -480;
	/** Base plate and upright, in millimetres. The upright stands on the plate's centre line. */
	public static inline var PLATE_LENGTH:Float = 300;
	public static inline var PLATE_WIDTH:Float = 150;
	public static inline var PLATE_THICKNESS:Float = 10;
	public static inline var UPRIGHT_THICKNESS:Float = 8;
	public static inline var UPRIGHT_HEIGHT:Float = 80;
	static inline var FIXTURE_SIZE:Float = 20;

	public final arm = new RobotArm(false, new ArmWeldingTool());
	public final feeder = new WireFeeder(150, 240, 180);
	public final source = new WeldingPowerSource();
	public final cylinder = new GasCylinder();

	public function new() {
		super();
		include("arm", arm);
		// The feeder rides on the upper arm, on the side the tube's centre line faces, and moves with it.
		var tube = 40.0;
		addMemberConnector("arm/upperArm", "feederSeat", AssemblyFrames.alongY(tube, 0, 160, 1, 0, 0));
		addComponent("feeder", feeder);
		addMate("feeder-mate", "fixed", "arm/upperArm", "feederSeat", "feeder", "mount");

		function at(x:Float, y:Float, z:Float):AssemblyFrame return AssemblyFrames.translation(x, y, z);
		addComponent("source", source, at(-1000, 100, 0));
		addComponent("cylinder", cylinder, at(-1000, 500, 0));

		addComponent("table", new ArmTable(800, 500, TABLE_TOP), at(0, TABLE_CENTRE_Y, 0));
		// The weldment sits at the arm's work position, and its fixtures hold the plate's long edges.
		var seatY = WORK_Y - TABLE_CENTRE_Y;
		addMemberConnector("table", "weldmentSeat", Solids.axial(0, seatY, TABLE_TOP));
		var stop = PLATE_WIDTH / 2 + FIXTURE_SIZE / 2;
		addMemberConnector("table", "fixtureNearSeat", Solids.axial(0, seatY - stop, TABLE_TOP));
		addMemberConnector("table", "fixtureFarSeat", Solids.axial(0, seatY + stop, TABLE_TOP));
		var fixture = new ArmBlock(PLATE_LENGTH, FIXTURE_SIZE, FIXTURE_SIZE, "steel", "Fixture");
		addComponent("fixtureNear", fixture);
		addComponent("fixtureFar", fixture);
		addMate("fixture-near-mate", "fixed", "table", "fixtureNearSeat", "fixtureNear", "base");
		addMate("fixture-far-mate", "fixed", "table", "fixtureFarSeat", "fixtureFar", "base");
		addComponent("basePlate", new ArmBlock(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS, "steel", "Base plate"));
		addMate("base-plate-mate", "fixed", "table", "weldmentSeat", "basePlate", "base");
		addComponent("upright", new ArmBlock(PLATE_LENGTH, UPRIGHT_THICKNESS, UPRIGHT_HEIGHT, "steel", "Upright"));
		addMate("upright-mate", "fixed", "basePlate", "top", "upright", "base");

		function line(partNumber:String, description:String):BomItem
			return {partNumber: partNumber, description: description, quantity: 1, material: null};
		connectPorts("gas-hose", "cylinder", "gas", "source", "gas", line("HOSE-GAS-3M", "Shielding gas hose, 3 m"));
		connectPorts("weld-cable", "source", "weldPositive", "feeder", "power", line("CABLE-WELD-35-3M", "Weld cable 35 mm², 3 m"));
		connectPorts("feeder-gas", "source", "gasOut", "feeder", "gas", line("HOSE-GAS-3M", "Shielding gas hose, 3 m"));
		connectPorts("feeder-control", "source", "feederControl", "feeder", "control",
			line("CABLE-CONTROL-12-3M", "Welder control cable, 12 core, 3 m"));
		connectPorts("torch-power", "feeder", "torchPower", "arm/tool/torch", "power");
		connectPorts("torch-gas", "feeder", "torchGas", "arm/tool/torch", "gas");
		connectPorts("torch-wire", "feeder", "torchWire", "arm/tool/torch", "wire");
		connectPorts("torch-control", "feeder", "torchControl", "arm/tool/torch", "control");
		addBomItem(line("HOSEPACK-ROBOT-1.5M", "Torch hose pack, 1.5 m: power, gas, wire liner and control"));
		// The power source takes the wall and the robot's controller.
		exposePort("mains", "source", "mains");
		exposePort("control", "source", "control");
	}

	/** The two fillet seams, along the upright's long sides where it meets the base plate's top. */
	public static function seams():Array<PlaceholderSeam> {
		var z = TABLE_TOP + PLATE_THICKNESS;
		var half = PLATE_LENGTH / 2, offset = UPRIGHT_THICKNESS / 2;
		return [for (side in [-1, 1]) {
			var y = WORK_Y + side * offset;
			{id: side < 0 ? "near" : "far", start: {x: -half, y: y, z: z}, stop: {x: half, y: y, z: z}};
		}];
	}
}
