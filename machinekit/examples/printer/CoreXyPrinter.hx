import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.AxisBuilder;
import machinekit.assembly.Sense.SenseTools;
import machinekit.assembly.Transmission;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.gantry.GantryParts.GantryExtrusion;
import machinekit.gantry.GantryParts.GantryIdlerPin;
import machinekit.gantry.GantryParts.GantryPlate;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LimitSwitch;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.MotorDriver;
import machinekit.motion.NemaStepper;
import machinekit.motion.PowerSupply;
import machinekit.motion.ProximitySwitch;
import machinekit.printing.Extruder;
import machinekit.printing.PrintedPart;
import machinekit.printing.Fan;
import machinekit.printing.HeatedBed;
import machinekit.printing.HeaterCartridge;
import machinekit.printing.Hotend;
import machinekit.printing.Nozzle;
import machinekit.printing.Thermistor;
import machinekit.structural.TSlotExtrusion;
import machinekit.transmission.SpurGear;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingBelt.BeltWrap;
import machinekit.transmission.TimingPulley;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * A CoreXY FDM printer: a 2020 T-slot frame, a gantry on two MGN12 rails along Y carrying an MGN12 rail
 * along X with the toolhead, two GT2 belts closing through the carriage so that two stepper motors move
 * the head in X and Y, and a bed that moves up and down on three T8 lead screws, each with its own motor.
 *
 * Joints `x`, `y` and `z` are prismatic and read in machine coordinates, in millimetres, as a printer's
 * G-code does: x and y are the nozzle's distance from the bed's front left corner (the bed is 220 mm
 * square, so 0 to 220 is the bed), and z is the distance of the bed below the nozzle tip: 0 has them
 * touching, 220 is the whole build height. The assembly frame has the floor at z = 0 and the middle of
 * the bed over the origin.
 *
 * Like the CoreXY plotter, the belts are drawn at the joints' zero, and each pulley follows x and y
 * through the strands of its belt, so a motor's pulley sums both axes. The extruder is a revolute joint
 * `e`: the angle of the hobbed gear that grips the filament, driven by a pancake stepper through a 3:1 gear
 * pair. The rest of the toolhead is a hotend, a nozzle, a heater cartridge and a thermistor, a blower
 * for the part and a fan for the heatsink; the bed has its own heater and thermistor.
 *
 * Homing is by switches: X and Y by roller switches that the carriage and the gantry press, Z by an
 * inductive probe on the toolhead that senses the bed; Z homes after X and Y.
 */
class CoreXyPrinter extends AxisBuilder {
	public static inline var PROFILE:String = "HFS5-2020";
	public static inline var RAIL:String = "MGN12C";
	public static inline var CARRIAGE_RAIL:String = "MGN12H";
	public static inline var Z_RAIL:String = "MGN9C";
	/** The bed's travel in X and Y, and its machine zero: the nozzle at the bed's front left corner. */
	public static inline var BED:Float = 220;
	public static inline var MACHINE_ZERO_X:Float = -110;
	public static inline var MACHINE_ZERO_Y:Float = -110;
	/** Height of the nozzle tip above the floor, and the bed's travel below it. */
	public static inline var NOZZLE_Z:Float = 400;
	public static inline var Z_TRAVEL:Float = 220;
	/** The nozzle hangs this far in front of the gantry's middle line. */
	public static inline var TOOL_OFFSET_Y:Float = 30;
	/** Frame: Y members at x = ±RAIL_X, X members at the front and rear, between y = -170 and 230. */
	public static inline var RAIL_X:Float = 198;
	public static inline var FRONT_Y:Float = -160;
	public static inline var REAR_Y:Float = 220;
	public static inline var RING_TOP:Float = NOZZLE_Z + 48;
	/** GT2 belts, 20-tooth pulleys. */
	public static inline var TEETH:Int = 20;
	public static inline var BELT_WIDTH:Float = 6;
	public static inline var COLUMN_X:Float = 170;
	public static inline var INNER_X:Float = 145;
	/** Supply of the stepper drivers and the heaters. */
	public static inline var SUPPLY_VOLTS:Float = 24;
	public static inline var MICROSTEPS:Int = 16;
	public static inline var STEP_TICK_HZ:Int = 40000;
	/** What the stepper drivers are set to (amperes per phase): the belt and screw motors', and the extruder's. */
	public static inline var DRIVER_CURRENT:Float = 1.68;
	public static inline var EXTRUDER_DRIVER_CURRENT:Float = 1.0;
	/**
	 * Input shaping: the resonance (hertz) of each belt axis, the carriage on its belts for x and the gantry carrying it
	 * for y, and the damping ratio of a belt-driven carriage (the belts' own: they state it, and it rides into the
	 * simulation as the damping of their couplings, so a carriage rings there as it does in the shaper's model). The printing gate checks them against the drive model (the
	 * mass an axis moves on the stiffness of its belts) to ten percent. MZV is the shaper Klipper recommends for
	 * a printer whose resonance is known only to a few percent: it stays under 5 % vibration for 20 % either side.
	 */
	public static inline var SHAPER_X_HZ:Float = 53.4;
	public static inline var SHAPER_Y_HZ:Float = 34.4;
	public static inline var SHAPER_DAMPING:Float = 0.1;
	/** The bed's screws: a Tr8x2 thread of four starts, so one turn lifts the bed 8 mm. */
	public static function zThread():LeadScrewThread return new LeadScrewThread(MetricTrapezoidal, 8, 2, 4);
	/** Where the three bed screws stand. */
	public static final SCREWS:Array<{id:String, x:Float, y:Float}> = [
		{id: "screwZRearLeft", x: -140, y: 130}, {id: "screwZRearRight", x: 140, y: 130}, {id: "screwZFront", x: 0, y: -130}];

	public final motor:NemaStepper;
	public final specs:Array<PrinterAxisSpec> = [
		{id: "x", lower: 0, upper: BED, initial: BED / 2},
		{id: "y", lower: 0, upper: BED, initial: BED / 2},
		{id: "z", lower: 0, upper: Z_TRAVEL, initial: 10}
	];
	/** Pitch radius of the belt pulleys, in millimetres. */
	public final pulleyRadius:Float;
	public final hotend = new Hotend();
	public final nozzle = new Nozzle();
	public final extruder = new Extruder();
	public final bed = new HeatedBed();
	/** The hotend's heater cartridge and thermistor, and the bed's thermistor: the parts the print job's heaters are read from. */
	public final cartridge = new HeaterCartridge();
	public final hotendThermistor = new Thermistor();
	public final bedThermistor = new Thermistor();
	/** Pinion on the extruder motor and the gear on the hobbed shaft: a 3:1 reduction. */
	public final pinion = new SpurGear(0.5, 18, 6, SpurGear.STANDARD_PRESSURE_ANGLE, 0, 0, 5);
	public final bigGear = new SpurGear(0.5, 54, 6, SpurGear.STANDARD_PRESSURE_ANGLE, 0, 0, 5);
	/** Belt levels (pulley bottoms) of belts A and B, and the carriage's belt clamp. */
	public final levelA:Float;
	public final levelB:Float;
	var driverIndex:Int = 0;
	var supplyAdded:Bool = false;

	public function new() {
		super();
		motor = NemaStepper.frame(17);
		var shaft = motor.variant.shaftLength;
		pulleyRadius = TimingPulley.profileDimensions(GT2).pitch * TEETH / (2 * Math.PI);
		var R = pulleyRadius;
		var profile = TSlotExtrusion.forProfile(PROFILE);
		var up = [0.0, 0, 1], alongX = [1.0, 0, 0], alongY = [0.0, 1, 0], front = [0.0, -1, 0], vertical = [0.0, 0, 1];
		var railSpec = LinearRailBlock.metric(RAIL).spec;
		var zSpec = LinearRailBlock.metric(Z_RAIL).spec;
		var carriageSpec = LinearRailBlock.metric(CARRIAGE_RAIL).spec;
		var memberY0 = FRONT_Y - 10, memberLength = REAR_Y - FRONT_Y + 20;

		// ---- Frame: two rings and four posts, a base plate under the bed's motors. -------------------------------
		var ringBottom = 0.0, postLength = RING_TOP - 20 - 20;
		for (level in [{name: "Bottom", z: 0.0}, {name: "Top", z: RING_TOP - 20}]) {
			for (side in [-1, 1]) {
				var name = side < 0 ? "Left" : "Right";
				place('side${level.name}$name', new GantryExtrusion(profile, memberLength),
					AxisBuilder.orient(side * RAIL_X, memberY0, level.z + 10, up, alongY));
			}
			for (entry in [{name: "Front", y: FRONT_Y}, {name: "Rear", y: REAR_Y}])
				place('cross${level.name}${entry.name}', new GantryExtrusion(profile, 2 * (RAIL_X - 10)),
					AxisBuilder.orient(-(RAIL_X - 10), entry.y, level.z + 10, up, alongX));
		}
		for (x in [-1, 1]) for (entry in [{name: "Front", y: FRONT_Y}, {name: "Rear", y: REAR_Y}])
			place('post${x < 0 ? "Left" : "Right"}${entry.name}', new GantryExtrusion(profile, postLength),
				AxisBuilder.orient(x * RAIL_X, entry.y, 20, up, up));
		var baseDepth = REAR_Y - FRONT_Y - 20;
		place("basePlate", new GantryPlate("Base plate", 2 * (RAIL_X - 10), baseDepth, 6), AssemblyFrames.translation(0, (FRONT_Y + REAR_Y) / 2, 20));

		// ---- Y: the gantry on two rails on the top ring. ------------------------------------------------------------
		var railTop = RING_TOP + railSpec.railHeight;
		var railLength = 350.0, railStart = (FRONT_Y + REAR_Y) / 2 - railLength / 2;
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			place('railY$name', LinearRail.metric(RAIL, railLength), AxisBuilder.orient(side * RAIL_X, railStart, railTop, up, alongY));
		}
		var gy = MACHINE_ZERO_Y + TOOL_OFFSET_Y;
		var ySpec = specs[1], xSpec = specs[0], zAxis = specs[2];
		slide(ySpec, "railYLeft", "blockYLeft", LinearRailBlock.metric(RAIL), AxisBuilder.orient(-RAIL_X, gy, railTop, up, alongY), {x: 0, y: 1, z: 0});
		var blockTop = RING_TOP + railSpec.blockHeight;
		var bracketHeight = 6.0;
		attach("bracketLeft", new GantryPlate("Gantry bracket", 72, 36, bracketHeight), AssemblyFrames.translation(-RAIL_X + 15, gy, blockTop), "blockYLeft");
		var barHeight = 40.0, barBase = blockTop;
		var barLength = 2 * RAIL_X - 40;
		attach("crossbar", new GantryExtrusion(TSlotExtrusion.forProfile("HFS5-2040"), barLength),
			AxisBuilder.orient(-barLength / 2, gy + 20, barBase + barHeight / 2, up, alongX), "bracketLeft");
		attach("bracketRight", new GantryPlate("Gantry bracket", 72, 36, bracketHeight), AssemblyFrames.translation(RAIL_X - 15, gy, blockTop), "crossbar");
		attach("blockYRight", LinearRailBlock.metric(RAIL), AxisBuilder.orient(RAIL_X, gy, railTop, up, alongY), "bracketRight");

		// ---- The belts and their motors, as on the plotter: one belt each, on two levels, A above B. ---------------
		var motorPlateTop = RING_TOP + 6;
		var motorFace = motorPlateTop + motor.bodyLength;
		levelA = motorFace + shaft - BELT_WIDTH - 1;
		levelB = levelA - BELT_WIDTH - 1;
		var beltA = new TimingBelt(GT2, BELT_WIDTH, [
			new BeltWrap(-INNER_X, gy, R, 1), new BeltWrap(COLUMN_X - 2 * R, gy - 2 * R, R, -1),
			new BeltWrap(COLUMN_X, FRONT_Y, R, 1), new BeltWrap(COLUMN_X, REAR_Y, R, 1),
			new BeltWrap(-INNER_X, REAR_Y, R, 1)], null, 0.0, SHAPER_DAMPING);
		var beltB = new TimingBelt(GT2, BELT_WIDTH, [
			new BeltWrap(INNER_X, gy, R, -1), new BeltWrap(-(COLUMN_X - 2 * R), gy - 2 * R, R, 1),
			new BeltWrap(-COLUMN_X, FRONT_Y, R, -1), new BeltWrap(-COLUMN_X, REAR_Y, R, -1),
			new BeltWrap(INNER_X, REAR_Y, R, -1)], null, 0.0, SHAPER_DAMPING);
		attach("beltA", beltA, AssemblyFrames.translation(0, 0, levelA), "sideTopLeft");
		attach("beltB", beltB, AssemblyFrames.translation(0, 0, levelB), "sideTopLeft");
		for (entry in [{id: "A", x: COLUMN_X}, {id: "B", x: -COLUMN_X}]) {
			var plate = 'motorPlate${entry.id}';
			attach(plate, new GantryPlate("Motor plate", 60, 60, 6), AssemblyFrames.translation(entry.x, REAR_Y - 5, RING_TOP), "crossTopRear");
			attach('motor${entry.id}', entry.id == "A" ? motor : NemaStepper.frame(17), AxisBuilder.orient(entry.x, REAR_Y, motorFace, [0.0, 1, 0], up), plate);
		}
		var pinLength = levelA + BELT_WIDTH + 2 - RING_TOP;
		var wraps = [beltA.wraps(), beltB.wraps()];
		var beltIds = ["A", "B"], belts = [beltA, beltB], levels = [levelA, levelB];
		// Wrap 0 and 1 of each belt are the gantry's idlers, 2 the front idler, 3 the driving pulley, 4 the rear idler.
		var frameWraps = [2, 3, 4];
		var initialX = specs[0].initial, initialY = specs[1].initial;
		for (b in 0...2) {
			var letter = beltIds[b], belt = belts[b], level = levels[b], loop = wraps[b];
			for (w in frameWraps) {
				var wrap = loop[w];
				var id = w == 3 ? 'pulley$letter' : 'idler$letter${w == 2 ? "Front" : "Rear"}';
				var pose = AssemblyFrames.translation(wrap.x, wrap.y, level);
				var parent = 'motor$letter';
				if (w != 3) {
					parent = '${id}Pin';
					attach(parent, new GantryIdlerPin(motor.variant.shaftDiameter, pinLength), AssemblyFrames.translation(wrap.x, wrap.y, RING_TOP),
						w == 2 ? "crossTopFront" : "crossTopRear");
				}
				hang(id, beltPulley(), pose, parent);
				turnCoreXyPulley(id, parent, 'belt$letter', belt, wrap, true, w == 3, initialX, initialY);
			}
		}
		// Each stepper turns its belt's driving pulley.
		for (letter in beltIds) addMotor('motor$letter', 'pulley$letter-turn', 'motor$letter', addDriver('motor$letter'));
		// The gantry's brackets carry the idlers of both belts at that end.
		for (b in 0...2) {
			var letter = beltIds[b], loop = wraps[b], level = levels[b];
			for (w in 0...2) {
				var wrap = loop[w];
				var id = 'idler$letter${w == 0 ? "Start" : "End"}';
				var pin = '${id}Pin';
				var bracket = wrap.x < 0 ? "bracketLeft" : "bracketRight";
				attach(pin, new GantryIdlerPin(motor.variant.shaftDiameter, level + BELT_WIDTH + 2 - (blockTop + bracketHeight)),
					AssemblyFrames.translation(wrap.x, wrap.y, blockTop + bracketHeight), bracket);
				hang(id, beltPulley(), AssemblyFrames.translation(wrap.x, wrap.y, level), pin);
				turnCoreXyPulley(id, pin, 'belt$letter', belts[b], wrap, false, false, initialX, initialY);
			}
		}

		// ---- X: the carriage on a rail along the crossbar's front face. -----------------------------------------------
		var xc = MACHINE_ZERO_X;
		var railXFace = gy + 10 - railSpec.railHeight;
		var railXLength = barLength - 20, railZ = barBase + barHeight / 2;
		attach("railX", LinearRail.metric(RAIL, railXLength), AxisBuilder.orient(-railXLength / 2, railXFace, railZ, front, alongX), "crossbar");
		slide(xSpec, "railX", "blockX", LinearRailBlock.metric(CARRIAGE_RAIL), AxisBuilder.orient(xc, railXFace, railZ, front, alongX), {x: 1, y: 0, z: 0});
		var blockFront = railXFace - (carriageSpec.blockHeight - railSpec.railHeight);
		var plateBase = NOZZLE_Z + 35, plateHeight = levelB - 3 - plateBase;
		var plateY = blockFront - 3;
		attach("carriagePlate", new GantryPlate("Carriage plate", 30, 6, plateHeight), AssemblyFrames.translation(xc, plateY, plateBase), "blockX");
		var padBase = plateBase + plateHeight, padTop = levelA + BELT_WIDTH + 2;
		attach("clamp", new GantryPlate("Belt clamp", 24, 9, padTop - padBase), AssemblyFrames.translation(xc, blockFront - 4.5, padBase), "carriagePlate");
		for (b in 0...2) {
			var letter = b == 0 ? "A" : "B";
			addMemberConnector("clamp", "belt" + letter, AssemblyFrames.compose(
				AssemblyFrames.inverse(zeroPoses.get("clamp")), AssemblyFrames.translation(xc, gy - R, b == 0 ? levelA : levelB)));
			addBeltPath({belt: "belt" + letter, clamp: {instanceId: "clamp", connectorName: "belt" + letter},
				wraps: [for (id in ["idler" + letter + "Start", "idler" + letter + "End", "idler" + letter + "Front", "pulley" + letter, "idler" + letter + "Rear"])
					{instanceId: id, connectorName: "attach-" + id}]});
		}

		// ---- The toolhead: nozzle, hotend, heater, thermistor, extruder, fans and the probe. -----------------------------
		var nx = xc, ny = gy - TOOL_OFFSET_Y;
		var nozzleHeight = nozzle.spec.coneHeight + nozzle.spec.hexHeight;
		var blockBottom = NOZZLE_Z + nozzleHeight, blockHeight = hotend.spec.blockHeight;
		var mountZ = blockBottom + blockHeight + hotend.spec.heatsinkLength - 12;
		attach("toolheadMount", new GantryPlate("Hotend mount", 30, plateY - 3 - (ny - 13), 20), AssemblyFrames.translation(nx, (plateY - 3 + ny - 13) / 2, mountZ), "carriagePlate");
		attach("hotend", hotend, AssemblyFrames.translation(nx, ny, blockBottom), "toolheadMount");
		attach("nozzle", nozzle, AssemblyFrames.translation(nx, ny, NOZZLE_Z), "hotend");
		attach("cartridge", cartridge, AxisBuilder.orient(nx, ny - 10, blockBottom + blockHeight / 2, up, alongY), "hotend");
		attach("thermistor", hotendThermistor, AxisBuilder.orient(nx - 20, ny, blockBottom + blockHeight / 2 + 3, up, alongX), "hotend");
		// The heatsink fan blows across the heatsink from the right; the blower lies beside the block, its outlet towards the nozzle.
		attach("hotendFan", new Fan("FAN-4010-24V"), AxisBuilder.orient(nx + 11, ny, blockBottom + blockHeight + 10, up, alongX), "toolheadMount");
		attach("partBlower", new Fan("BLOWER-5015-24V"), AxisBuilder.orient(nx - 50, ny, blockBottom + 6, alongY, up), "toolheadMount");
		addExtruder(nx, ny, mountZ + 34.5, plateY - 3);

		// ---- Z: the bed on three screws and two MGN9 rails on the rear posts. --------------------------------------------
		addBed(zAxis, zSpec, shaft);

		// ---- Switches, drivers and the supply. -----------------------------------------------------------------------------
		addHoming(xSpec, ySpec, zAxis, gy, xc, railZ, blockFront, plateY, railXFace, railTop, blockTop);
		homeAfter("x", ["z"]);
		homeAfter("y", ["z"]);
		exposeConnector("nozzleTip", "nozzle", "tip");
		exposeConnector("bedSurface", "bed", "surface");
	}

	/** A GT2 pulley of the printer's size, bored for the motor shaft. */
	function beltPulley():TimingPulley
		return new TimingPulley(GT2, TEETH, motor.variant.shaftDiameter, BELT_WIDTH);

	// ---- the extruder --------------------------------------------------------------------------------------------------

	/**
	 * The extruder housing sits in front of the hotend's mount on the filament axis; its pancake motor is on its front
	 * face, beside the hobbed shaft, and a small pinion drives the gear on that shaft. The gear's angle is the
	 * extruder joint `e`; the motor's shaft follows it, three times as far, through the gear mesh.
	 */
	function addExtruder(nx:Float, ny:Float, zE:Float, plateFront:Float):Void {
		var up = [0.0, 0, 1], alongY = [0.0, 1, 0];
		var spec = extruder.spec;
		var shaftX = nx + spec.hobDiameter / 2 + spec.filamentDiameter / 2;
		var distance = (pinion.pitchDiameter + bigGear.pitchDiameter) / 2;
		attach("extruderMountPlate", new GantryPlate("Extruder mount", 42, plateFront - (ny + 11), spec.height),
			AssemblyFrames.translation(nx, (plateFront + ny + 11) / 2, zE - spec.height / 2), "carriagePlate");
		attach("extruder", extruder, AssemblyFrames.translation(nx, ny, zE), "extruderMountPlate");
		var pancake = NemaStepper.model("17HS08-1004S");
		var motorShaft = shaftX + distance;
		var gearFront = ny - 11 - 6;
		// The motor stands on the front of the gears with its shaft pointing back into the housing.
		attach("motorE", pancake, AxisBuilder.orient(motorShaft, gearFront - 1, zE, up, [0.0, 1, 0]), "extruder");
		// The gear on the hobbed shaft: the joint `e`.
		hang("extruderGear", bigGear, AxisBuilder.orient(shaftX, gearFront, zE, up, alongY), "extruder");
		addMateOnAxis("e", "continuous", "extruder", "to-extruderGear", "extruderGear", "attach-extruderGear", {x: 0, y: 1, z: 0});
		hang("extruderPinion", pinion, AxisBuilder.orient(motorShaft, gearFront, zE, up, alongY), "motorE");
		addMateOnAxis("motorE-turn", "continuous", "motorE", "to-extruderPinion", "extruderPinion", "attach-extruderPinion", {x: 0, y: 1, z: 0});
		addTransmission("extruder-gear", "e", "motorE-turn", Transmission.GearMesh("extruderGear", "extruderPinion"), SenseTools.fromAlignment(1));
		addMotor("motorE", "motorE-turn", "motorE", addDriver("motorE"));
	}

	// ---- the bed ------------------------------------------------------------------------------------------------------

	function addBed(axis:PrinterAxisSpec, railSpec:machinekit.motion.LinearRailProfile.LinearRailProfileSpec, shaft:Float):Void {
		var up = [0.0, 0, 1], vertical = [0.0, 0, 1], alongY = [0.0, 1, 0];
		var thread = zThread();
		var bedTop = NOZZLE_Z;
		var supportBottom = bedTop - bed.thickness() - 6, carrierBottom = supportBottom - 8;
		// Two MGN9 rails on the faces of the rear posts, and the blocks that carry the bed's frame.
		var postFront = REAR_Y - 10;
		var railLength = 360.0, railBottom = 62.0;
		var blockMid = carrierBottom + 4;
		var first = true;
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			var rail = 'railZ$name';
			place(rail, LinearRail.metric(Z_RAIL, railLength), AxisBuilder.orient(side * RAIL_X, postFront, railBottom, [0.0, -1, 0], vertical));
			if (first) {
				slide(axis, rail, 'blockZ$name', LinearRailBlock.metric(Z_RAIL), AxisBuilder.orient(side * RAIL_X, postFront, blockMid, [0.0, -1, 0], vertical),
					{x: 0, y: 0, z: -1});
				first = false;
			}
		}
		var wingWidth = 40.0, wingDepth = 70.0;
		attach("wingLeft", new GantryPlate("Bed wing", wingWidth, wingDepth, 8), AssemblyFrames.translation(-RAIL_X, postFront - wingDepth / 2 + 5, carrierBottom), "blockZLeft");
		attach("carrier", new GantryPlate("Bed carrier", 360, 300, 8), AssemblyFrames.translation(0, -10, carrierBottom), "wingLeft");
		attach("wingRight", new GantryPlate("Bed wing", wingWidth, wingDepth, 8), AssemblyFrames.translation(RAIL_X, postFront - wingDepth / 2 + 5, carrierBottom), "carrier");
		attach("blockZRight", LinearRailBlock.metric(Z_RAIL), AxisBuilder.orient(RAIL_X, postFront, blockMid, [0.0, -1, 0], vertical), "wingRight");
		attach("bedSupport", new GantryPlate("Bed support", 250, 250, 6), AssemblyFrames.translation(0, 0, supportBottom), "carrier");
		attach("bed", bed, AssemblyFrames.translation(0, 0, bedTop), "bedSupport");
		// Where the print is laid: at the machine's zero, the bed's front left corner at its surface.
		attach("printed", new PrintedPart(), AssemblyFrames.translation(MACHINE_ZERO_X, MACHINE_ZERO_Y, bedTop), "bed");
		attach("bedThermistor", bedThermistor, AxisBuilder.orient(110, 0, bedTop - bed.thickness(), up, alongY), "bedSupport");
		// Three screws, each on its own motor standing on the base plate.
		var motorBase = 26.0, motorBody = motor.bodyLength;
		var screwBottom = motorBase + motorBody + motor.variant.shaftLength;
		var screwLength = carrierBottom + 8 + 6 - screwBottom + 12;
		var driverFor = addDriver;
		for (entry in SCREWS) {
			var motorId = 'motor' + entry.id.substr(5);
			attach(motorId, NemaStepper.frame(17), AxisBuilder.orient(entry.x, entry.y, motorBase, [0.0, 1, 0], up), "basePlate");
			driveScrew(axis, entry.id, motorId, new LeadScrew(thread, screwLength), AxisBuilder.orient(entry.x, entry.y, screwBottom, [0.0, 1, 0], up),
				up, [0.0, 0, -1], false, driverFor);
			mountNut(entry.id, "carrier", AxisBuilder.orient(entry.x, entry.y, carrierBottom + 8, [0.0, 1, 0], up));
		}
	}

	// ---- homing -------------------------------------------------------------------------------------------------------

	/**
	 * X: a roller switch on the crossbar at the carriage's left, pressed by a flag on the carriage plate. Y: a roller
	 * switch beside the left rail, pressed by a flag on the gantry's left bracket. Z: an inductive probe on the toolhead,
	 * over the bed's middle at the joints' start, whose trip is the bed surface 1 mm below the nozzle tip.
	 */
	function addHoming(xSpec:PrinterAxisSpec, ySpec:PrinterAxisSpec, zSpec:PrinterAxisSpec, gy:Float, xc:Float, railZ:Float,
			blockFront:Float, plateY:Float, railXFace:Float, railTop:Float, blockTop:Float):Void {
		buildHome(xSpec, "X", "crossbar", "carriagePlate", [xc - 24.0, plateY, railZ - 6.0], [8.0, 8.0, 12.0], [1.0, 0, 0], -1,
			null, [0.0, 1, 0]);
		buildHome(ySpec, "Y", "sideTopLeft", "bracketLeft", [-RAIL_X - 2.0, gy - 22.0, blockTop + 6.0], [8.0, 8.0, 10.0], [0.0, 1, 0], -1,
			null, [1.0, 0, 0]);
		// Z homes with the bed lowered, so that X and Y never find the nozzle over a bed that is up: an inductive
		// sensor on a pillar from the base plate senses a steel flag under the bed's carrier. Pre-homing the bed
		// away from the nozzle is also why a probe at the nozzle cannot be the home switch: it would trip inside
		// the travel, and a home switch must trip beyond it, in the room before the end stop.
		buildHome(zSpec, "Z", "basePlate", "carrier", [60.0, -60.0, carrierBottomZero() - 6.0], [12.0, 12.0, 6.0], [0.0, 0, -1], 1,
			null, [1.0, 0, 0], true);
		// A probe beside the nozzle for levelling the bed: its face is a millimetre above the nozzle tip.
		var probe = new ProximitySwitch();
		var nx = xc, ny = gy - TOOL_OFFSET_Y;
		var face = NOZZLE_Z + PROBE_FACE_ABOVE_TIP;
		attach("probeMount", new GantryPlate("Probe mount", 30, 20, 6), AssemblyFrames.translation(nx + PROBE_X - 5, ny + PROBE_Y, face + probe.spec.length - 6), "toolheadMount");
		attach("probe", probe, AxisBuilder.orient(nx + PROBE_X, ny + PROBE_Y, face + probe.spec.length, [1.0, 0, 0], [0.0, 0, -1]), "probeMount");
	}

	/** Underside of the bed's carrier with the bed up. */
	static function carrierBottomZero():Float return NOZZLE_Z - 5.5 - 6 - 8;

	/** How far above the nozzle tip the probe's face sits; the probe trips when the bed is its sensing distance below. */
	public static inline var PROBE_FACE_ABOVE_TIP:Float = 1.0;
	/** Where the probe stands relative to the nozzle, millimetres: to the right of it and a little in front. */
	public static inline var PROBE_X:Float = 38;
	public static inline var PROBE_Y:Float = -4;

	/**
	 * A roller switch on `fixed`, pressed by a flag on `moving` at `point` (the flag's base centre, zero pose) as axis
	 * `axis` reaches the end named by `side`. The flag is `size` big and its face towards the switch carries the trip
	 * connector; the switch stands on a small block from the nearest point of `fixed`, a quarter of the axis's overtravel
	 * beyond its end of travel.
	 */
	function buildHome(axis:PrinterAxisSpec, suffix:String, fixed:String, moving:String, point:Array<Float>, size:Array<Float>,
			direction:Array<Float>, side:Int, shaft:Null<String>, transverse:Array<Float>, ?inductive:Bool = false):Void {
		var room = axisOvertravel(axis.id);
		var sensor:machinekit.component.MachineComponent = inductive ? new ProximitySwitch() : new LimitSwitch();
		var contact:machinekit.motion.SwitchPart = cast sensor;
		if (!Math.isFinite(room) || room <= 4 * contact.switchRepeatability())
			throw "Printer home uncertainty does not fit its physical guide overtravel";
		var trigger = "homeTrigger" + suffix;
		attach(trigger, new GantryPlate("Home flag " + suffix, size[0], size[1], size[2]),
			AssemblyFrames.translation(point[0], point[1], point[2] - size[2] / 2), moving);
		var half = 0.0;
		for (i in 0...3) half += Math.abs(direction[i]) * size[i] / 2;
		addMemberConnector(trigger, "trip", AssemblyFrames.translation(direction[0] * side * half,
			direction[1] * side * half, size[2] / 2 + direction[2] * side * half));
		var travel = (side < 0 ? axis.lower : axis.upper) + side * room * 0.25;
		var normal = [for (value in direction) -side * value];
		var face = [for (i in 0...3) point[i] + direction[i] * (travel + side * half)];
		// The roller's trip point sits at the flag's face; the housing stands behind it.
		var tripLocal = if (inductive) {
			var spec = cast(sensor, ProximitySwitch).spec;
			{x: 0.0, z: spec.length + spec.sensingDistance};
		} else {
			var spec = cast(sensor, LimitSwitch).spec;
			{x: spec.width * 0.4, z: spec.depth + spec.rollerDiameter - spec.pretravel};
		}
		var across = [normal[1] * transverse[2] - normal[2] * transverse[1], normal[2] * transverse[0] - normal[0] * transverse[2],
			normal[0] * transverse[1] - normal[1] * transverse[0]];
		// orient(up = transverse, along = normal): local x is up x along.
		var localX = [transverse[1] * normal[2] - transverse[2] * normal[1], transverse[2] * normal[0] - transverse[0] * normal[2],
			transverse[0] * normal[1] - transverse[1] * normal[0]];
		var origin = [for (i in 0...3) face[i] - localX[i] * tripLocal.x - normal[i] * tripLocal.z];
		var pose = AxisBuilder.orient(origin[0], origin[1], origin[2], transverse, normal);
		var host = switchMountBounds(fixed);
		var anchor = [for (i in 0...3) Math.max(host.min[i], Math.min(host.max[i], origin[i]))];
		var lo = [for (i in 0...3) Math.min(anchor[i], origin[i]) - 3];
		var hi = [for (i in 0...3) Math.max(anchor[i], origin[i]) + 3];
		var id = "home" + suffix, mount = id + "Mount";
		attach(mount, new GantryPlate("Home switch mount " + suffix, hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]),
			AssemblyFrames.translation((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]), fixed);
		attach(id, sensor, pose, mount);
		addTripSwitch(id, axis.id, id, {instanceId: trigger, connectorName: "trip"}, side, "home", 1, shaft);
	}

	function switchMountBounds(id:String):{min:Array<Float>, max:Array<Float>} {
		var member = component(id), min:Array<Float>, max:Array<Float>;
		if (Std.isOfType(member, GantryExtrusion)) {
			var frame:GantryExtrusion = cast member;
			min = [-frame.profile.size / 2, -frame.profile.height / 2, 0.0];
			max = [frame.profile.size / 2, frame.profile.height / 2, frame.length];
		} else if (Std.isOfType(member, GantryPlate)) {
			var plate:GantryPlate = cast member;
			min = [-plate.width / 2, -plate.depth / 2, 0.0]; max = [plate.width / 2, plate.depth / 2, plate.height];
		} else throw "Printer switch mount requires a frame member or a plate";
		var lo = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var hi = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		for (x in [min[0], max[0]]) for (y in [min[1], max[1]]) for (z in [min[2], max[2]]) {
			var point = AssemblyFrames.transformPoint(zeroPose(id), x, y, z), values = [point.x, point.y, point.z];
			for (i in 0...3) { lo[i] = Math.min(lo[i], values[i]); hi[i] = Math.max(hi[i], values[i]); }
		}
		return {min: lo, max: hi};
	}

	// ---- electronics --------------------------------------------------------------------------------------------------

	/** Drivers live on the base plate, apart from the carriage. */
	function addDriver(motorId:String):String {
		var id = motorId + "Driver";
		if (!supplyAdded) {
			attach("powerSupply", new PowerSupply(new machinekit.power.DcVoltage(SUPPLY_VOLTS), new machinekit.power.DcCurrent(20), 8), AssemblyFrames.translation(0, FRONT_Y + 40, 26), "basePlate");
			supplyAdded = true;
		}
		var extruderMotor = motorId == "motorE";
		attach(id, new MotorDriver("GENERIC-TMC2209",
			new machinekit.power.PhaseRmsCurrent(extruderMotor ? EXTRUDER_DRIVER_CURRENT : DRIVER_CURRENT), MICROSTEPS),
			AssemblyFrames.translation(-120 + 40 * driverIndex++, FRONT_Y + 80, 26), "basePlate");
		connectPorts('$id-power', "powerSupply", 'power$driverIndex', id, "power");
		return id;
	}

	public static function noseAt(x:Float, y:Float, z:Float):{x:Float, y:Float, z:Float}
		return {x: MACHINE_ZERO_X + x, y: MACHINE_ZERO_Y + y, z: NOZZLE_Z + z};
}

/** Travel of one axis, in millimetres. */
typedef PrinterAxisSpec = machinekit.assembly.AxisBuilder.AxisSpec;
