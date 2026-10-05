package machinekit.gantry;

import machinekit.assembly.AxisBuilder;
import machinekit.assembly.AxisBuilder.AxisSpec;
import machinekit.assembly.MachineAssembly;
import machinekit.gantry.GantryParts.GantryExtrusion;
import machinekit.gantry.GantryParts.GantryPlate;
import machinekit.gantry.GantryParts.GantryBoredBracket;
import machinekit.component.Solids;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.NemaStepper;
import machinekit.motion.MotorDriver;
import machinekit.motion.PowerSupply;
import machinekit.robotics.RobotFlange;
import machinekit.structural.TSlotExtrusion;
import machinekit.transmission.Rack;
import machinekit.transmission.SpurGear;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Floor-mounted external axis. Dimensions and cable-chain clearance are design assumptions,
 * while the rack ratio and drive envelope come from its physical parts. Coordinates are mm.
 */
class LinearTrack extends AxisBuilder {
	public final axis:AxisSpec;
	public final flange:RobotFlange;
	public final mountZero:AssemblyFrame;
	public final railMargin:Float;
	/** Unoccupied longitudinal strip beside the carriage for the moving service chain. */
	public final cableChainRoom:{minY:Float, maxY:Float, minZ:Float, maxZ:Float} = {minY: 360.0, maxY: 480.0, minZ: 0.0, maxZ: 120.0};
	public final assumedFields:Array<String> = ["travel", "rail spacing", "carriage dimensions", "frame dimensions",
		"cable chain room", "supply voltage", "driver current", "microsteps"];

	public function new(travel:Float = 3000, initial:Float = 0) {
		super();
		if (!Math.isFinite(travel) || travel <= 0 || !Math.isFinite(initial) || initial < 0 || initial > travel)
			throw "Track needs positive finite travel and an initial coordinate inside it";
		axis = {id: "track", lower: 0.0, upper: travel, initial: initial};
		flange = new RobotFlange(63);
		var guide = LinearRailBlock.metric("HGR15").spec;
		var blockOffset = 140.0;
		railMargin = guide.railEndMargin + guide.blockLength / 2 + 50;
		var start = -railMargin - blockOffset;
		var length = travel + 2 * (railMargin + blockOffset);
		var up = [0.0, 0, 1], along = [1.0, 0, 0];
		var frame = new TSlotExtrusion(80);
		place("frameLeft", new GantryExtrusion(frame, length), AxisBuilder.orient(start, -170, 40, up, along));
		attach("frameRight", new GantryExtrusion(frame, length), AxisBuilder.orient(start, 170, 40, up, along), "frameLeft");
		for (end in [0, 1]) attach('frameEnd$end', new GantryPlate("track end", 80, 420, 40),
			AssemblyFrames.translation(end == 0 ? start - 40 : start + length + 40, 0, 0), "frameLeft");
		var railTop = 80 + guide.railHeight;
		for (side in ["Left", "Right"]) attach('rail$side', LinearRail.metric("HGR15", length),
			AxisBuilder.orient(start, side == "Left" ? -170 : 170, railTop, up, along), 'frame$side');
		slide(axis, "railLeft", "blockLeftRear", LinearRailBlock.metric("HGR15"),
			AxisBuilder.orient(-blockOffset, -170, railTop, up, along), {x: 1.0, y: 0.0, z: 0.0});
		var carriageBottom = railTop + guide.blockHeight - guide.railHeight;
		attach("carriage", new GantryPlate("track carriage", 520, 560, 25),
			AssemblyFrames.translation(0, 0, carriageBottom), "blockLeftRear");
		for (side in ["Left", "Right"]) for (end in ["Rear", "Front"]) {
			var id = 'block$side$end';
			if (id == "blockLeftRear") continue;
			attach(id, LinearRailBlock.metric("HGR15"), AxisBuilder.orient(end == "Rear" ? -blockOffset : blockOffset,
				side == "Left" ? -170 : 170, railTop, up, along), "carriage");
		}
		var flangeZero = AssemblyFrames.translation(0, 0, carriageBottom + 25 + flange.thickness);
		attach("armMount", flange, flangeZero, "carriage");
		var adapterHeight = flange.pilotHeight + 3;
		attach("armAdapter", new GantryBoredBracket("track arm adapter", 280, 280, adapterHeight,
			AssemblyFrames.identity(), flange.pilotDiameter + 0.5), flangeZero, "armMount");
		addMemberConnector("armAdapter", "floor", Solids.axial(0, 0, adapterHeight));
		exposeConnector("armMount", "armAdapter", "floor");
		mountZero = AssemblyFrames.translation(0, 0, flangeZero.z + adapterHeight);

		var motor = NemaStepper.frame(34);
		var rating = motor.rating();
		if (rating == null) throw "Track motor needs rated drive data";
		var driverCurrent = Math.min(rating.ratedCurrent, 3.0);
		var motorFace = AxisBuilder.orient(0, -310, 98, up, [0.0, 1, 0]);
		// Two side struts leave the pinion's complete swept disc clear.
		for (side in [-1, 1]) attach(side < 0 ? "motorSupportLeft" : "motorSupportRight",
			new GantryPlate("track motor strut", 20, 36, 12), AssemblyFrames.translation(side * 38, -292, carriageBottom - 12), "carriage");
		attach("motorPlate", new GantryPlate("track motor mount", motor.spec.face + 12, motor.spec.face + 12, 6,
			motor, AssemblyFrames.identity()), motorFace, "motorSupportLeft");
		attach("motor", motor, motorFace, "motorPlate");
		var moduleSize = 2.0, width = 20.0;
		var pinion = new SpurGear(moduleSize, 20, width, SpurGear.STANDARD_PRESSURE_ANGLE, 0, 0, motor.variant.shaftDiameter);
		var pinionY = motorFace.y + motor.variant.shaftLength - width;
		var rackPitch = Math.PI * moduleSize;
		var rackMargin = Math.ceil((railMargin + blockOffset) / rackPitch) * rackPitch;
		var rack = new Rack(moduleSize, Std.int(Math.ceil((travel + 2 * rackMargin) / rackPitch)), width,
			SpurGear.STANDARD_PRESSURE_ANGLE, 12);
		var rackZ = motorFace.z - pinion.pitchDiameter / 2;
		attach("rackSupport", new GantryExtrusion(new TSlotExtrusion(30), rack.length),
			AxisBuilder.orient(-rackMargin, pinionY + width / 2, rackZ - 1.25 * moduleSize - rack.barHeight - 15, up, along), "frameLeft");
		for (end in [0, 1]) attach('rackFoot$end', new GantryPlate("track rack foot", 60, 155, 30),
			AssemblyFrames.translation(end == 0 ? start + 30 : start + length - 30, -227.5, 20), "frameLeft");
		var pinionPose = AssemblyFrames.compose(AxisBuilder.orient(0, pinionY, motorFace.z, up, [0.0, 1, 0]),
			AssemblyFrames.fromRotationMatrix(0, 0, 0, [0.0, 1, 0, -1, 0, 0, 0, 0, 1]));
		attach("powerSupply", new PowerSupply(48, 10, 1), AssemblyFrames.translation(start, 550, 0), "frameLeft");
		driveRack(axis, "pinion", "motor", pinion, pinionPose, [0.0, 1, 0], "rack", rack,
			AxisBuilder.orient(-rackMargin, pinionY + width, rackZ, up, along), "rackSupport", -1.0, _ -> {
				attach("driver", new MotorDriver("GENERIC-DM542", driverCurrent, 16),
					AssemblyFrames.translation(start + 150, 550, 0), "frameLeft");
				connectPorts("driver-power", "powerSupply", "power1", "driver", "power");
				return "driver";
			});
	}

	/** Preserve the arm's level and its drives; only its floor interface is mated here. */
	public function includeArm(id:String, arm:MachineAssembly):Void {
		var floor = arm.connector("floor");
		include(id, arm);
		for (name in arm.portNames()) {
			var port = arm.port(name, id);
			exposePort(id + "/" + name, port.instanceId, port.portName);
		}
		var mount = connector("armMount");
		addMate(id + "-mount", "fixed", mount.instanceId, mount.connectorName, id + "/" + floor.instanceId, floor.connectorName);
	}
}
