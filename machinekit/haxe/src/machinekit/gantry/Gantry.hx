package machinekit.gantry;

import machinekit.assembly.AxisBuilder;
import machinekit.assembly.AxisBuilder.AxisSpec;
import machinekit.gantry.GantrySpec.GantryDrive;
import machinekit.gantry.GantryParts.GantryExtrusion;
import machinekit.gantry.GantryParts.GantryPlate;
import machinekit.gantry.GantryParts.GantryIdlerPin;
import machinekit.gantry.GantryParts.GantryBeltClamp;
import machinekit.gantry.GantryParts.GantryNutMount;
import machinekit.gantry.GantryParts.GantryMotorSupport;
import machinekit.component.MachineComponent;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.NemaStepper;
import machinekit.motion.MotorDriver;
import machinekit.motion.PowerSupply;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewNut;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;
import machinekit.transmission.Rack;
import machinekit.transmission.SpurGear;
import machinekit.robotics.RobotFlange;
import machinekit.structural.TSlotExtrusion;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

private typedef GantryBox = {min:Array<Float>, max:Array<Float>};

/** Three translational axes assembled from real guides, drives and a standard robot tool flange. */
class Gantry extends AxisBuilder {
	public final spec:GantrySpec;
	public final axes:Array<AxisSpec>;
	public final flangeZero:AssemblyFrame;
	public final railMargin:Float;
	var driverIndex:Int = 0;

	public function new(spec:GantrySpec) {
		super();
		this.spec = spec;
		axes = [{id: "x", lower: 0.0, upper: spec.travelX, initial: 0.0},
			{id: "y", lower: 0.0, upper: spec.travelY, initial: 0.0},
			{id: "z", lower: 0.0, upper: spec.travelZ, initial: 0.0}];
		var frame = TSlotExtrusion.forProfile(spec.frameProfile);
		var beam = TSlotExtrusion.forProfile(spec.beamProfile);
		var guide = LinearRailBlock.metric(spec.railProfile).spec;
		railMargin = Math.max(80, guide.railEndMargin + guide.blockLength / 2 + 20);
		var left = -railMargin, right = spec.travelX + railMargin;
		var front = -railMargin, back = spec.travelY + railMargin;
		var frameZ = spec.travelZ + 350;
		var endAllowance = Math.max(frame.size / 2, NemaStepper.frame(spec.motorFrame).variant.shaftLength + 6);
		var up = [0.0, 0, 1], alongX = [1.0, 0, 0], alongY = [0.0, 1, 0];
		place("frameFront", new GantryExtrusion(frame, right - left - frame.size),
			AxisBuilder.orient(left + frame.size / 2, front, frameZ, up, alongX));
		attach("frameBack", new GantryExtrusion(frame, right - left - frame.size),
			AxisBuilder.orient(left + frame.size / 2, back, frameZ, up, alongX), "frameFront");
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right", x = side < 0 ? left : right;
			attach('frame$name', new GantryExtrusion(frame, back - front + 2 * endAllowance),
				AxisBuilder.orient(x, front - endAllowance, frameZ, up, alongY), "frameFront");
			if (!spec.tableMounted) for (end in [0, 1]) {
				var id = 'post$name${end == 0 ? "Front" : "Back"}';
				attach(id, new GantryExtrusion(frame, frameZ - frame.height / 2),
					AxisBuilder.orient(x, end == 0 ? front : back, 0, [0.0, 1, 0], up), "frameFront");
			}
		}
		var railYTop = frameZ + frame.height / 2 + guide.railHeight;
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right", x = side < 0 ? left : right;
			attach('railY$name', LinearRail.metric(spec.railProfile, spec.travelY + 2 * railMargin),
				AxisBuilder.orient(x, -railMargin, railYTop, up, alongY), 'frame$name');
		}
		// One translational leader; the opposite guide is rigidly held by the beam.
		slide(axes[1], "railYLeft", "blockYLeft", LinearRailBlock.metric(spec.railProfile),
			AxisBuilder.orient(left, 0, railYTop, up, alongY), {x: 0.0, y: 1.0, z: 0.0});
		var feetZ = railYTop + guide.blockHeight - guide.railHeight;
		attach("beamFootLeft", new GantryPlate("beam foot", 60, 60, 12),
			AssemblyFrames.translation(left, 0, feetZ), "blockYLeft");
		var beamZ = feetZ + 12 + beam.height / 2;
		attach("beam", new GantryExtrusion(beam, right - left + 2 * endAllowance),
			AxisBuilder.orient(left - endAllowance, 0, beamZ, up, alongX), "beamFootLeft");
		attach("beamFootRight", new GantryPlate("beam foot", 60, 60, 12),
			AssemblyFrames.translation(right, 0, feetZ), "beam");
		attach("blockYRight", LinearRailBlock.metric(spec.railProfile),
			AxisBuilder.orient(right, 0, railYTop, up, alongY), "beamFootRight");
		var railXTop = beamZ + beam.height / 2 + guide.railHeight;
		attach("railX", LinearRail.metric(spec.railProfile, spec.travelX + 2 * railMargin),
			AxisBuilder.orient(-railMargin, 0, railXTop, up, alongX), "beam");
		slide(axes[0], "railX", "blockX", LinearRailBlock.metric(spec.railProfile),
			AxisBuilder.orient(0, 0, railXTop, up, alongX), {x: 1.0, y: 0.0, z: 0.0});
		var xPlateZ = railXTop + guide.blockHeight - guide.railHeight;
		// The Z column is in front of the beam, joined by the carriage's top plate.
		var columnY = -(beam.size / 2 + frame.height / 2 + 40);
		attach("xCarriage", new GantryPlate("X carriage", 80, -columnY + 80, 12),
			AssemblyFrames.translation(0, columnY / 2, xPlateZ), "blockX");
		var columnLength = spec.travelZ + 2 * railMargin;
		attach("zColumn", new GantryExtrusion(frame, columnLength),
			AxisBuilder.orient(0, columnY, xPlateZ - columnLength, [0.0, 1, 0], up), "xCarriage");
		var railZFace = columnY - frame.height / 2 - guide.railHeight;
		attach("railZ", LinearRail.metric(spec.railProfile, columnLength),
			AxisBuilder.orient(0, railZFace, xPlateZ, [0.0, -1, 0], [0.0, 0, -1]), "zColumn");
		slide(axes[2], "railZ", "blockZ", LinearRailBlock.metric(spec.railProfile),
			AxisBuilder.orient(0, railZFace, xPlateZ - railMargin, [0.0, -1, 0], [0.0, 0, -1]),
			{x: 0.0, y: 0.0, z: -1.0});
		var zPlateY = railZFace - (guide.blockHeight - guide.railHeight) - 4;
		attach("zCarriage", new GantryPlate("Z carriage", 80, 8, 100),
			AssemblyFrames.translation(0, zPlateY, xPlateZ - railMargin - 80), "blockZ");
		var flange = new RobotFlange(50);
		flangeZero = AxisBuilder.orient(0, zPlateY, xPlateZ - railMargin - 80 - flange.thickness,
			[0.0, 1, 0], [0.0, 0, -1]);
		attach("flange", flange, flangeZero, "zCarriage");
		exposeConnector("toolFlange", "flange", "face");
		attach("powerSupply", new PowerSupply(spec.supplyVoltage, 20, spec.dualY ? 4 : 3),
			AssemblyFrames.translation(left - 180, front, 30), "frameFront");
		buildDrive(axes[1], spec.driveY, "YLeft", "frameLeft", "beamFootLeft",
			[left - 70, 0.0, feetZ + 6], alongY);
		if (spec.dualY) buildDrive(axes[1], spec.driveY, "YRight", "frameRight", "beamFootRight",
			[right + 70, 0.0, feetZ + 6], alongY);
		buildDrive(axes[0], spec.driveX, "X", "beam", "xCarriage", [0.0, 70, xPlateZ + 6], alongX);
		buildDrive(axes[2], spec.driveZ, "Z", "zColumn", "zCarriage",
			[70.0, zPlateY, xPlateZ - railMargin - 40], [0.0, 0, -1]);
	}

	function addDriver(motorId:String):String {
		var motor:NemaStepper = cast component(motorId);
		var rating = motor.rating();
		if (rating == null) throw "Gantry motor needs rated drive data";
		var family = rating.ratedCurrent <= 2 && spec.supplyVoltage <= 29 ? "GENERIC-TMC2209" : "GENERIC-DM542";
		var id = motorId + "Driver";
		attach(id, new MotorDriver(family, Math.min(rating.ratedCurrent, family == "GENERIC-TMC2209" ? 2.0 : 3.0), spec.microsteps),
			AssemblyFrames.translation(-railMargin - 180, -railMargin - 130 - 125 * driverIndex, 30), "frameFront");
		driverIndex++;
		connectPorts(id + "-power", "powerSupply", 'power$driverIndex', id, "power");
		return id;
	}

	function mountMotor(id:String, parent:String, face:AssemblyFrame):NemaStepper {
		var motor = NemaStepper.frame(spec.motorFrame);
		var plate = new GantryPlate("motor mount", motor.spec.face + 12, motor.spec.face + 12, 6, motor, AssemblyFrames.identity());
		var support = supportMotor(id + "Support", parent, plate, face, motor);
		attach(id + "Plate", plate, face, support);
		attach(id, motor, face, id + "Plate");
		return motor;
	}

	/** World bounds of an extrusion or mounting plate at its zero pose. */
	static function mountBounds(member:MachineComponent, pose:AssemblyFrame):GantryBox {
		var min:Array<Float>, max:Array<Float>;
		if (Std.isOfType(member, GantryExtrusion)) {
			var extrusion:GantryExtrusion = cast member;
			min = [-extrusion.profile.size / 2, -extrusion.profile.height / 2, 0.0];
			max = [extrusion.profile.size / 2, extrusion.profile.height / 2, extrusion.length];
		} else if (Std.isOfType(member, GantryPlate)) {
			var plate:GantryPlate = cast member;
			min = [-plate.width / 2, -plate.depth / 2, 0.0]; max = [plate.width / 2, plate.depth / 2, plate.height];
		} else throw "Gantry motor mount must be supported by a frame or carriage plate";
		var lo = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var hi = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		for (x in [min[0], max[0]]) for (y in [min[1], max[1]]) for (z in [min[2], max[2]]) {
			var point = AssemblyFrames.transformPoint(pose, x, y, z), coordinates = [point.x, point.y, point.z];
			for (axis in 0...3) { lo[axis] = Math.min(lo[axis], coordinates[axis]); hi[axis] = Math.max(hi[axis], coordinates[axis]); }
		}
		return {min: lo, max: hi};
	}

	function supportMotor(id:String, parent:String, plate:GantryPlate, face:AssemblyFrame, motor:NemaStepper):String {
		var host = mountBounds(component(parent), zeroPose(parent)), target = mountBounds(plate, face);
		var gaps:Array<Int> = [];
		for (axis in 0...3) if (target.min[axis] > host.max[axis] + 1e-7 || host.min[axis] > target.max[axis] + 1e-7) gaps.push(axis);
		if (gaps.length == 0) return parent;
		if (gaps.length == 2) return supportMotorCorner(id, parent, host, target, face, motor, gaps);
		if (gaps.length > 2) throw "A gantry motor bracket needs a common supporting plane";
		var gapAxis = gaps[0];
		var min:Array<Float> = [], max:Array<Float> = [];
		for (axis in 0...3) {
			if (axis == gapAxis) {
				min.push(Math.min(host.max[axis], target.max[axis])); max.push(Math.max(host.min[axis], target.min[axis]));
			} else {
				min.push(Math.max(host.min[axis], target.min[axis])); max.push(Math.min(host.max[axis], target.max[axis]));
			}
			if (max[axis] - min[axis] <= 1e-7) throw "Gantry motor bracket has no supporting contact area";
		}
		var pose = AssemblyFrames.translation((min[0] + max[0]) / 2, (min[1] + max[1]) / 2, min[2]);
		attach(id, new GantryMotorSupport(id, max[0] - min[0], max[1] - min[1], max[2] - min[2], motor,
			AssemblyFrames.compose(AssemblyFrames.inverse(pose), face)), pose, parent);
		return id;
	}

	/** Two intersecting arms support a motor beside a frame corner without passing through its body. */
	function supportMotorCorner(id:String, parent:String, host:GantryBox, target:GantryBox,
			face:AssemblyFrame, motor:NemaStepper, gaps:Array<Int>):String {
		var a = gaps[0], b = gaps[1], c = 3 - a - b;
		var signA = target.min[a] > host.max[a] ? 1.0 : -1.0;
		var signB = target.min[b] > host.max[b] ? 1.0 : -1.0;
		var hostA = signA > 0 ? host.max[a] : host.min[a];
		var targetA = signA > 0 ? target.min[a] : target.max[a];
		var hostB = signB > 0 ? host.max[b] : host.min[b];
		var stripeA = Math.min(12, Math.abs(targetA - hostA));
		var stripeB = Math.min(12, host.max[b] - host.min[b]);
		var loC = Math.max(host.min[c], target.min[c]), hiC = Math.min(host.max[c], target.max[c]);
		if (hiC - loC <= 1e-7) throw "Corner motor bracket has no common contact area";
		var firstMin = [0.0, 0, 0], firstMax = [0.0, 0, 0];
		firstMin[a] = Math.min(hostA, targetA); firstMax[a] = Math.max(hostA, targetA);
		firstMin[b] = Math.min(hostB, hostB - signB * stripeB); firstMax[b] = Math.max(hostB, hostB - signB * stripeB);
		firstMin[c] = loC; firstMax[c] = hiC;
		var secondMin = firstMin.copy(), secondMax = firstMax.copy();
		secondMin[a] = Math.min(targetA, targetA - signA * stripeA); secondMax[a] = Math.max(targetA, targetA - signA * stripeA);
		secondMin[b] = Math.min(target.min[b], firstMin[b]); secondMax[b] = Math.max(target.max[b], firstMax[b]);
		var min = [for (axis in 0...3) Math.min(firstMin[axis], secondMin[axis])];
		var max = [for (axis in 0...3) Math.max(firstMax[axis], secondMax[axis])];
		var pose = AssemblyFrames.translation((min[0] + max[0]) / 2, (min[1] + max[1]) / 2, min[2]);
		var sections:Array<Array<Float>> = [];
		for (pair in [{lo: firstMin, hi: firstMax}, {lo: secondMin, hi: secondMax}])
			sections.push([pair.lo[0] - pose.x, pair.lo[1] - pose.y, pair.lo[2] - pose.z,
				pair.hi[0] - pose.x, pair.hi[1] - pose.y, pair.hi[2] - pose.z]);
		attach(id, new GantryMotorSupport(id, max[0] - min[0], max[1] - min[1], max[2] - min[2], motor,
			AssemblyFrames.compose(AssemblyFrames.inverse(pose), face), sections), pose, parent);
		return id;
	}

	/** A plate bridges the actual gap from the carriage edge to the clamp jaw. */
	function supportClamp(id:String, parent:String, point:Array<Float>, direction:Array<Float>,
			transverse:Array<Float>, normal:Array<Float>, beltWidth:Float):String {
		var halfNormal = (beltWidth + 4) / 2;
		var radiusX = Math.abs(direction[0]) * 10 + Math.abs(transverse[0]) * 6 + Math.abs(normal[0]) * halfNormal;
		var radiusY = Math.abs(direction[1]) * 10 + Math.abs(transverse[1]) * 6 + Math.abs(normal[1]) * halfNormal;
		var pose = zeroPose(parent);
		var height = direction[2] == 0 ? 4.0 : 20.0;
		var z = direction[2] == 0 ? pose.z : point[2] - height / 2;
		return supportBox(id, parent, point, radiusX, radiusY, height, z);
	}

	function supportBox(id:String, parent:String, point:Array<Float>, radiusX:Float, radiusY:Float,
			height:Float, z:Float):String {
		var carrier:GantryPlate = cast component(parent);
		var pose = zeroPose(parent);
		var dx = point[0] - pose.x, dy = point[1] - pose.y;
		var gapX = Math.abs(dx) - carrier.width / 2 - radiusX;
		var gapY = Math.abs(dy) - carrier.depth / 2 - radiusY;
		if (gapX <= 0 && gapY <= 0) return parent;
		var width:Float, depth:Float, x:Float, y:Float;
		if (gapX > gapY) {
			var sign = dx < 0 ? -1.0 : 1.0;
			var host = pose.x + sign * carrier.width / 2, jaw = point[0] - sign * radiusX;
			width = Math.abs(jaw - host); depth = Math.max(12, Math.abs(dy) + 8);
			x = (host + jaw) / 2; y = (point[1] + pose.y) / 2;
		} else {
			var sign = dy < 0 ? -1.0 : 1.0;
			var host = pose.y + sign * carrier.depth / 2, jaw = point[1] - sign * radiusY;
			width = 20; depth = Math.abs(jaw - host); x = point[0]; y = (host + jaw) / 2;
		}
		attach(id, new GantryPlate("drive support bracket", width, depth, height), AssemblyFrames.translation(x, y, z), parent);
		return id;
	}

	function buildDrive(axis:AxisSpec, drive:GantryDrive, suffix:String, fixed:String, moving:String,
			origin:Array<Float>, direction:Array<Float>):Void {
		var transverse:Array<Float> = direction[2] == 0 ? [0.0, 0, 1] : [1.0, 0, 0];
		var normal = [direction[1] * transverse[2] - direction[2] * transverse[1],
			direction[2] * transverse[0] - direction[0] * transverse[2],
			direction[0] * transverse[1] - direction[1] * transverse[0]];
		var motorId = "motor" + suffix;
		var selectedMotor = NemaStepper.frame(spec.motorFrame);
		var shaft = selectedMotor.variant.shaftLength;
		function point(distance:Float):Array<Float> return [origin[0] + direction[0] * distance,
			origin[1] + direction[1] * distance, origin[2] + direction[2] * distance];
		switch drive {
			case Screw(thread):
				var start = point(-railMargin), face = point(-railMargin - shaft);
				mountMotor(motorId, fixed, AxisBuilder.orient(face[0], face[1], face[2], transverse, direction));
				var screwId = "screw" + suffix;
				driveScrew(axis, screwId, motorId, new LeadScrew(thread, axis.upper + 2 * railMargin),
					AxisBuilder.orient(start[0], start[1], start[2], transverse, direction), direction, direction, true, addDriver);
				var nut:LeadScrewNut = cast component(screwId + "Nut");
				var faceDistance = nut.bodyLength + nut.flangeThickness;
				var nutFace = point(faceDistance);
				var mount = new GantryNutMount(nut);
				var size = mount.size;
				var localX = [transverse[1] * direction[2] - transverse[2] * direction[1],
					transverse[2] * direction[0] - transverse[0] * direction[2],
					transverse[0] * direction[1] - transverse[1] * direction[0]];
				var centre = [nutFace[0] + direction[0] * GantryNutMount.THICKNESS / 2,
					nutFace[1] + direction[1] * GantryNutMount.THICKNESS / 2,
					nutFace[2] + direction[2] * GantryNutMount.THICKNESS / 2];
				var radiusX = (Math.abs(localX[0]) + Math.abs(transverse[0])) * size / 2 + Math.abs(direction[0]) * GantryNutMount.THICKNESS / 2;
				var radiusY = (Math.abs(localX[1]) + Math.abs(transverse[1])) * size / 2 + Math.abs(direction[1]) * GantryNutMount.THICKNESS / 2;
				var supportZ = direction[2] == 0 ? zeroPose(moving).z : centre[2] - GantryNutMount.THICKNESS / 2;
				var parent = supportBox(screwId + "NutSupport", moving, centre, radiusX, radiusY, GantryNutMount.THICKNESS, supportZ);
				var mountPose = AxisBuilder.orient(nutFace[0], nutFace[1], nutFace[2], transverse, direction);
				attach(screwId + "NutMount", mount, mountPose, parent);
				mountNut(screwId, screwId + "NutMount", mountPose);
			case Belt(profile, teeth, width):
				var start = point(-railMargin), end = point(axis.upper + railMargin);
				var face = [start[0] - normal[0] * (shaft - width), start[1] - normal[1] * (shaft - width), start[2] - normal[2] * (shaft - width)];
				var motor = mountMotor(motorId, fixed, AxisBuilder.orient(face[0], face[1], face[2], transverse, normal));
				var beltId = "belt" + suffix;
				var belt = TimingBelt.twoPulley(profile, teeth, teeth, axis.upper + 2 * railMargin, width);
				var plane = AxisBuilder.orient(start[0], start[1], start[2], transverse, normal);
				var pulleyId = "pulley" + suffix, idlerId = "idler" + suffix;
				twoPulleyAxis({axis: axis, beltId: beltId, belt: belt, beltPose: plane, beltParent: fixed,
					motorId: motorId, pulleyId: pulleyId, pulley: new TimingPulley(profile, teeth, motor.variant.shaftDiameter, width),
					idlerId: idlerId, idler: new TimingPulley(profile, teeth, motor.variant.shaftDiameter, width),
					idlerPose: AxisBuilder.orient(end[0], end[1], end[2], transverse, normal), about: normal,
					strand: 0, travelX: 1.0, travelY: 0.0, driverForMotor: addDriver,
					mountIdler: () -> {
						var mount = AxisBuilder.orient(end[0] - normal[0] * 6, end[1] - normal[1] * 6,
							end[2] - normal[2] * 6, transverse, normal);
						var plate = new GantryPlate("idler mount", 60, 60, 6);
						var support = supportMotor(idlerId + "Support", fixed, plate, mount, motor);
						attach(idlerId + "Plate", plate, mount, support);
						attach(idlerId + "Pin", new GantryIdlerPin(motor.variant.shaftDiameter, width),
							AxisBuilder.orient(end[0], end[1], end[2], transverse, normal), idlerId + "Plate");
						return idlerId + "Pin";
					}});
				var strand = belt.strands()[0];
				var clampPoint = AssemblyFrames.transformPoint(plane, railMargin, strand.startY, width / 2);
				var clampId = "clamp" + suffix;
				var clampParent = supportClamp(clampId + "Support", moving,
					[clampPoint.x, clampPoint.y, clampPoint.z], direction, transverse, normal, width);
				attach(clampId, new GantryBeltClamp(width, belt.thickness),
					{x: clampPoint.x, y: clampPoint.y, z: clampPoint.z, qx: plane.qx, qy: plane.qy, qz: plane.qz, qw: plane.qw}, clampParent);
				addBeltPath({belt: beltId, clamp: {instanceId: clampId, connectorName: "belt"},
					wraps: [{instanceId: pulleyId, connectorName: "attach-" + pulleyId},
						{instanceId: idlerId, connectorName: "attach-" + idlerId}]});
			case Rack(moduleSize, teeth, width):
				var face = [origin[0] - normal[0] * (shaft - width), origin[1] - normal[1] * (shaft - width), origin[2] - normal[2] * (shaft - width)];
				mountMotor(motorId, moving, AxisBuilder.orient(face[0], face[1], face[2], transverse, normal));
				var pinion = new SpurGear(moduleSize, teeth, width, SpurGear.STANDARD_PRESSURE_ANGLE, 0, 0, selectedMotor.variant.shaftDiameter);
				// At coordinate zero a pinion tooth meets a rack gap. Round the end margin
				// to full rack pitches to retain this phase for every module and travel.
				var rackPitch = Math.PI * moduleSize;
				var rackMargin = Math.ceil(railMargin / rackPitch) * rackPitch;
				var rackTeeth = Std.int(Math.ceil((axis.upper + 2 * rackMargin) / rackPitch));
				var start = point(-rackMargin);
				var rackPose = AxisBuilder.orient(start[0] - transverse[0] * pinion.pitchDiameter / 2 + normal[0] * width,
					start[1] - transverse[1] * pinion.pitchDiameter / 2 + normal[1] * width,
					start[2] - transverse[2] * pinion.pitchDiameter / 2 + normal[2] * width, transverse, direction);
				var pinionPose = AssemblyFrames.compose(AxisBuilder.orient(origin[0], origin[1], origin[2], transverse, normal),
					AssemblyFrames.fromRotationMatrix(0, 0, 0, [0.0, 1, 0, -1, 0, 0, 0, 0, 1]));
				driveRack(axis, "pinion" + suffix, motorId, pinion, pinionPose, normal,
					"rack" + suffix, new Rack(moduleSize, rackTeeth, width), rackPose, fixed, -1.0, addDriver);
		}
	}
}
