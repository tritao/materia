import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.component.ComponentDetail;
import materia.project.SceneArtifact;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Bench mill preview with a toe-clamped single-tool bearing-block job. */
class BenchMillPreview {
	public static inline var ASSEMBLY_ID:String = "bench-mill";
	public static inline var TARGET_PART:String = "machining-target";
	public static function mill():Bytes {
		var mill = new BenchMill();
		var scene = AssemblyPreview.scene(mill, ASSEMBLY_ID);
		var block = new BearingBlock(BenchMill.STOCK_WIDTH, BenchMill.STOCK_DEPTH, BenchMill.STOCK_HEIGHT);
		var part = block.geometry(ComponentDetail.Preview);
		var program = BearingBlockJob.program(mill, block, part);
		scene.parts.push(AssemblyPreview.part(TARGET_PART, "Finished bearing block", part, block.materialId, 0.002));
		scene.machining = {program: program, axes: ["x", "y", "z"], spindle: "spindle",
			workOffset: [for (v in mill.workOffset()) v / 1000],
			tools: [for (tool in mill.tools()) {number: tool.number, length: tool.length, profile: tool.profile().encode()}],
			stock: "stock", toolPart: "tool", loadedTool: 1, target: TARGET_PART, loop: true,
			controller: {stepTickHz: BenchMill.STEP_TICK_HZ}};
		return SceneArtifact.encode(scene);
	}
	/** Enclosed mill with its pressure-clamped vise and bearing-block job. */
	public static function enclosure():Bytes return enclosed();

	/** Enclosed vise job; final G53 moves bring its datum to the doorway. */
	public static function enclosed():Bytes {
		var cell = new EnclosedBenchMill(), mill = cell.mill;
		var scene = AssemblyPreview.scene(cell, ASSEMBLY_ID);
		var block = new BearingBlock(BenchMill.STOCK_WIDTH, BenchMill.STOCK_DEPTH, BenchMill.STOCK_HEIGHT);
		var part = block.geometry(ComponentDetail.Preview);
		var program = BearingBlockJob.program(mill, block, part, cell.loadPosition);
		scene.parts.push(AssemblyPreview.part(TARGET_PART, "Finished bearing block", part, block.materialId, 0.002));
		scene.machining = {program: program, axes: ["mill/x", "mill/y", "mill/z"], spindle: "mill/spindle",
			workOffset: [for (v in mill.workOffset()) v / 1000],
			tools: [for (tool in mill.tools()) {number: tool.number, length: tool.length, profile: tool.profile().encode()}],
			stock: "mill/stock", toolPart: "mill/tool", loadedTool: 1, target: TARGET_PART, loop: true,
			controller: {stepTickHz: BenchMill.STEP_TICK_HZ}, controllerId: "cncController"};
		return SceneArtifact.encode(scene);
	}

}

class BenchMillChecks {
	public static function run():Void {
		var mill = new BenchMill();
		var model = new AssemblyModel("mm");
		mill.addTo(model, "");
		var definition = model.definition(BenchMillPreview.ASSEMBLY_ID);
		if (definition.schemaVersion != materia.assembly.AssemblyDefinitionCodec.VERSION)
			throw "Bench mill did not select the process-velocity assembly schema";
		var spindleBinding = false, atSpeedSensor = false;
		if (definition.actuators != null) for (actuator in definition.actuators)
			if (actuator.processVelocity != null && actuator.processVelocity.speedChannel == "spindle.speed" &&
				actuator.processVelocity.directionChannel == "spindle.direction" &&
				Math.abs(actuator.processVelocity.radiansPerSpeedUnit - 2 * Math.PI / 60) < 1e-12)
				spindleBinding = true;
		if (definition.sensors != null) for (sensor in definition.sensors)
			if (sensor.kind == "at_speed" && sensor.joint == "spindle-turn" && sensor.windowLower != null &&
				Math.abs(sensor.windowLower - 0.02) < 1e-9 && sensor.hysteresis != null &&
				Math.abs(sensor.hysteresis - 0.01) < 1e-9) atSpeedSensor = true;
		if (!spindleBinding || !atSpeedSensor)
			throw "Spindle process channels or at-speed feedback were not derived into the assembly";
		var state = new AssemblyState(definition);
		var railSpec = machinekit.motion.LinearRailBlock.metric(BenchMill.RAIL).spec;
		var guides = [
			{axis: "y", rail: "railYLeft", carriage: "saddle", blocks: ["blockYLeft", "blockYLeftRear"], dx: 0.0, dy: 0.0, dz: 0.05},
			{axis: "y", rail: "railYRight", carriage: "saddle", blocks: ["blockYRight", "blockYRightRear"], dx: 0.0, dy: 0.0, dz: 0.05},
			{axis: "x", rail: "railXFront", carriage: "table", blocks: ["blockXFront", "blockXFrontRear"], dx: 0.0, dy: 0.0, dz: 0.05},
			{axis: "x", rail: "railXBack", carriage: "table", blocks: ["blockXBack", "blockXBackRear"], dx: 0.0, dy: 0.0, dz: 0.05},
			{axis: "z", rail: "railZLeft", carriage: "head", blocks: ["blockZLeft", "blockZLeftTop"], dx: 0.0, dy: -0.05, dz: 0.0},
			{axis: "z", rail: "railZRight", carriage: "head", blocks: ["blockZRight", "blockZRightTop"], dx: 0.0, dy: -0.05, dz: 0.0}
		];
		var observedRoom:Map<String, Float> = [];
		for (x in [0.0, 250]) for (y in [0.0, 150]) for (z in [-250.0, 0]) {
			state.setJoint("x", x); state.setJoint("y", y); state.setJoint("z", z);
			state.forwardKinematics();
			for (guide in guides) {
				var inverse = AssemblyFrames.inverse(state.worldConnector(guide.rail, "axis"));
				var rail = posed(mill, state, guide.rail);
				var railBounds = alongBounds(rail, inverse); rail.close();
				var previousUpper:Null<Float> = null;
				for (id in guide.blocks) {
					var block = posed(mill, state, id);
					var bounds = alongBounds(block, inverse); block.close();
					var room = Math.min(bounds.lower - railBounds.lower - railSpec.railEndMargin,
						railBounds.upper - railSpec.railEndMargin - bounds.upper);
					if (room < -1e-6) throw '$id leaves its usable rail at $x,$y,$z';
					var previousRoom = observedRoom.get(guide.axis);
					observedRoom.set(guide.axis, previousRoom == null ? room : Math.min(room, previousRoom));
					if (previousUpper != null && bounds.lower - previousUpper < BenchMill.BLOCK_GAP - 1e-6)
						throw '${guide.rail} blocks lack their assembly gap';
					previousUpper = bounds.upper;
				}
			}
			var gauge = state.worldConnector("spindle", "gaugeLine");
			var stock = state.worldPose("stock");
			if (Math.abs(gauge.z - mill.gaugeZero.z - z) > 1e-6 ||
				Math.abs(stock.x - (125 - x)) > 1e-6 || Math.abs(stock.y - (mill.gaugeZero.y + 75 - y)) > 1e-6)
				throw "Bench mill table/head FK disagrees with machine coordinates";
			for (pair in [["saddle", "column"], ["table", "column"], ["head", "column"],
				["table", "screwXFixed"], ["table", "screwXFloating"], ["saddle", "screwYFixed"],
				["saddle", "screwYFloating"], ["head", "screwZFixed"], ["head", "screwZFloating"],
				["screwXNut", "saddle"], ["screwXNut", "table"], ["screwYNut", "saddle"],
				["screwZNut", "column"], ["screwXNutBracket", "saddle"], ["screwXNutBracket", "table"],
				["screwYNutBracket", "saddle"], ["screwZNutBracket", "head"], ["screwZNutBracket", "column"],
				["motorXPlate", "screwXNearBridge"], ["motorYPlate", "motorYBridge"],
				["motorZPlate", "motorZPost"], ["screwXCoupling", "motorXPlate"], ["screwYCoupling", "motorYPlate"],
				["screwZCoupling", "motorZPlate"], ["spindleMotor", "spindleMotorPulley"], ["spindleMotorShelf", "spindleMotorPulley"], ["spindle", "stock"], ["holder", "clampLeft"], ["holder", "clampRight"]]) {
				var a = posed(mill, state, pair[0]), b = posed(mill, state, pair[1]);
				var common = a.intersect(b), volume = common.volume();
				common.close(); a.close(); b.close();
				if (volume > 1e-3) throw '${pair[0]} overlaps ${pair[1]} at $x,$y,$z by $volume mm³';
			}
			for (index in 0...3) {
				var id = "screw" + mill.specs[index].id.toUpperCase();
				var expected = 2 * Math.PI / mill.axisDrives[index].screw.thread.lead * [x, y, z][index];
				if (Math.abs(state.joint(id + "-turn") - expected) > 1e-6) throw '$id ratio is wrong';
			}
		}
		for (spec in mill.specs) {
			var measured = observedRoom.get(spec.id);
			if (measured == null || Math.abs(measured - mill.axisOvertravel(spec.id)) > 1e-5)
				throw '${spec.id} overtravel disagrees with the actual posed block and rail bounds';
		}
		for (guide in guides) for (id in guide.blocks)
			if (mountOverlap(mill, state, id, guide.carriage, guide.dx, guide.dy, guide.dz) < 1)
				throw '$id lacks finite contact with ${guide.carriage}';
		var saddleSolid = mill.saddle.geometry(), lowerWebProbe = Part.box(10, 10, 5);
		var lowerWeb = saddleSolid.intersect(lowerWebProbe);
		var lowerWebVolume = lowerWeb.volume();
		lowerWeb.close(); lowerWebProbe.close(); saddleSolid.close();
		if (lowerWebVolume < 200) throw "The Y screw bore severed the saddle's lower load web";
		if (mountOverlap(mill, state, "screwXNutBracket", "table", 0, 0, 0.05) < 1 ||
			mountOverlap(mill, state, "screwYNutBracket", "saddle", 0, -0.05, 0) < 1)
			throw "The X/Y nut brackets lack finite bearing faces on their moving castings";
		for (drive in mill.axisDrives) {
			var spec = drive.spec, screw = drive.screw, motor = drive.motor;
			if (!(mill.axisOvertravel(spec.id) > 0)) throw "Mill rail has no overtravel";
			var suffix = spec.id.toUpperCase();
			var couplingId = "screw" + suffix + "Coupling", shaftId = "motor" + suffix + "Shaft";
			var sleeve = state.worldPose(couplingId);
			var axis = AssemblyFrames.transformVector(sleeve, 0, 0, 1);
			var face = state.worldConnector(couplingId, "sideA"), midpoint = state.worldConnector(couplingId, "axis");
			var shaftEnd = state.worldConnector(shaftId, "output"), shaftStart = state.worldConnector(shaftId, "input");
			var depth = (shaftEnd.x - face.x) * axis.x + (shaftEnd.y - face.y) * axis.y + (shaftEnd.z - face.z) * axis.z;
			var start = (shaftStart.x - face.x) * axis.x + (shaftStart.y - face.y) * axis.y + (shaftStart.z - face.z) * axis.z;
			var dx = shaftEnd.x - midpoint.x, dy = shaftEnd.y - midpoint.y, dz = shaftEnd.z - midpoint.z;
			if (start > 1e-6 || depth <= 5 || dx * dx + dy * dy + dz * dz > 1e-12)
				throw '${spec.id} motor shaft does not seat into its coupling';
			var rapid = Math.min(screw.criticalSpeed(Fixed, Simple), motor.rating.maxSpeed) * screw.thread.lead / (2 * Math.PI);
			if (rapid < 8000.0 / 60) throw '${spec.id} rapid is below 8 m/min';
			trace('Mill ${spec.id}: ${rapid * 60 / 1000} m/min source-derived rapid, ${mill.axisOvertravel(spec.id)} mm overtravel');
		}
		var scene = SceneArtifact.decode(BenchMillPreview.mill());
		var job = scene.machining;
		if (job == null) throw "Mill preview has no job";
		var controller = new cnckit.CncController();
		for (tool in mill.tools()) controller.toolLibrary.set(tool);
		var offset = mill.workOffset();
		controller.setWorkOffset(54, offset[0] / 1000, offset[1] / 1000, offset[2] / 1000);
		var compiled = cnckit.CncCompiler.compileDetailed(job.program, controller,
			new toolpathkit.path.Point3(mill.specs[0].initial / 1000, mill.specs[1].initial / 1000, mill.specs[2].initial / 1000));
		if (compiled.diagnostics.length != 0) throw 'Mill exported G-code does not recompile: ${compiled.diagnostics}';
		var block = new BearingBlock(BenchMill.STOCK_WIDTH, BenchMill.STOCK_DEPTH, BenchMill.STOCK_HEIGHT);
		var solid = block.geometry();
		if (Math.abs(solid.volume() - (block.width * block.depth * block.height - block.removedVolume())) > 1)
			throw "Bearing block removal disagrees with its solid";
		// Material witnesses independently check the actual cut walls, rather than their construction parameters.
		for (sample in [{x: 0.0, y: -28.5, width: 12.0, depth: 3.0},
			{x: 0.0, y: -13.0, width: 2.0, depth: 3.0}]) {
			var witness = Part.box(sample.width, sample.depth, 3).translated(new Vector(sample.x, sample.y, 17));
			var material = solid.intersect(witness);
			if (Math.abs(material.volume() - witness.volume()) > 0.001)
				throw "Bearing-block machining leaves less than 3 mm of material at its front or seat ligament";
			material.close(); witness.close();
		}
		solid.close();
		if (mill.billOfMaterials().lines().length < 20) throw "Mill BOM lacks its drive parts";
		trace('Bench mill geometry and CNC job passed (${mill.components().length} members)');
	}
	public static function posed(mill:machinekit.assembly.MachineAssembly, state:AssemblyState, id:String):Part {
		for (entry in mill.components()) if (entry.id == id) {
			var pose = state.worldPose(id);
			var x = AssemblyFrames.transformVector(pose, 1, 0, 0), z = AssemblyFrames.transformVector(pose, 0, 0, 1);
			var local = entry.component.geometry();
			var part = local.placed(new Location(new Plane(new Vector(pose.x, pose.y, pose.z),
				new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z))));
			local.close(); return part;
		}
		throw 'Mill has no part "$id"';
	}
	static function alongBounds(part:Part, inverse:AssemblyFrame):{lower:Float, upper:Float} {
		var box = part.shape.bounds(), lo = box.get_min(), hi = box.get_max();
		// Mechanical connector frames use +Y as their axis, including the rail's axial connector.
		var first = AssemblyFrames.transformPoint(inverse, lo.get_x(), lo.get_y(), lo.get_z()).y;
		var lower = first, upper = first;
		for (x in [lo.get_x(), hi.get_x()]) for (y in [lo.get_y(), hi.get_y()]) for (z in [lo.get_z(), hi.get_z()]) {
			var along = AssemblyFrames.transformPoint(inverse, x, y, z).y;
			lower = Math.min(lower, along); upper = Math.max(upper, along);
		}
		return {lower: lower, upper: upper};
	}
	static function mountOverlap(mill:machinekit.assembly.MachineAssembly, state:AssemblyState,
			bracket:String, carriage:String, dx:Float, dy:Float, dz:Float):Float {
		var mounted = posed(mill, state, bracket), body = posed(mill, state, carriage);
		var pressed = mounted.translated(new Vector(dx, dy, dz));
		var common = pressed.intersect(body), volume = common.volume();
		common.close(); pressed.close(); mounted.close(); body.close();
		return volume;
	}

}

/** Standalone mechanical and CAM checks, also called by MachineKit smoke. */
function main():Void { BenchMillChecks.run(); EnclosedMillChecks.run(); }

/** Clear openings and a datum taken from the actual vise parts. */
class EnclosedMillChecks {
	public static function run():Void {
		var cell = new EnclosedBenchMill(), state = cell.state();
		var deck = BenchMillChecks.posed(cell, state, "standDeck");
		var tray = BenchMillChecks.posed(cell, state, "chipTray");
		var db = deck.shape.bounds(), tb = tray.shape.bounds();
		if (Math.abs(db.get_min().get_x() - tb.get_min().get_x()) > 1e-5 ||
			Math.abs(db.get_max().get_x() - tb.get_max().get_x()) > 1e-5 ||
			Math.abs(db.get_min().get_y() - tb.get_min().get_y()) > 1e-5 ||
			Math.abs(db.get_max().get_y() - tb.get_max().get_y()) > 1e-5 ||
			Math.abs(db.get_max().get_z() - tb.get_min().get_z()) > 1e-5)
			throw "Stand deck must support the complete chip tray at its underside";
		deck.close(); tray.close();
		for (id in cell.standIds) if (StringTools.startsWith(id, "standLeg")) {
			var leg = BenchMillChecks.posed(cell, state, id), bounds = leg.shape.bounds();
			if (Math.abs(bounds.get_min().get_z()) > 1e-5 ||
				Math.abs(bounds.get_max().get_z() - (cell.standHeight - 8)) > 1e-5)
				throw "Stand legs must reach from the floor to the deck";
			var expected = (80 * 80 - 72 * 72) * (cell.standHeight - 8);
			if (Math.abs(leg.volume() - expected) > 0.01)
				throw "Stand leg mass must come from hollow structural stock";
			leg.close();
		}
		if (cell.standIds.length != 15) throw "Stand needs a deck, four legs, eight rails and two rear braces";
		// The X guide preserves the leaf's Y interval, so this separating plane
		// proves panel clearance for the entire continuous stroke.
		var guide = [for (joint in state.definition.joints) if (joint.id == "door") joint][0];
		if (Math.abs(guide.axis.y) > 1e-12 || Math.abs(guide.axis.z) > 1e-12) throw "Door guide is not along X";
		for (door in cell.doorIds) for (panel in cell.panelIds) {
			var a = BenchMillChecks.posed(cell, state, door), b = BenchMillChecks.posed(cell, state, panel);
			var leafBack = a.shape.bounds().get_max().get_y(), panelFront = b.shape.bounds().get_min().get_y();
			a.close(); b.close();
			if (leafBack >= panelFront) throw '$door has no invariant separating plane from $panel';
		}
		for (step in 0...11) {
			state.setJoint("door", cell.doorStroke * step / 10);
			for (door in cell.doorIds) for (panel in cell.panelIds) {
				var a = BenchMillChecks.posed(cell, state, door), b = BenchMillChecks.posed(cell, state, panel);
				var overlap = a.intersect(b), volume = overlap.volume();
				overlap.close(); a.close(); b.close();
				if (volume > 0.001) throw '$door overlaps $panel at door step $step';
			}
		}
		var leaf = BenchMillChecks.posed(cell, state, "doorLeft");
		var leafBounds = leaf.shape.bounds();
		var openLeft = leafBounds.get_min().get_x(); leaf.close();
		if (openLeft <= cell.opening.x + cell.openingWidth / 2)
			throw "Open door does not clear the opening";
		state.setJoint("mill/z", cell.mill.specs[2].upper);
		var head = BenchMillChecks.posed(cell, state, "mill/head");
		var headBottom = head.shape.bounds().get_min().get_z();
		head.close();
		if (headBottom < cell.opening.z + cell.openingHeight) throw "Retracted head casting crosses the doorway";
		var vise = cell.mill.vise;
		if (vise == null) throw "Enclosed mill has no vise";
		var cabinet = BenchMillChecks.posed(cell, state, "cabinetBottom");
		for (x in [-1, 1]) for (y in [-1, 1]) {
			var id = 'cabinetLeg${x + 1}${y + 1}';
			var leg = BenchMillChecks.posed(cell, state, id), bounds = leg.shape.bounds();
			if (Math.abs(bounds.get_min().get_z()) > 1e-6 ||
				Math.abs(bounds.get_max().get_z() - cabinet.shape.bounds().get_min().get_z()) > 1e-6)
				throw "Cabinet legs must reach the floor and the cabinet bottom";
			var pressed = leg.translated(new Vector(0, 0, 0.1)), contact = pressed.intersect(cabinet);
			if (contact.volume() < 20) throw "Cabinet leg has no supporting contact with its bottom";
			if (Math.abs(leg.volume() - 168000) > 0.01) throw "Cabinet leg must retain its hollow section";
			contact.close(); pressed.close(); leg.close();
		}
		cabinet.close();
		var processDefinition = state.definition;
		if (processDefinition.schemaVersion != materia.assembly.AssemblyDefinitionCodec.VERSION)
			throw "Enclosed mill did not keep the process-velocity schema";
		if (processDefinition.actuators == null || processDefinition.sensors == null)
			throw "Enclosed mill did not export its typed actuator and sensor tables";
		var doorForce:Null<Float> = null, viseForce:Null<Float> = null;
		for (actuator in processDefinition.actuators) {
			if (actuator.id == "door-cylinder") doorForce = actuator.maxEffort;
			if (actuator.id == "vise-cylinder") viseForce = actuator.maxEffort;
		}
		if (doorForce == null || viseForce == null || Math.abs(doorForce - 294.524311274) > 0.01 ||
			Math.abs(viseForce - 482.54863159) > 0.01)
			throw 'Enclosed cylinder models do not retain their newton ratings: $doorForce/$viseForce N';
		var seenDoorClosed = false, seenDoorOpen = false, seenViseClamp = false, seenStock = false;
		for (sensor in processDefinition.sensors) {
			if (sensor.id == "door-closed" && sensor.kind == "joint_switch") seenDoorClosed = true;
			if (sensor.id == "door-open" && sensor.kind == "joint_switch") seenDoorOpen = true;
			if (sensor.id == "vise-clamped" && sensor.kind == "joint_switch" &&
				sensor.windowLower != null && sensor.windowUpper != null) {
				var grip = vise.grippedPosition;
				seenViseClamp = sensor.windowLower < grip && grip < sensor.windowUpper &&
					sensor.windowUpper < vise.stroke;
			}
			if (sensor.id == "stock-present" && sensor.kind == "presence") seenStock = true;
		}
		if (!seenDoorClosed || !seenDoorOpen || !seenViseClamp || !seenStock)
			throw "Enclosed mill did not preserve its door, vise and part sensors";
		var occurrences = state.definition.occurrences;
		var viseCylinder = [for (occurrence in occurrences) if (occurrence.id == "mill/vise/cylinder") occurrence];
		var viseRod = [for (occurrence in occurrences) if (occurrence.id == "mill/vise/clampRod") occurrence];
		var duplicateViseCylinder = [for (occurrence in occurrences)
			if (StringTools.endsWith(occurrence.id, "viseCylinder") || occurrence.id == "viseCylinder") occurrence];
		if (viseCylinder.length != 1 || viseRod.length != 1 || duplicateViseCylinder.length != 0)
			throw "The vise must own the only clamp cylinder body and rod";
		var connections = cell.describe().machine.portConnections;
		var viseAirA = false, viseAirB = false;
		var visePortA = vise.port("airA", "mill/vise");
		var visePortB = vise.port("airB", "mill/vise");
		for (connection in connections) {
			if (connection.toInstance == visePortA.instanceId && connection.toPort == visePortA.portName &&
				connection.fromInstance == "viseValve" && connection.fromPort == "A") viseAirA = true;
			if (connection.toInstance == visePortB.instanceId && connection.toPort == visePortB.portName &&
				connection.fromInstance == "viseValve" && connection.fromPort == "B") viseAirB = true;
		}
		if (!viseAirA || !viseAirB) throw "The enclosure valve must connect to the vise's exposed A/B air ports";
		state.setJoint("mill/vise/jaw", 0);
		var fixedFace = state.worldPose("mill/vise/fixedJaw").y + vise.fixedJaw.depth / 2;
		var movingFace = state.worldPose("mill/vise/movingJaw").y - vise.movingJaw.depth / 2;
		if (Math.abs((movingFace - fixedFace - BenchMill.STOCK_DEPTH) / 2 - vise.clearance) > 1e-9)
			throw "Actual jaw faces do not give the loading blank clearance";
		state.setJoint("mill/vise/jaw", vise.stroke);
		var closedFace = state.worldPose("mill/vise/movingJaw").y - vise.movingJaw.depth / 2;
		if (Math.abs(movingFace - closedFace - vise.stroke) > 1e-9) throw "Jaw guide does not close the opening";
		for (spec in cell.mill.specs) state.setJoint("mill/" + spec.id, 0);
		var datum = state.worldConnector("mill/vise/body", "datum");
		var stop = state.worldPose("mill/vise/endStop"), parallel = state.worldPose("mill/vise/parallel0");
		fixedFace = state.worldPose("mill/vise/fixedJaw").y + vise.fixedJaw.depth / 2;
		if (Math.abs(datum.x - stop.x - vise.endStop.width / 2) > 1e-9 || Math.abs(datum.y - fixedFace) > 1e-9 ||
			Math.abs(datum.z - parallel.z - vise.parallels.height) > 1e-9) throw "Datum does not lie on the actual locating faces";
		var offset = cell.mill.workOffset();
		var probeController = new cnckit.CncController();
		probeController.setWorkOffset(54, offset[0] / 1000, offset[1] / 1000, offset[2] / 1000);
		var probe = cnckit.CncCompiler.compileDetailed("G21 G90 G54\nG0 X0 Y0 Z0\nM2",
			probeController, new toolpathkit.path.Point3(0, 0, 0));
		if (probe.diagnostics.length != 0) throw "G54 datum probe does not compile";
		var commanded:Null<toolpathkit.path.Point3> = null;
		var probeFrame = toolpathkit.path.ToolpathFrame.of(probe.program);
		for (op in probe.program.ops) {
			probeFrame.advance(op);
			switch op {
				case Move(_, geometry, _, _, _):
					var machineGeometry = toolpathkit.path.GeometryOffset.translate(geometry, probeFrame.toMachine());
					commanded = toolpathkit.path.GeometryTools.pointAt(machineGeometry, toolpathkit.path.GeometryTools.length(machineGeometry));
				case MachineMove(_, geometry, _, _, _):
					commanded = toolpathkit.path.GeometryTools.pointAt(geometry, toolpathkit.path.GeometryTools.length(geometry));
				case _:
			}
		}
		if (commanded == null) throw "G54 datum probe produced no machine move";
		state.setJoint("mill/x", commanded.x * 1000);
		state.setJoint("mill/y", commanded.y * 1000);
		state.setJoint("mill/z", commanded.z * 1000);
		var probeDatum = state.worldConnector("mill/vise/body", "datum");
		var probeGauge = state.worldConnector("mill/spindle", "gaugeLine");
		if (Math.abs(probeDatum.x - probeGauge.x) > 1e-6 || Math.abs(probeDatum.y - probeGauge.y) > 1e-6 ||
			Math.abs(probeDatum.z + BenchMill.STOCK_HEIGHT - probeGauge.z) > 1e-6)
			throw "Compiled G54 zero does not align the spindle gauge with the seated stock datum";
		for (spec in cell.mill.specs) state.setJoint("mill/" + spec.id, 0);
		var scene = SceneArtifact.decode(BenchMillPreview.enclosed());
		var job = scene.machining;
		if (job == null) throw "Enclosed mill has no CNC job";
		var controller = new cnckit.CncController();
		for (tool in cell.mill.tools()) controller.toolLibrary.set(tool);
		controller.setWorkOffset(54, offset[0] / 1000, offset[1] / 1000, offset[2] / 1000);
		var compiled = cnckit.CncCompiler.compileDetailed(job.program, controller,
			new toolpathkit.path.Point3(cell.mill.specs[0].initial / 1000,
				cell.mill.specs[1].initial / 1000, cell.mill.specs[2].initial / 1000));
		if (compiled.diagnostics.length != 0) throw "Enclosed CNC program does not compile";
		var machineMoves = [for (op in compiled.program.ops) switch op {
			case MachineMove(_, geometry, _, _, _): geometry;
			case _: null;
		}];
		var end:Null<toolpathkit.path.Point3> = null;
		for (geometry in machineMoves) if (geometry != null)
			end = toolpathkit.path.GeometryTools.pointAt(geometry, toolpathkit.path.GeometryTools.length(geometry));
		if (end == null || Math.abs(end.x - cell.loadPosition[0] / 1000) > 1e-9 ||
			Math.abs(end.y - cell.loadPosition[1] / 1000) > 1e-9 || Math.abs(end.z - cell.loadPosition[2] / 1000) > 1e-9)
			throw "Final G53 does not command the geometry-derived load position";
		var definition = scene.assemblyDefinition;
		if (definition == null) throw "Enclosed CNC preview has no typed assembly definition";
		for (id in cell.panelIds.concat(cell.doorIds)) {
			var occurrence = [for (o in definition.occurrences) if (o.id == id) o][0];
			var part = [for (p in scene.parts) if (p.id == occurrence.definition) p][0];
			var hull = cadkit.ConvexHullVertices.enclosingFromMesh(part.vertices, part.vertexCount, 0.1, 0);
			if (hull.warning != null || hull.errorRatio > 0.05) throw '$id has an inaccurate collision hull';
		}
		trace('Enclosed mill: bounds ${cell.envelope.low}..${cell.envelope.high}, opening ${cell.opening.x},${cell.opening.y},${cell.opening.z}, load ${cell.loadPosition}, G54 $offset');
	}
	public static function main():Void run();
}
