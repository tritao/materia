import machinekit.assembly.PosedParts;
import machinekit.assembly.AssemblyPreview;
import haxe.io.Bytes;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.motion.LinearRail;
import machinekit.motion.NemaStepper;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

/** Materia project entrypoint for the desktop CNC router. */
class CncRouterPreview {
	public static inline var ASSEMBLY_ID:String = "cnc-router";

	/** Id of the finished motor plate in the scene: the machining target, not a part of the machine. */
	public static inline var TARGET_PART:String = "machining-target";
	/**
	 * Largest gap between the target's mesh and the plate, in millimetres: a tenth of the 0.02 mm the
	 * app counts as on target, so the mesh's chords on round walls do not read as cuts into the part.
	 */
	static inline var TARGET_DEFLECTION:Float = 0.002;

	/**
	 * Geometry, joints and initial pose of the router (parts with equal designations share geometry),
	 * and its machining job: CAM made from the motor plate it mills, with the plate as the target.
	 */
	public static function router(belts:Bool = false, foldedZ:Bool = false):Bytes {
		var router = new CncRouter(belts, foldedZ);
		var scene = AssemblyPreview.scene(router, ASSEMBLY_ID);
		var plate = motorPlate();
		var part = plate.geometry(ComponentDetail.Preview);
		var program = MountPlateJob.program(router, plate, part);
		scene.parts.push(AssemblyPreview.part(TARGET_PART, "Finished motor plate", part, plate.materialId, TARGET_DEFLECTION));
		scene.machining = {program: program, axes: ["x", "y", "z"], spindle: "spindle",
			workOffset: [for (value in CncRouter.workOffset()) value / 1000],
			tools: [for (tool in router.tools()) {number: tool.number, length: tool.length, profile: tool.profile().encode()}],
			stock: "stock", sacrificial: ["spoilboard"], toolPart: "tool", loadedTool: 1,
			target: TARGET_PART, loop: true,
			controller: {stepTickHz: CncRouter.STEP_TICK_HZ}};
		return SceneArtifact.encode(scene);
	}

	/** The same router with belts on X and Y instead of lead screws. */
	public static function beltRouter():Bytes return router(true);

	/** The same screw router with the Z motor folded below the gantry through a 2:1 belt. */
	public static function foldedZRouter():Bytes return router(false, true);

	/** The part the router mills from its stock block. */
	public static function motorPlate():NemaMountPlate
		return new NemaMountPlate(NemaStepper.frame(23), CncRouter.STOCK_WIDTH, CncRouter.STOCK_DEPTH, CncRouter.STOCK_HEIGHT, 6);
}

/** Geometry builds, the axes move the tool where machine coordinates say, and nothing collides. */
class CncRouterChecks {
	public static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void {
		if (!(Math.abs(actual - expected) <= tolerance))
			throw '$message: expected $expected, got $actual';
	}

	public static function run():Void withGeometry(runChecks);

	/** Own the geometry cache for checks invoked from either standalone fixture. */
	public static function withGeometry(check:Void -> Void):Void {
		parts = new PosedParts();
		try check() catch (error:Dynamic) {
			parts.close(); parts = null;
			throw error;
		}
		parts.close(); parts = null;
	}

	/** The geometry of the members posed by this run, built once each; closed when the run ends. */
	static var parts:PosedParts;

	static function runChecks():Void {
		var scene = SceneArtifact.decode(CncRouterPreview.router());
		var definition = scene.assemblyDefinition;
		var router = new CncRouter();
		if (definition == null || definition.occurrences.length != router.components().length)
			throw "CNC router preview has the wrong number of occurrences";
		for (part in scene.parts) if (part.volume == null || part.volume <= 0 || part.inertia == null)
			throw 'CNC router part "${part.id}" has no mass properties';
		var prismatic = [for (joint in definition.joints) if (Std.string(joint.type) == "prismatic") joint.id];
		if (prismatic.join(",") != "y,x,z") throw 'CNC router should have prismatic joints y, x, z, got $prismatic';

		var model = new AssemblyModel("mm");
		router.addTo(model, "");
		var state = new AssemblyState(model.definition(CncRouterPreview.ASSEMBLY_ID));
		var fixed = ["sideLeft", "railYRight", "spoilboard", "stock", "clampLeft", "motorYRight"];
		var fixedPoses = [for (id in fixed) state.worldPose(id)];
		// Corners of the travel box and a point inside it.
		var positions = [[150.0, 150, 0], [0.0, 0, 0], [300.0, 300, -80], [0.0, 300, -80], [300.0, 0, -40], [37.5, 212, -12.5]];
		for (position in positions) {
			state.setJoint("x", position[0]);
			state.setJoint("y", position[1]);
			state.setJoint("z", position[2]);
			state.forwardKinematics();
			var expected = CncRouter.noseAt(position[0], position[1], position[2]);
			var nose = state.worldConnector("spindle", "nose"), tip = state.worldConnector("tool", "tip");
			var label = 'spindle nose at machine ${position.join(", ")}';
			near(nose.x, expected.x, '$label, x', 1e-6);
			near(nose.y, expected.y, '$label, y', 1e-6);
			near(nose.z, expected.z, '$label, z', 1e-6);
			near(tip.z, expected.z - router.tool.stickout, '$label: tool 1 tip is its length below', 1e-6);
			near(AssemblyFrames.transformVector(tip, 0, 1, 0).z, 1, '$label: tool axis points up', 1e-9);
			for (i in 0...fixed.length) {
				var pose = state.worldPose(fixed[i]), start = fixedPoses[i];
				near(pose.x + pose.y + pose.z, start.x + start.y + start.z, '${fixed[i]} stays put', 1e-9);
			}
			checkBlocksOnRails(router, state);
			// Tr10 × 2 right-hand screws pointing against their axes: half a turn per millimetre.
			for (screw in [{id: "screwX", axis: 0}, {id: "screwYLeft", axis: 1}, {id: "screwYRight", axis: 1}, {id: "screwZ", axis: 2}])
				near(state.joint(screw.id + "-turn"), Math.PI * position[screw.axis], '$label: ${screw.id} turns with its axis', 1e-9);
		}

		// Z reaches through the stock into the spoilboard, and at the top of travel the tool clears the clamps.
		var longest = Math.max(router.tool.stickout, router.drill.stickout);
		var throughCut = CncRouter.SPOILBOARD_TOP + router.tool.stickout - CncRouter.MACHINE_ZERO_Z;
		if (!(throughCut >= router.specs[2].lower)) throw "Z should reach through the stock";
		if (!(CncRouter.noseAt(0, 0, 0).z - longest > CncRouter.STOCK_TOP + 25))
			throw "at the top of Z every tool should clear the clamps";
		var tools = router.tools();
		if (tools.length != 2 || Math.abs(tools[0].length - router.tool.stickout / 1000) > 1e-12 ||
				Math.abs(tools[1].profile().cuttingDiameter() - router.drill.diameter / 1000) > 1e-12)
			throw "the tool table should list the end mill and the drill by their lengths and shapes";
		// Cutting through the stock, the Z slide and spindle stay clear of it, and the flutes span it.
		if (!(router.tool.fluteLength >= CncRouter.STOCK_HEIGHT)) throw 'the end mill flutes (${router.tool.fluteLength} mm, stickout ${router.tool.stickout}, d ${router.tool.diameter}) should span the ${CncRouter.STOCK_HEIGHT} mm stock';
		var moving = ["zPlate", "spindleClamp", "spindle", "blockZLeft", "blockZRight"];
		checkClear(router, state, [150, 150, throughCut], moving, ["stock", "clampLeft", "clampRight", "spoilboard"]);
		checkClear(router, state, [150, 150, router.specs[2].lower], moving, ["clampLeft", "clampRight", "spoilboard"]);
		// The tool is the one part meant to meet the stock.
		if (overlap(router, state, [150, 150, -60], "tool", "stock") <= 0) throw "at depth the tool should be in the stock";
		// Gantry and carriages at the ends of travel.
		var gantry = ["uprightLeft", "uprightRight", "nutBracketYLeft", "nutBracketYRight", "beamUpper", "beamLower",
			"blockYLeft", "blockYRight", "motorX"];
		var frame = ["motorPlateYLeft", "motorPlateYRight", "motorYLeft", "motorYRight", "sideLeft", "sideRight",
			"railYLeft", "railYRight", "crossBack", "crossFront", "spoilboard"];
		checkClear(router, state, [300, 300, 0], gantry, frame);
		checkClear(router, state, [0, 0, 0], gantry, frame);
		var carriage = ["xPlate", "nutBracketX", "motorBracketZ", "motorZ", "zPlate", "blockXUpper", "blockXLower"];
		checkClear(router, state, [0, 0, 0], carriage, ["uprightLeft", "beamUpper", "beamLower", "screwX"]);
		checkClear(router, state, [300, 0, 0], carriage, ["uprightRight", "motorX", "beamUpper", "beamLower"]);
		checkClear(router, state, [150, 150, 0], ["zPlate", "spindle", "spindleClamp"], ["motorBracketZ", "motorZ", "xPlate", "screwZ"]);
		checkClear(router, state, [150, 150, -80], ["zPlate", "spindle", "spindleClamp"], ["motorBracketZ", "motorZ", "xPlate", "screwZ"]);

		// Each screw's turn is a lead-screw drive, so its thread sets the ratio.
		checkClear(router, state, [150, 150, 0], ["screwZNut"], ["xPlate", "zPlate", "motorBracketZ"]);
		checkClear(router, state, [150, 150, -80], ["screwZNut"], ["xPlate", "zPlate", "motorBracketZ"]);
		for (screw in ["screwX", "screwYLeft", "screwYRight", "screwZ"]) {
			var drive = router.transmissionFor(screw + "-lead");
			if (drive == null || !switch drive.source { case LeadScrew(id, nut): id == screw && nut == screw + "Nut"; default: false; })
				throw '$screw should turn through a lead-screw drive';
		}
		// Each shaft coupling turns inside its mount's pilot bore, clear of the motor and the mount.
		checkClear(router, state, [150, 150, 0], ["screwYLeftCoupling"], ["motorPlateYLeft", "motorYLeft"]);
		checkClear(router, state, [150, 150, 0], ["motorYLeft"], ["motorPlateYLeft"]);
		checkClear(router, state, [150, 150, 0], ["motorX"], ["uprightRight"]);
		checkClear(router, state, [150, 150, 0], ["motorZ", "screwZCoupling"], ["motorBracketZ", "standoffZ1", "standoffZ2", "standoffZ3", "standoffZ4"]);
		checkClear(router, state, [150, 150, 0], ["screwYRightCoupling"], ["motorPlateYRight"]);
		checkClear(router, state, [0, 0, 0], ["screwXCoupling"], ["uprightRight", "xPlate", "nutBracketX"]);
		checkClear(router, state, [300, 0, -80], ["screwXCoupling"], ["uprightRight", "xPlate", "nutBracketX"]);
		checkClear(router, state, [150, 150, 0], ["screwZCoupling"], ["motorBracketZ", "zPlate", "blockZLeft", "blockZRight"]);
		checkClear(router, state, [150, 150, -80], ["screwZCoupling"], ["motorBracketZ", "zPlate"]);
		// Every axis has room past its travel before its blocks reach their rail ends.
		near(router.axisOvertravel("x"), 62.65, "x overtravel", 1e-6);
		near(router.axisOvertravel("y"), 6.65, "y overtravel", 1e-6);
		near(router.axisOvertravel("z"), 0.5, "z overtravel", 1e-6);
		// The part the router machines: its recesses' closed-form volume is what the solid loses.
		var plate = CncRouterPreview.motorPlate();
		var solid = plate.geometry(ComponentDetail.Preview);
		var block = CncRouter.STOCK_WIDTH * CncRouter.STOCK_DEPTH * CncRouter.STOCK_HEIGHT;
		near(solid.volume(), block - plate.removedVolume(), "the motor plate is the block less its recesses and holes", 1.0);
		solid.close();
		var bom = router.billOfMaterials().lines();
		var motors = [for (entry in router.components()) if (Std.isOfType(entry.component, NemaStepper)) entry.id];
		if (motors.length != 4) throw 'CNC router should have four stepper motors, got $motors';
		var mass = router.massProperties().mass;
		Sys.println('cnc router: ${scene.parts.length} definitions, ${definition.occurrences.length} occurrences, ' +
			'${bom.length} BOM lines, ${Math.round(mass * 10) / 10} kg');
		checkXHomeClearance(router);
		runBelts();
	}


	/**
	 * The belt-driven router: its pulleys turn travel over their pitch radius (and the right way, by
	 * the rule of a belt's strand), its belts are whole teeth long, clear the frame and are clamped on
	 * exactly one strand each, and its motors, plates and idlers clear the gantry.
	 */
	static function runBelts():Void {
		var router = new CncRouter(true);
		var scene = AssemblyPreview.scene(router, CncRouterPreview.ASSEMBLY_ID);
		var definition = scene.assemblyDefinition;
		if (definition == null || definition.occurrences.length != router.components().length)
			throw "belt router preview has the wrong number of occurrences";
		var prismatic = [for (joint in definition.joints) if (Std.string(joint.type) == "prismatic") joint.id];
		if (prismatic.join(",") != "y,x,z") throw 'belt router should have prismatic joints y, x, z, got $prismatic';
		var motors = [for (entry in router.components()) if (Std.isOfType(entry.component, NemaStepper)) entry.id];
		if (motors.length != 4) throw 'belt router should have four stepper motors, got $motors';

		var model = new AssemblyModel("mm");
		router.addTo(model, "");
		var state = new AssemblyState(model.definition(CncRouterPreview.ASSEMBLY_ID));
		var radius = 2 * 20 / (2 * Math.PI);
		// Each pulley's drive is a belt drive of its pulley, and turns the way a belt clamped on the lower
		// strand makes it: the strand's point on the pulley moves along the axis with the carriage.
		var pulleys = [{id: "pulleyX", axis: 0, direction: [1.0, 0, 0]}, {id: "idlerX", axis: 0, direction: [1.0, 0, 0]},
			{id: "pulleyYLeft", axis: 1, direction: [0.0, 1, 0]}, {id: "idlerYLeft", axis: 1, direction: [0.0, 1, 0]},
			{id: "pulleyYRight", axis: 1, direction: [0.0, 1, 0]}, {id: "idlerYRight", axis: 1, direction: [0.0, 1, 0]}];
		for (pulley in pulleys) {
			var drive = beltDrive(router, pulley.id);
			var joint = [for (candidate in definition.joints) if (candidate.id == pulley.id + "-turn") candidate][0];
			// The lower strand's point on the pulley is straight below its axis.
			// Turning by about moves a point straight below the axis by about x (0, 0, -1) = (-about.y, about.x, 0).
			var about = joint.axis;
			var along = -about.y * pulley.direction[0] + about.x * pulley.direction[1];
			near(machinekit.assembly.Sense.SenseTools.sign(drive.sense), along, '${pulley.id} turns the way its lower strand moves', 1e-9);
		}
		var positions = [[150.0, 150, 0], [0.0, 0, 0], [300.0, 300, -80], [0.0, 300, -80], [300.0, 0, -40], [37.5, 212, -12.5]];
		for (position in positions) {
			state.setJoint("x", position[0]);
			state.setJoint("y", position[1]);
			state.setJoint("z", position[2]);
			state.forwardKinematics();
			var expected = CncRouter.noseAt(position[0], position[1], position[2]);
			var nose = state.worldConnector("spindle", "nose");
			near(nose.x, expected.x, 'belt router nose, x', 1e-6);
			near(nose.y, expected.y, 'belt router nose, y', 1e-6);
			near(nose.z, expected.z, 'belt router nose, z', 1e-6);
			for (pulley in pulleys) {
				var drive = beltDrive(router, pulley.id);
				var turned = state.joint(pulley.id + "-turn");
				near(turned, machinekit.assembly.Sense.SenseTools.sign(drive.sense) * position[pulley.axis] / radius, '${pulley.id} turns travel over its pitch radius', 1e-9);
			}
			near(state.joint("screwZ-turn"), Math.PI * position[2], "Z still turns its screw", 1e-9);
		}

		// Whole teeth: 20-tooth GT2 pulleys put the loop at the centre distance plus 20 teeth.
		var report:Array<String> = [];
		for (id in ["beltX", "beltYLeft", "beltYRight"]) {
			var loop = belt(router, id);
			near(loop.slack(), 0, '$id is a whole number of teeth long', 1e-6);
			near(loop.centreAdjustment(), 0, '$id needs no centre adjustment', 1e-6);
			report.push('$id ${loop.teeth} teeth, ${Math.round(loop.length * 100) / 100} mm');
		}
		near(belt(router, "beltX").length, 2 * 444 + 40, "X belt length", 1e-6);
		near(belt(router, "beltYLeft").length, 2 * 640 + 40, "Y belt length", 1e-6);

		// Each belt clears the frame, and its carriage's bracket holds exactly its lower strand: a band
		// 6 mm wide and 1.38 mm thick through the bracket's 40 mm.
		var clamp = 6 * 1.38 * 40;
		var xFrame = ["beamUpper", "beamLower", "uprightLeft", "uprightRight", "motorPlateX", "idlerPlateX", "xPlate", "blockXUpper",
			"blockXLower", "zPlate", "motorBracketZ", "motorZ", "screwZ", "spindle", "spindleClamp", "axleX"];
		for (at in [[0.0, 0, 0], [300.0, 300, -80], [150.0, 150, 0]]) {
			checkClear(router, state, at, ["beltX"], xFrame);
			near(overlap(router, state, at, "beltX", "beltBracketX"), clamp, "the X bracket clamps one strand", 0.5);
		}
		var yFrame = ["sideLeft", "sideRight", "railYLeft", "railYRight", "crossFront", "crossMiddle", "crossBack", "spoilboard",
			"blockYLeft", "blockYRight", "uprightLeft", "uprightRight", "motorPlateYLeft", "motorPlateYRight", "idlerPlateYLeft",
			"idlerPlateYRight", "motorYLeft", "motorYRight", "axleYLeft", "axleYRight"];
		for (at in [[150.0, 0, 0], [150.0, 300, 0], [150.0, 150, 0]]) {
			checkClear(router, state, at, ["beltYLeft", "beltYRight"], yFrame);
			near(overlap(router, state, at, "beltYLeft", "beltBracketYLeft"), clamp, "the left Y bracket clamps one strand", 0.5);
			near(overlap(router, state, at, "beltYRight", "beltBracketYRight"), clamp, "the right Y bracket clamps one strand", 0.5);
		}
		// Motors, plates and idlers clear the gantry and carriage at the ends of travel.
		var gantry = ["uprightLeft", "uprightRight", "beltBracketYLeft", "beltBracketYRight", "beamUpper", "beamLower", "blockYLeft",
			"blockYRight", "motorX", "motorPlateX", "idlerPlateX", "pulleyX", "idlerX"];
		var ends = ["motorPlateYLeft", "motorPlateYRight", "motorYLeft", "motorYRight", "pulleyYLeft", "pulleyYRight"];
		checkClear(router, state, [150, 300, 0], gantry, ends);
		checkClear(router, state, [150, 0, 0], gantry, ["idlerPlateYLeft", "idlerPlateYRight", "idlerYLeft", "idlerYRight", "axleYLeft", "axleYRight"]);
		checkClear(router, state, [300, 300, -80], ["motorX", "motorPlateX", "idlerPlateX", "pulleyX", "idlerX"],
			["crossBack", "sideLeft", "sideRight", "spoilboard"]);
		checkClear(router, state, [300, 0, 0], ["xPlate", "beltBracketX", "blockXUpper", "blockXLower"], ["pulleyX", "idlerX", "motorX", "motorPlateX", "idlerPlateX"]);
		checkClear(router, state, [0, 0, 0], ["xPlate", "beltBracketX", "blockXUpper", "blockXLower"], ["pulleyX", "idlerX", "motorX", "motorPlateX", "idlerPlateX"]);
		checkXHomeClearance(router);
		Sys.println('cnc router belts: ${report.join("; ")}');
	}

	static function beltDrive(router:CncRouter, pulley:String):machinekit.assembly.MachineAssemblyDescription.TransmissionRecord {
		var drive = router.transmissionFor(pulley + "-belt");
		if (drive == null) throw '$pulley should turn through a belt drive';
		if (!switch drive.source { case TimingBelt(_, id), BeltIdler(_, id): id == pulley; default: false; }) throw '$pulley should turn through a belt drive';
		return drive;
	}

	static function belt(router:CncRouter, id:String):TimingBelt {
		for (entry in router.components()) if (entry.id == id) return cast entry.component;
		throw 'belt router has no member "$id"';
	}

	/** Every rail block stays within its rail's usable length. */
	static function checkBlocksOnRails(router:CncRouter, state:AssemblyState):Void {
		for (pair in [["railYLeft", "blockYLeft"], ["railYRight", "blockYRight"], ["railXUpper", "blockXUpper"],
				["railXLower", "blockXLower"], ["railZLeft", "blockZLeft"], ["railZRight", "blockZRight"]]) {
			var local = AssemblyFrames.compose(AssemblyFrames.inverse(state.worldPose(pair[0])), state.worldPose(pair[1]));
			var rail = [for (entry in router.components()) if (entry.id == pair[0]) cast(entry.component, LinearRail)][0];
			var reach = rail.spec.railEndMargin + rail.spec.blockLength / 2;
			near(local.x, 0, '${pair[1]} centred on ${pair[0]}', 1e-6);
			near(local.y, 0, '${pair[1]} seated on ${pair[0]}', 1e-6);
			if (!(local.z >= reach - 1e-6 && local.z <= rail.length - reach + 1e-6))
				throw '${pair[1]} runs off ${pair[0]}: ${local.z} of ${rail.length} mm';
		}
	}

	/** No member of `moving` intersects a member of `others` at machine position `at`. */
	static function checkXHomeClearance(router:CncRouter):Void {
		var definition = router.definition();
		for (joint in definition.joints) if (joint.limits.overtravel != null) {
			if (joint.limits.lower != null) joint.limits.lower -= joint.limits.overtravel;
			if (joint.limits.upper != null) joint.limits.upper += joint.limits.overtravel;
		}
		var state = new AssemblyState(definition);
		// Include the interior pose that blocked physical homing, and both guide ends.
		for (x in [-router.axisOvertravel("x"), 0.0, 36.4, 150.0, 300.0, 300.0 + router.axisOvertravel("x")])
			for (z in [-80.0 - router.axisOvertravel("z"), -80.0, 0.0, router.axisOvertravel("z")]) {
				checkClear(router, state, [x, 150.0, z],
					["xPlate", "zPlate", "blockZLeft", "blockZRight", "spindleClamp", "spindle", "tool", "homeTriggerZ"],
					["homeX", "homeXMount"]);
				checkClear(router, state, [x, 150.0, z], ["homeTriggerX"], ["beamUpper", "railXUpper"]);
			}
	}

	public static function checkClear(router:CncRouter, state:AssemblyState, at:Array<Float>, moving:Array<String>,
			others:Array<String>):Void {
		moveTo(state, at);
		var first:Array<Part> = [], second:Array<Part> = [];
		try {
			for (id in moving) first.push(posed(router, state, id));
			for (id in others) second.push(posed(router, state, id));
			var firstBoxes = [for (part in first) PosedParts.boxOf(part)], secondBoxes = [for (part in second) PosedParts.boxOf(part)];
			for (a in 0...first.length) for (b in 0...second.length) {
				var volume = PosedParts.commonVolume(first[a], firstBoxes[a], second[b], secondBoxes[b]);
				if (volume > 1e-3) throw '${moving[a]} collides with ${others[b]} at machine ${at.join(", ")}: ${Math.round(volume)} mm³';
			}
		} catch (error:Dynamic) {
			PosedParts.closeAll(first);
			PosedParts.closeAll(second);
			throw error;
		}
		PosedParts.closeAll(first);
		PosedParts.closeAll(second);
	}

	static function moveTo(state:AssemblyState, at:Array<Float>):Void {
		state.setJoint("x", at[0]);
		state.setJoint("y", at[1]);
		state.setJoint("z", at[2]);
		state.forwardKinematics();
	}

	static function overlap(router:CncRouter, state:AssemblyState, at:Array<Float>, a:String, b:String):Float {
		moveTo(state, at);
		var first = posed(router, state, a), second = posed(router, state, b);
		var volume = PosedParts.commonVolume(first, PosedParts.boxOf(first), second, PosedParts.boxOf(second));
		first.close();
		second.close();
		return volume;
	}

	static function posed(router:CncRouter, state:AssemblyState, id:String):Part {
		for (entry in router.components()) if (entry.id == id) return parts.posed(entry.component, state.worldPose(id));
		throw 'CNC router has no member "$id"';
	}
}

/** Standalone check of the router example. */
function main():Void CncRouterChecks.run();
