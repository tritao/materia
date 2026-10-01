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
	public static function router():Bytes {
		var router = new CncRouter();
		var scene = AssemblyPreview.scene(router, ASSEMBLY_ID);
		var plate = motorPlate();
		var part = plate.geometry(ComponentDetail.Preview);
		var program = MountPlateJob.program(router, plate, part);
		scene.parts.push(AssemblyPreview.part(TARGET_PART, "Finished motor plate", part, plate.materialId, TARGET_DEFLECTION));
		scene.machining = {program: program, axes: ["x", "y", "z"], spindle: "spindle",
			workOffset: [for (value in CncRouter.workOffset()) value / 1000],
			tools: [for (tool in router.tools()) {number: tool.number, length: tool.length, profile: tool.profile().encode()}],
			stock: "stock", sacrificial: ["spoilboard"], toolPart: "tool", loadedTool: 1,
			target: TARGET_PART, loop: true};
		return SceneArtifact.encode(scene);
	}

	/** The part the router mills from its stock block. */
	public static function motorPlate():NemaMountPlate
		return new NemaMountPlate(NemaStepper.frame(23), CncRouter.STOCK_WIDTH, CncRouter.STOCK_DEPTH, CncRouter.STOCK_HEIGHT, 6);
}

/** Geometry builds, the axes move the tool where machine coordinates say, and nothing collides. */
class CncRouterChecks {
	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void {
		if (!(Math.abs(actual - expected) <= tolerance))
			throw '$message: expected $expected, got $actual';
	}

	public static function run():Void {
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
	static function checkClear(router:CncRouter, state:AssemblyState, at:Array<Float>, moving:Array<String>,
			others:Array<String>):Void {
		for (a in moving) for (b in others) {
			var volume = overlap(router, state, at, a, b);
			if (volume > 1e-3) throw '$a collides with $b at machine ${at.join(", ")}: ${Math.round(volume)} mm³';
		}
	}

	static function overlap(router:CncRouter, state:AssemblyState, at:Array<Float>, a:String, b:String):Float {
		state.setJoint("x", at[0]);
		state.setJoint("y", at[1]);
		state.setJoint("z", at[2]);
		state.forwardKinematics();
		var first = posed(router, state, a), second = posed(router, state, b);
		var common = first.intersect(second);
		var volume = common.volume();
		common.close();
		first.close();
		second.close();
		return volume;
	}

	static function posed(router:CncRouter, state:AssemblyState, id:String):Part {
		for (entry in router.components()) if (entry.id == id) {
			var pose = state.worldPose(id);
			var x = AssemblyFrames.transformVector(pose, 1, 0, 0), z = AssemblyFrames.transformVector(pose, 0, 0, 1);
			var local = entry.component.geometry(ComponentDetail.Preview);
			var placed = local.placed(new Location(new Plane(new Vector(pose.x, pose.y, pose.z), new Vector(x.x, x.y, x.z),
				new Vector(z.x, z.y, z.z))));
			local.close();
			return placed;
		}
		throw 'CNC router has no member "$id"';
	}
}

/** Standalone check of the router example. */
function main():Void CncRouterChecks.run();
