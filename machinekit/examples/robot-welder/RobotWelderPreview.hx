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
import machinekit.robotics.EndEffectorControls;
import machinekit.welding.WeldSeam;
import machinekit.welding.WeldSeams;
import machinekit.welding.WeldingEquipment;
import machinekit.welding.WeldingPowerSource;
import machinekit.welding.WeldingRecipe;
import machinekit.welding.WeldingTorch;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;

/** Materia project entrypoint for the robot welding cell. */
class RobotWelderPreview {
	public static inline var ASSEMBLY_ID:String = "robot-welder";

	/**
	 * Geometry, joints and initial pose of the cell: the arm with its torch, the feeder, the power
	 * source and gas cylinder, and the table with its weldment. The scene's `robotTools` has the torch as a
	 * `torch` tool: its wire tip, its channels, and the welder behind it (the supply's limits, the wire, and
	 * the work the weld circuit returns through, which the work clamp's connections and mates give).
	 */
	public static function cell():Bytes {
		var cell = new WeldingCell();
		var scene = AssemblyPreview.scene(cell, ASSEMBLY_ID);
		scene.robotTools = AssemblyPreview.robotTools(cell.arm.tool, "arm/tool", cell.equipment());
		scene.mission = {steps: [weldStep(cell, scene.metresPerUnit)]};
		return SceneArtifact.encode(scene);
	}

	/** The seam the cell's mission welds: the plate's T-joint with the upright, on the upright's +Y side. */
	public static inline var WELD_SEAM:String = "work/basePlate:box.+z|work/upright:box.+y";

	/** The wire the cell's feeder declares, which the recipes are made for. */
	static function wireOf(cell:WeldingCell):machinekit.welding.WeldingRecipe.RecipeWire
		return {diameterMm: cell.feeder.wireDiameterMm, depositionEfficiency: cell.feeder.depositionEfficiency,
			maxSpeedMPerMin: cell.feeder.maxSpeedMPerMin};

	/** The cell's seams, found from the members' geometry, in the workpiece's frame (its reference member's). */
	static function seamsOf(cell:WeldingCell):Array<WeldSeam> {
		var model = new AssemblyModel("mm");
		cell.addTo(model, "");
		var state = new AssemblyState(model.definition(ASSEMBLY_ID));
		state.forwardKinematics();
		return cell.weldment().findIn(cell, cell.solvedPoses(state)).require();
	}

	/**
	 * The mission's one step: weld `WELD_SEAM`. The seam is found from the members' geometry and given relative to the
	 * workpiece's reference member, so a player finds it where the workpiece stands; it is welded with the recipe for the
	 * leg the weldment asks for, in the wire the cell's feeder declares.
	 */
	public static function weldStep(cell:WeldingCell, metresPerUnit:Float):materia.project.SceneArtifact.SceneArtifactMissionStep {
		var found = [for (item in seamsOf(cell)) if (item.name() == WELD_SEAM) item];
		if (found.length != 1) throw 'The cell has no seam "$WELD_SEAM"';
		return WeldingRecipe.fillet(found[0].legSize, wireOf(cell)).step([found[0]], cell.weldment().reference, "work/weldMetal", metresPerUnit);
	}

	/**
	 * A weld of one tube post's perimeter: its four sides, chained into one closed loop (`WeldSeams.chains`) and welded as
	 * one step. The sides lie between different pairs of faces, so the torch's angles change at every corner; the
	 * path turns it there. `post` counts the closed chains the weldment has, in the order they are found.
	 */
	public static function postStep(cell:WeldingCell, metresPerUnit:Float, post:Int = 0):materia.project.SceneArtifact.SceneArtifactMissionStep {
		var loops = [for (chain in WeldSeams.chains(seamsOf(cell))) if (chain.closed) chain];
		if (post >= loops.length) throw 'The cell has ${loops.length} posts to weld, not ${post + 1}';
		var seams = loops[post].seams;
		return WeldingRecipe.fillet(seams[0].legSize, wireOf(cell)).step(seams, cell.weldment().reference, "work/weldMetal", metresPerUnit);
	}

	/** The cell with a mission that welds the first post's perimeter instead of the plate's seam. */
	public static function postCell():Bytes {
		var cell = new WeldingCell();
		var scene = AssemblyPreview.scene(cell, ASSEMBLY_ID);
		scene.robotTools = AssemblyPreview.robotTools(cell.arm.tool, "arm/tool", cell.equipment());
		scene.mission = {steps: [postStep(cell, scene.metresPerUnit)]};
		return SceneArtifact.encode(scene);
	}
}

/** A pose of the arm: its six joint values, in radians. */
typedef ArmPose = Array<Float>;

/** An arm pose that reaches a place on a seam, and where that is. */
typedef SeamPose = {
	var label:String;
	var angles:ArmPose;
}

/**
 * Geometry builds, the services reach the torch, the torch points down at the table, its neck
 * clears the arm, the weldment's seams are found from its geometry, and the arm reaches them with
 * the torch in the seam frames.
 */
class RobotWelderChecks {
	static final JOINTS = ["arm/j1", "arm/j2", "arm/j3", "arm/j4", "arm/j5", "arm/j6"];

	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void {
		if (!(Math.abs(actual - expected) <= tolerance))
			throw '$message: expected $expected, got $actual';
	}

	public static function run():Void {
		parts = new PosedParts();
		try runChecks() catch (error:Dynamic) {
			parts.close();
			throw error;
		}
		parts.close();
	}

	/** The geometry of the members posed by this run, built once each; closed when the run ends. */
	static var parts:PosedParts;

	static function runChecks():Void {
		var scene = SceneArtifact.decode(RobotWelderPreview.cell());
		var definition = scene.assemblyDefinition;
		var cell = new WeldingCell();
		if (definition == null || definition.occurrences.length != cell.components().length)
			throw "Robot welder preview has the wrong number of occurrences";
		for (part in scene.parts) if (part.volume == null || part.volume <= 0 || part.inertia == null)
			throw 'Robot welder part "${part.id}" has no mass properties';

		checkServices(cell);
		checkTorchTool(cell, scene);
		checkMission(cell, scene);
		checkPostMission(cell);
		checkRecipeWire();

		var model = new AssemblyModel("mm");
		cell.addTo(model, "");
		var state = new AssemblyState(model.definition(RobotWelderPreview.ASSEMBLY_ID));
		var ready = [for (spec in cell.arm.specs) spec.initial];
		pose(state, ready);
		checkReadyPose(cell, state);
		var seams = WeldSeamChecks.run(cell, state);
		var poses = checkReach(cell, state, ready, seams);
		reportChainReach(cell, state, ready, seams);
		checkClearance(cell, state, ready, poses);

		var bom = cell.billOfMaterials().lines();
		var parts = [for (line in bom) line.partNumber];
		for (prefix in ["WELD-TORCH-MIG-", "WIRE-FEEDER-", "WELD-SOURCE-", "GAS-CYLINDER-", "HOSE-GAS-", "CABLE-WELD-", "CABLE-WORK-", "WELD-WORK-CLAMP-", "HOSEPACK-"])
			if ([for (number in parts) if (StringTools.startsWith(number, prefix)) number].length == 0)
				throw 'The bill of materials should list $prefix equipment, got $parts';
		var mass = cell.massProperties().mass;
		Sys.println('robot welder: ${scene.parts.length} definitions, ${definition.occurrences.length} occurrences, ' +
			'${bom.length} BOM lines, ${Math.round(mass * 10) / 10} kg');
	}

	/** The torch's inlets are all supplied from the power source, the cylinder or the controller. */
	static function checkServices(cell:WeldingCell):Void {
		var arm = cell.arm;
		// Both the arm alone and the cell supply every required inlet, or throw.
		arm.validate();
		cell.validate();
		function chain(instance:String, port:String):String return cell.upstreamChain(instance, port).join(" < ");
		var power = chain("arm/tool/torch", "power");
		if (power.indexOf("source/weldPositive") < 0) throw 'Torch power should come from the power source, got $power';
		var gas = chain("arm/tool/torch", "gas");
		if (gas.indexOf("cylinder/gas") < 0) throw 'Torch gas should come from the cylinder, got $gas';
		if (!cell.upstream("arm/tool/torch", "control").external) throw "Torch control should come from the controller";
		if (!cell.upstream("source", "mains").external) throw "The power source should run from the wall";
		if (cell.port("mains").instanceId != "source") throw "The cell should expose the power source's mains";
		var supplies = [for (entry in cell.components()) if (Std.isOfType(entry.component, WeldingPowerSource)) entry.id];
		if (supplies.join(",") != "source") throw 'Expected one power source, got $supplies';

		// The effector derives its arc channel from the torch, to drive the arc through the feeder and source.
		var controls = EndEffectorControls.derive(arm.tool, "tool");
		if (controls.arcChannel() != "tool/torch.arc" || controls.arcs[0].tcpConnector != "tcp")
			throw 'The torch should derive an arc channel on tcp, got ${controls.arcs}';
		var arc = controls.arcs[0];
		if (arc.wireSpeedChannel != "tool/torch.wire_speed" || arc.voltageChannel != "tool/torch.voltage" || arc.sensor != "tool/torch.weld")
			throw 'The torch should derive wire speed, voltage and weld sensor names, got $arc';
		var digital:Array<String> = [];
		for (control in controls.controls) switch control {
			case Arc(channel, member, port): digital.push('$channel>$member/$port');
			case _:
		}
		if (digital.join(",") != "tool/torch.arc>torch/control") throw 'The arc should be a control on the torch inlet, got $digital';
		if (controls.vacuumChannel() != null) throw "The welding tool has no vacuum control";
	}

	/**
	 * The weld circuit returns through the work clamp, and the simulated welder is told so: the work lead is
	 * traced to the clamp, the clamp's mate says where it sits, and everything welded to that is grounded work.
	 */
	static function checkTorchTool(cell:WeldingCell, scene:SceneArtifactData):Void {
		if (cell.upstreamChain("clamp", "lead").join(" < ") != "clamp/lead < source/weldNegative")
			throw "The work clamp should be fed by the power source's work lead";
		var tools = scene.robotTools;
		if (tools == null || tools.length != 1 || tools[0].kind != "torch") throw "The cell should carry one torch tool";
		var tool = tools[0];
		if (tool.contact.occurrence != "arm/tool/torch" || tool.contact.connector != "tcp")
			throw 'The torch tool should work at the wire tip, got ${tool.contact}';
		if (tool.channel != "arm/tool/torch.arc" || tool.sensor != "arm/tool/torch.weld")
			throw 'The torch tool channels are named after the member, got ${tool.channel} and ${tool.sensor}';
		var welder = tool.torch;
		if (welder == null) throw "The torch tool should describe its welder";
		if (welder.wireSpeedChannel != "arm/tool/torch.wire_speed" || welder.voltageChannel != "arm/tool/torch.voltage")
			throw "The torch tool should name its analogue channels";
		near(welder.maxCurrentA, cell.source.maxCurrentA, "supply rating", 0);
		near(welder.efficiency, cell.source.efficiency, "supply efficiency", 0);
		near(welder.wireDiameterMm, cell.feeder.wireDiameterMm, "wire diameter", 0);
		near(welder.stickoutMm, WeldingTorch.STICKOUT, "stickout", 0);
		// The grounded work is the weldment, not the table or the fixtures it lies in.
		var expected = cell.weldment().members.copy();
		expected.sort(Reflect.compare);
		var grounded = welder.groundedWork.copy();
		grounded.sort(Reflect.compare);
		if (grounded.join(",") != expected.join(","))
			throw 'The grounded work should be the weldment ${expected.join(",")}, got ${grounded.join(",")}';
		for (other in ["table", "fixtureNear", "fixtureFar", "arm/tool/torch"])
			if (welder.groundedWork.indexOf(other) >= 0) throw '$other should not be grounded work';
		// An unwelded workpiece is grounded only where the clamp sits, and a cell with no clamp has no circuit.
		var bare = WeldingEquipment.of(cell, []);
		if (bare.groundedWork.join(",") != "work/basePlate") throw 'Without welds only the clamped plate is grounded, got ${bare.groundedWork}';
		var open = new machinekit.assembly.MachineAssembly();
		open.addComponent("source", new WeldingPowerSource());
		open.addComponent("feeder", new machinekit.welding.WireFeeder());
		var failed = false;
		try WeldingEquipment.of(open, []) catch (_:Dynamic) failed = true;
		if (!failed) throw "A power source with no work clamp should be refused";
		Sys.println('robot welder: torch tool grounded on ${grounded.join(", ")}');
	}

	/**
	 * The mission welds one seam of the T-joint, found from the geometry, with the recipe for its leg: the travel speed
	 * that deposits an equal-leg fillet's cross-section from the wire speed, the torch at the seam frame's angles.
	 */
	static function checkMission(cell:WeldingCell, scene:SceneArtifactData):Void {
		var mission = scene.mission;
		if (mission == null || mission.steps.length != 1 || mission.steps[0].kind != "weld" || mission.steps[0].weld == null)
			throw "The cell should carry a mission that welds one seam";
		var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast mission.steps[0].weld;
		if (weld.frame != "work/basePlate") throw 'The weld should be placed by the workpiece\'s reference member, got ${weld.frame}';
		if (weld.path.length != 1 || weld.path[0].seam != RobotWelderPreview.WELD_SEAM || weld.path[0].joint != "fillet")
			throw 'The mission should weld the plate/upright fillet, got ${[for (segment in weld.path) segment.seam]}';
		var segment = weld.path[0];
		near(weld.legSize, WeldingWorkpiece.LEG_SIZE * 0.001, "the seam's leg", 1e-12);
		var length = Math.sqrt(Math.pow(segment.stop.position[0] - segment.start.position[0], 2) + Math.pow(segment.stop.position[1] - segment.start.position[1], 2) +
			Math.pow(segment.stop.position[2] - segment.start.position[2], 2));
		near(length, WeldingWorkpiece.PLATE_LENGTH * 0.001, "the seam's length", 1e-6);
		var process = weld.process;
		// Wire speed times wire area times efficiency over travel speed is the section of an equal-leg fillet of that leg.
		var cellFeeder = new WeldingCell().feeder;
		var area = process.wireSpeed * 1000 / 60 * Math.PI * cellFeeder.wireDiameterMm * cellFeeder.wireDiameterMm / 4 * cellFeeder.depositionEfficiency / (process.travelSpeed * 1000);
		near(Math.sqrt(2 * area), WeldingWorkpiece.LEG_SIZE, "the leg the recipe deposits", 1e-9);
		if (!(process.wireSpeed > 5 && process.wireSpeed < 12 && process.voltage > 20 && process.voltage < 30))
			throw 'The recipe for a ${WeldingWorkpiece.LEG_SIZE} mm fillet should run near 8 m/min and 24 V, got ${process.wireSpeed} m/min and ${process.voltage} V';
		// The wire is on the faces' bisector, tilted back from the plate by the push angle.
		var q = segment.start.rotation;
		var wire = AssemblyFrames.transformVector({x: 0.0, y: 0.0, z: 0.0, qx: q[0], qy: q[1], qz: q[2], qw: q[3]}, 0, 0, 1);
		if (!(wire.z < -0.5)) throw 'The seam\'s wire should point down at the plate, got ${wire.z}';
		Sys.println('robot welder: mission welds ${segment.seam} (${Math.round(length * 1000)} mm) at ${Math.round(process.wireSpeed * 10) / 10} m/min, ' +
			'${Math.round(process.voltage * 10) / 10} V, ${Math.round(process.travelSpeed * 10000) / 10} mm/s');
	}

	/**
	 * A tube post's four sides weld as one step: a path of four connected segments, one per side, each with its own faces
	 * (the torch's angles change at the corners), that closes on itself, and the whole scene still validates with it.
	 */
	static function checkPostMission(cell:WeldingCell):Void {
		var metres = 0.001;
		var step = RobotWelderPreview.postStep(cell, metres);
		var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast step.weld;
		if (weld.path.length != 4) throw 'A post\'s perimeter should be four segments, got ${weld.path.length}';
		var names = new Map<String, Bool>();
		for (index in 0...4) {
			var segment = weld.path[index], next = weld.path[(index + 1) % 4];
			names.set(segment.seam, true);
			for (axis in 0...3) near(segment.stop.position[axis], next.start.position[axis], "a side ends where the next begins", 1e-9);
			if (segment.normals[0].join(",") == next.normals[0].join(",") && segment.normals[1].join(",") == next.normals[1].join(","))
				throw "Adjacent sides of a post should lie between different faces";
		}
		var count = 0;
		for (_ in names.keys()) count++;
		if (count != 4) throw "The four sides of a post should be four seams";
		var scene = SceneArtifact.decode(RobotWelderPreview.postCell());
		if (scene.mission == null || scene.mission.steps.length != 1) throw "The post cell should carry a mission of one weld";
		Sys.println('robot welder: post mission welds ${weld.path.length} sides as one step');
	}

	/**
	 * The recipe is made for the wire the feeder declares: a thinner wire deposits less per metre and so travels slower for
	 * the same leg, a wire that deposits less does the same, and a leg that needs more wire than the feeder can feed is refused.
	 */
	static function checkRecipeWire():Void {
		var standard = WeldingRecipe.fillet(5, {diameterMm: 1.2, depositionEfficiency: 0.95, maxSpeedMPerMin: 20});
		var thin = WeldingRecipe.fillet(5, {diameterMm: 1.0, depositionEfficiency: 0.95, maxSpeedMPerMin: 20});
		near(thin.wireSpeed, standard.wireSpeed, "the wire speed depends on the leg", 1e-12);
		near(thin.travelSpeed / standard.travelSpeed, 1.0 / 1.44, "a 1.0 mm wire travels at (1.0 / 1.2)^2 of the 1.2 mm wire's speed", 1e-9);
		var lossy = WeldingRecipe.fillet(5, {diameterMm: 1.2, depositionEfficiency: 0.8, maxSpeedMPerMin: 20});
		near(lossy.travelSpeed / standard.travelSpeed, 0.8 / 0.95, "a wire that deposits less travels slower", 1e-9);
		var refused = false;
		try WeldingRecipe.fillet(5, {diameterMm: 1.2, depositionEfficiency: 0.95, maxSpeedMPerMin: 6}) catch (_:Dynamic) refused = true;
		if (!refused) throw "A 5 mm fillet needs 8 m/min of wire, which a 6 m/min feeder cannot give: the recipe should be refused";
		var feeder = new WeldingCell().feeder;
		near(feeder.depositionEfficiency, machinekit.welding.WireFeeder.SOLID_WIRE_EFFICIENCY, "the feeder's wire efficiency", 0);
	}

	static function checkReadyPose(cell:WeldingCell, state:AssemblyState):Void {
		var tcp = state.worldConnector("arm/tool/torch", "tcp");
		var wire = AssemblyFrames.transformVector(tcp, 0, 0, 1);
		var flange = state.worldPose("arm/toolFlange");
		// The wire leaves the torch along the neck's bend, 45 degrees off the flange axis, which points down.
		var flangeAxis = AssemblyFrames.transformVector(flange, 0, 0, 1);
		near(flangeAxis.z, -1, "tool axis points down in the ready pose", 1e-6);
		near(wire.x * flangeAxis.x + wire.y * flangeAxis.y + wire.z * flangeAxis.z, Math.cos(Math.PI / 4),
			"the wire leaves the torch 45 degrees off the flange axis", 1e-9);
		if (!(wire.z < -0.7)) throw 'The torch should point down toward the table, its wire is ${wire.x}, ${wire.y}, ${wire.z}';
		// The tip hangs a torch length below the flange, over the table.
		var reach = WeldingTorch.BEND_Z + WeldingTorch.NECK_LENGTH + WeldingTorch.NOZZLE_LENGTH;
		if (!(flange.z - tcp.z > 150 && flange.z - tcp.z < reach))
			throw 'The wire tip should hang below the flange, got ${Math.round(flange.z - tcp.z)} mm';
		if (!(tcp.z > WeldingCell.TABLE_TOP - 1)) throw "The wire tip should clear the table at the ready pose";
		Sys.println('robot welder: wire tip at ${Math.round(tcp.x)}, ${Math.round(tcp.y)}, ${Math.round(tcp.z)} mm, ' +
			'wire ${round2(wire.x)}, ${round2(wire.y)}, ${round2(wire.z)} in the ready pose');
	}

	/**
	 * Every seam is within reach, with the torch in the seam's frame: the wire on the bisector of the two faces,
	 * tilted by the push angle. A long seam is tried at its start, middle and end, a short one (a tube side) at its
	 * middle. Returns the poses found.
	 */
	static function checkReach(cell:WeldingCell, state:AssemblyState, ready:ArmPose, seams:Array<WeldSeam>):Array<SeamPose> {
		var poses:Array<SeamPose> = [];
		var limits = [for (spec in cell.arm.specs) {lower: spec.lower, upper: spec.upper}];
		// Seams are in the workpiece's frame, so the cell places them with the workpiece's pose.
		var workpiece = state.worldPose("work/basePlate");
		for (seam in seams) {
			for (fraction in seam.length() > 100 ? [0.0, 0.5, 1.0] : [0.5]) {
				var frame = seam.frameAtParameter(fraction).transformed(workpiece);
				var target = {x: frame.position.x, y: frame.position.y, z: frame.position.z};
				var wire = frame.wire();
				var found:Null<Array<Float>> = null;
				for (seed in seeds(ready)) if (found == null)
					found = ArmIk.solve(state, JOINTS, limits, target, {x: wire.x, y: wire.y, z: wire.z}, seed);
				if (found == null) throw 'The seam ${seam.name()} at ${Math.round(target.x)}, ${Math.round(target.y)}, ${Math.round(target.z)} is out of reach of the torch';
				poses.push({label: '${seam.name()} at $fraction', angles: found});
			}
		}
		pose(state, ready);
		return poses;
	}

	static function reportChainReach(cell:WeldingCell, state:AssemblyState, ready:ArmPose, seams:Array<WeldSeam>):Void {
		var limits = [for (spec in cell.arm.specs) {lower: spec.lower, upper: spec.upper}];
		var workpiece = state.worldPose("work/basePlate");
		var chainNumber = 0;
		for (chain in WeldSeams.chains(seams)) if (chain.closed) {
			for (index in 0...chain.seams.length) for (fraction in [0.0, 0.5, 1.0]) for (out in [0.0, 40.0]) {
				var frame = chain.seams[index].frameAtParameter(fraction).transformed(workpiece);
				var wire = frame.wire();
				var target = {x: frame.position.x - wire.x * out, y: frame.position.y - wire.y * out, z: frame.position.z - wire.z * out};
				var found:Null<Array<Float>> = null;
				for (seed in seeds(ready)) if (found == null)
					found = ArmIk.solve(state, JOINTS, limits, target, {x: wire.x, y: wire.y, z: wire.z}, seed);
				Sys.println('chain $chainNumber side $index at $fraction, ${out} mm out: ${found == null ? "UNREACHABLE" : "reached"} (${Math.round(target.x)}, ${Math.round(target.y)}, ${Math.round(target.z)}; wire ${round2(wire.x)}, ${round2(wire.y)}, ${round2(wire.z)})');
			}
			chainNumber++;
		}
	}

	/**
	 * At the ready pose and at each pose that reaches a seam, the torch (neck, nozzle and breakaway
	 * mount) clears every other part of the arm and the feeder, and the table, equipment and weldment;
	 * and the arm and feeder clear the table, equipment and weldment too.
	 */
	static function checkClearance(cell:WeldingCell, state:AssemblyState, ready:ArmPose, seams:Array<SeamPose>):Void {
		var moving = [for (entry in cell.components()) if (StringTools.startsWith(entry.id, "arm/")) entry.id];
		moving.push("feeder");
		var fixed = ["table", "source", "cylinder", "clamp", "fixtureNear", "fixtureFar"].concat(cell.weldment().members);
		for (entry in [{label: "the ready pose", angles: ready}].concat(seams)) {
			pose(state, entry.angles);
			try {
				for (id in moving) {
					// The tool's plate meets the flange it is bolted to, and the arm's own links meet at their joints.
					if (id != "arm/tool/torch") for (other in fixed) checkApart(cell, state, id, other);
					if (id != "arm/tool/torch" && !StringTools.startsWith(id, "arm/tool/")) checkApart(cell, state, "arm/tool/torch", id);
				}
				for (other in fixed) checkApart(cell, state, "arm/tool/torch", other);
			} catch (error:Dynamic) {
				throw 'At ${entry.label}: $error';
			}
		}
		pose(state, ready);
	}

	static function checkApart(cell:WeldingCell, state:AssemblyState, a:String, b:String):Void {
		var first = posed(cell, state, a), second = posed(cell, state, b);
		var volume = PosedParts.commonVolume(first, PosedParts.boxOf(first), second, PosedParts.boxOf(second));
		first.close();
		second.close();
		if (volume > 1e-3) throw '$a hits $b: ${Math.round(volume)} mm³';
	}

	/** Starting poses for the solver: the ready pose, and the arm leaning further out and in, tool kept down. */
	static function seeds(ready:ArmPose):Array<ArmPose> {
		var result = [ready];
		for (shoulder in [0.4, 0.8, 1.2]) for (elbow in [-1.4, -0.9, -0.4]) for (turn in [0.0, 0.5, -0.5])
			result.push([turn, shoulder, elbow, 0, Math.PI - (shoulder - elbow), 0]);
		return result;
	}

	static function pose(state:AssemblyState, angles:ArmPose):Void {
		for (i in 0...JOINTS.length) state.setJoint(JOINTS[i], angles[i]);
		state.forwardKinematics();
	}

	static function round2(value:Float):Float return Math.round(value * 100) / 100;

	static function posed(cell:WeldingCell, state:AssemblyState, id:String):Part {
		for (entry in cell.components()) if (entry.id == id) return parts.posed(entry.component, state.worldPose(id));
		throw 'Robot welder has no member "$id"';
	}
}

/**
 * A damped least-squares inverse kinematics solver for the cell's arm, by finite differences on its
 * forward kinematics: it brings the wire tip to a point with the wire along a direction, from a
 * seed pose, within the joint limits. Enough to ask whether a seam can be reached; a runtime planner
 * does this properly.
 */
class ArmIk {
	/** Weight of one unit of direction error against one millimetre of position error. */
	static inline var DIRECTION_WEIGHT:Float = 200;

	/** The joint values that put the tip on `target` with the wire along `direction`, or null. */
	public static function solve(state:AssemblyState, joints:Array<String>, limits:Array<{lower:Float, upper:Float}>,
			target:{x:Float, y:Float, z:Float}, direction:{x:Float, y:Float, z:Float}, seed:Array<Float>):Null<Array<Float>> {
		var q = [for (j in 0...joints.length) Math.min(limits[j].upper, Math.max(limits[j].lower, seed[j]))];
		var residual = evaluate(state, joints, q, target, direction);
		var damping = 1.0;
		for (iteration in 0...120) {
			if (norm(residual) < 0.05) return q;
			var jacobian:Array<Array<Float>> = [];
			for (j in 0...joints.length) {
				var step = q.copy();
				var h = q[j] + 1e-4 <= limits[j].upper ? 1e-4 : -1e-4;
				step[j] += h;
				var moved = evaluate(state, joints, step, target, direction);
				jacobian.push([for (k in 0...6) (moved[k] - residual[k]) / h]);
			}
			// (J^T J + damping I) delta = -J^T r, with jacobian[j][k] = d residual k / d joint j.
			var normal:Array<Array<Float>> = [];
			for (a in 0...joints.length) {
				var row:Array<Float> = [];
				for (b in 0...joints.length) {
					var sum = a == b ? damping : 0.0;
					for (k in 0...6) sum += jacobian[a][k] * jacobian[b][k];
					row.push(sum);
				}
				var rhs = 0.0;
				for (k in 0...6) rhs -= jacobian[a][k] * residual[k];
				row.push(rhs);
				normal.push(row);
			}
			var delta = gauss(normal);
			var next = [for (j in 0...joints.length) Math.min(limits[j].upper, Math.max(limits[j].lower, q[j] + delta[j]))];
			var nextResidual = evaluate(state, joints, next, target, direction);
			if (norm(nextResidual) < norm(residual)) {
				q = next;
				residual = nextResidual;
				damping = Math.max(1e-6, damping * 0.5);
			} else damping *= 4;
		}
		return norm(residual) < 0.5 ? q : null;
	}

	static function evaluate(state:AssemblyState, joints:Array<String>, q:Array<Float>, target:{x:Float, y:Float, z:Float},
			direction:{x:Float, y:Float, z:Float}):Array<Float> {
		for (i in 0...joints.length) state.setJoint(joints[i], q[i]);
		state.forwardKinematics();
		var tcp = state.worldConnector("arm/tool/torch", "tcp");
		var wire = AssemblyFrames.transformVector(tcp, 0, 0, 1);
		return [tcp.x - target.x, tcp.y - target.y, tcp.z - target.z, DIRECTION_WEIGHT * (wire.x - direction.x),
			DIRECTION_WEIGHT * (wire.y - direction.y), DIRECTION_WEIGHT * (wire.z - direction.z)];
	}

	static function norm(values:Array<Float>):Float {
		var sum = 0.0;
		for (value in values) sum += value * value;
		return Math.sqrt(sum);
	}

	/** Solves an augmented n x (n+1) system by elimination with partial pivoting. */
	static function gauss(matrix:Array<Array<Float>>):Array<Float> {
		var n = matrix.length;
		for (column in 0...n) {
			var pivot = column;
			for (row in column + 1...n) if (Math.abs(matrix[row][column]) > Math.abs(matrix[pivot][column])) pivot = row;
			var swap = matrix[column];
			matrix[column] = matrix[pivot];
			matrix[pivot] = swap;
			for (row in column + 1...n) {
				var factor = matrix[row][column] / matrix[column][column];
				for (k in column...n + 1) matrix[row][k] -= factor * matrix[column][k];
			}
		}
		var result = [for (i in 0...n) 0.0];
		var i = n - 1;
		while (i >= 0) {
			var sum = matrix[i][n];
			for (k in i + 1...n) sum -= matrix[i][k] * result[k];
			result[i] = sum / matrix[i][i];
			i--;
		}
		return result;
	}
}

/** Standalone check of the robot welder example. */
function main():Void RobotWelderChecks.run();
