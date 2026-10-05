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
import machinekit.robot.EndEffectorControls;
import machinekit.welding.WeldMetal;
import machinekit.welding.WeldSeam;
import machinekit.welding.WeldSeams;
import machinekit.welding.WeldingEquipment;
import machinekit.welding.WeldingMission;
import machinekit.welding.WeldingMission.TorchPlace;
import machinekit.welding.WeldingMission.WeldAccess;
import machinekit.welding.WeldingPowerSource;
import machinekit.welding.WeldingRecipe;
import machinekit.welding.WeldingTorch;
import machinekit.welding.Weldment;
import machinekit.welding.Weldment.WeldmentSeams;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import machinekit.robot.RobotScene;

/** Materia project entrypoint for the robot welding cell. */
class RobotWelderPreview {
	public static inline var ASSEMBLY_ID:String = "robot-welder";
	/** Where the arm's end effector is in the cell. */
	public static inline var TOOL_PREFIX:String = "arm/tool";

	/**
	 * Geometry, joints and initial pose of the cell: the arm with its torch, the feeder, the power
	 * source and gas cylinder, and the table with its weldment. The scene's `robotTools` has the torch as a
	 * `torch` tool: its wire tip, its channels, and the welder behind it (the supply's limits, the wire, and
	 * the work the weld circuit returns through, which the work clamp's connections and mates give). The mission welds
	 * the whole weldment: `WeldingMission` generates a `weld` step for every run of seams the weldment's declared joints
	 * have, in an order that spares the torch air travel and turning.
	 */
	public static function cell():Bytes return encode(new WeldingCell(), (cell, scene) -> mission(cell, scene.metresPerUnit).require());

	/** The seam the single-seam cell welds by default: the plate's T-joint with the upright, on the upright's +Y side. */
	public static inline var WELD_SEAM:String = "work/basePlate:box.+z|work/upright:box.+y";

	/** The cell with a mission of one weld, of the seam `WELD_SEAM`: for tests that look at one bead. */
	public static function seamCell():Bytes return encode(new WeldingCell(), (cell, scene) -> [weldStep(cell, scene.metresPerUnit)]);

	/** Process-quality examples over the same CAD-derived plate seam. */
	public static function wovenCell():Bytes return encode(new WeldingCell(), (cell, scene) -> [weldStep(cell, scene.metresPerUnit, null, 7)]);
	public static function multipassCell():Bytes return encode(new WeldingCell(), (cell, scene) -> [weldStep(cell, scene.metresPerUnit, null, 10)]);

	/** The cell with a mission that welds the first post's perimeter only. */
	public static function postCell():Bytes return encode(new WeldingCell(), (cell, scene) -> [postStep(cell, scene.metresPerUnit)]);

	static function encode(cell:WeldingCell, steps:(WeldingCell, SceneArtifactData) -> Array<materia.project.SceneArtifact.SceneArtifactMissionStep>):Bytes {
		var scene = AssemblyPreview.scene(cell, ASSEMBLY_ID);
		scene.assemblyState = readyState(cell).record();
		scene.robotTools = RobotScene.robotTools(cell.arm.tool, TOOL_PREFIX, cell.equipment());
		scene.mission = {steps: steps(cell, scene)};
		return SceneArtifact.encode(scene);
	}

	/** The wire the cell's feeder declares, which the recipes are made for. */
	public static function wireOf(cell:WeldingCell):machinekit.welding.WeldingRecipe.RecipeWire
		return {diameterMm: cell.feeder.wireDiameterMm, depositionEfficiency: cell.feeder.depositionEfficiency,
			maxSpeedMPerMin: cell.feeder.maxSpeedMPerMin};

	/** The cell's state with the arm at its ready pose. */
	public static function readyState(cell:WeldingCell):AssemblyState {
		var model = new AssemblyModel("mm");
		cell.addTo(model, "");
		var state = new AssemblyState(model.definition(ASSEMBLY_ID));
		RobotWelderChecks.pose(state, cell.readyPose());
		return state;
	}

	/** The cell's seams, found from the members' geometry, in the workpiece's frame (its reference member's). */
	public static function seamsOf(cell:WeldingCell, ?weldment:Weldment):WeldmentSeams
		return (weldment == null ? cell.weldment() : weldment).findIn(cell, cell.solvedPoses(readyState(cell)));

	/** Where the torch's wire tip is, pointing where, with the arm at its ready pose: in the workpiece's frame. */
	public static function readyPlace(cell:WeldingCell):TorchPlace {
		var state = readyState(cell);
		var arc = EndEffectorControls.derive(cell.arm.tool, TOOL_PREFIX).arcs[0];
		var tip = state.worldConnector(TOOL_PREFIX + "/" + arc.member, arc.tcpConnector);
		var work = AssemblyFrames.inverse(state.worldPose(cell.weldment().reference));
		var at = AssemblyFrames.transformPoint(work, tip.x, tip.y, tip.z);
		var wire = AssemblyFrames.transformVector(AssemblyFrames.compose(work, tip), 0, 0, 1);
		return {position: new Vector(at.x, at.y, at.z), wire: new Vector(wire.x, wire.y, wire.z)};
	}

	/**
	 * The mission that welds the whole weldment: every seam its declared joints have, in the order `WeldingMission` finds, with
	 * the weld metal part and the reference member read from the CAD. `access`, when given, judges every seam (reach and
	 * clearance); the preview does not ask for it (it is a check of the design, `RobotWelderChecks`).
	 */
	public static function mission(cell:WeldingCell, metresPerUnit:Float, ?access:WeldAccess, ?declared:Weldment):WeldingMission {
		var weldment = declared == null ? cell.weldment() : declared;
		return WeldingMission.generate(weldment, seamsOf(cell, weldment), wireOf(cell), WeldMetal.carrierOf(cell, weldment), metresPerUnit,
			readyPlace(cell), access);
	}

	/**
	 * A mission step that welds one seam of the T-joint, `WELD_SEAM` unless another is named. The seam is found from the members'
	 * geometry and given relative to the workpiece's reference member, so a player finds it where the workpiece stands; it is
	 * welded with the recipe for the leg the weldment asks for, in the wire the cell's feeder declares.
	 */
	public static function weldStep(cell:WeldingCell, metresPerUnit:Float, ?name:String, ?leg:Float):materia.project.SceneArtifact.SceneArtifactMissionStep {
		var wanted = name == null ? WELD_SEAM : name;
		var found = [for (item in seamsOf(cell).require()) if (item.name() == wanted) item];
		if (found.length != 1) throw 'The cell has no seam "$wanted"';
		var weldment = cell.weldment();
		var seam = leg == null ? found[0] : found[0].configured(leg);
		return WeldingRecipe.passStep(seam.legSize, wireOf(cell), [seam], weldment.reference, WeldMetal.carrierOf(cell, weldment), metresPerUnit);
	}

	/**
	 * A weld of one tube post's perimeter: its four sides, chained into one closed loop (`WeldSeams.chains`) and welded as
	 * one step. The sides lie between different pairs of faces, so the torch's angles change at every corner; the
	 * path turns it there. `post` counts the closed chains the weldment has, in the order they are found.
	 */
	public static function postStep(cell:WeldingCell, metresPerUnit:Float, post:Int = 0):materia.project.SceneArtifact.SceneArtifactMissionStep {
		var loops = [for (chain in WeldSeams.chains(seamsOf(cell).require())) if (chain.closed) chain];
		if (post >= loops.length) throw 'The cell has ${loops.length} posts to weld, not ${post + 1}';
		var seams = loops[post].seams;
		var weldment = cell.weldment();
		return WeldingRecipe.fillet(seams[0].legSize, wireOf(cell)).step(seams, weldment.reference, WeldMetal.carrierOf(cell, weldment), metresPerUnit);
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
	/** The torch's parts: the mount and handle, the swan neck, and the nozzle with its contact tip. */
	public static final TORCH_PARTS = ["arm/tool/torch", "arm/tool/neck", "arm/tool/nozzle"];
	public static final JOINTS = ["arm/j1", "arm/j2", "arm/j3", "arm/j4", "arm/j5", "arm/j6"];

	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void {
		if (!(Math.abs(actual - expected) <= tolerance))
			throw '$message: expected $expected, got $actual';
	}

	public static function run():Void {
		PosedParts.scope(posed -> {
			clearance = posed;
			try runChecks() catch (error:Dynamic) { clearance = null; throw error; }
			clearance = null;
		});
	}

	/** The members posed by this run, with their geometry built once each; closed when the run ends. */
	static var clearance:PosedParts;

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
		checkSeamMission(cell);
		checkPostMission(cell);
		checkRecipeWire();

		var model = new AssemblyModel("mm");
		cell.addTo(model, "");
		var state = new AssemblyState(model.definition(RobotWelderPreview.ASSEMBLY_ID));
		var ready = cell.readyPose();
		pose(state, ready);
		checkReadyPose(cell, state);
		var seams = WeldSeamChecks.run(cell, state);
		var poses = checkReach(cell, state, ready, seams);
		checkWholeMission(cell, scene, state, ready);
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
		var controls = EndEffectorControls.derive(cell.arm.tool, RobotWelderPreview.TOOL_PREFIX);
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
		near(welder.stickoutMm, controls.arcs[0].stickoutMm, "the stickout is the torch's", 0);
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
		if (bare.groundedWork.join(",") != cell.weldment().reference) throw 'Without welds only the clamped plate is grounded, got ${bare.groundedWork}';
		var open = new machinekit.assembly.MachineAssembly();
		open.addComponent("source", new WeldingPowerSource());
		open.addComponent("feeder", new machinekit.welding.WireFeeder());
		var failed = false;
		try WeldingEquipment.of(open, []) catch (_:Dynamic) failed = true;
		if (!failed) throw "A power source with no work clamp should be refused";
		Sys.println('robot welder: torch tool grounded on ${grounded.join(", ")}');
	}

	/**
	 * The single-seam cell welds one seam of the T-joint, found from the geometry, with the recipe for its leg: the travel speed
	 * that deposits an equal-leg fillet's cross-section from the wire speed, the torch at the seam frame's angles.
	 */
	static function checkSeamMission(cell:WeldingCell):Void {
		var scene = SceneArtifact.decode(RobotWelderPreview.seamCell());
		var mission = scene.mission;
		if (mission == null || mission.steps.length != 1 || mission.steps[0].kind != "weld" || mission.steps[0].weld == null)
			throw "The single-seam cell should carry a mission that welds one seam";
		var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast mission.steps[0].weld;
		if (weld.frame != cell.weldment().reference) throw 'The weld should be placed by the workpiece\'s reference member, got ${weld.frame}';
		if (weld.path.length != 1 || weld.path[0].seam != RobotWelderPreview.WELD_SEAM || weld.path[0].joint != "fillet")
			throw 'The mission should weld the plate/upright fillet, got ${[for (segment in weld.path) segment.seam]}';
		var segment = weld.path[0];
		near(weld.legSize, WeldingWorkpiece.LEG_SIZE * 0.001, "the seam's leg", 1e-12);
		var length = Math.sqrt(Math.pow(segment.stop.position[0] - segment.start.position[0], 2) + Math.pow(segment.stop.position[1] - segment.start.position[1], 2) +
			Math.pow(segment.stop.position[2] - segment.start.position[2], 2));
		near(length, WeldingWorkpiece.PLATE_LENGTH * 0.001, "the seam's length", 1e-6);
		var process = weld.passes[0].process;
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
	 * The default mission welds the whole weldment. Every seam its declared joints have is welded exactly once, as the runs
	 * `WeldSeams.chains` finds (the plate's two sides, and each post's perimeter as one closed run), each run one `weld` step
	 * relative to the weldment's reference member, with the weld metal part and the reference read from the CAD. The runs
	 * are ordered to spare the torch air travel and turning, which beats the order they were found in. Every seam is also
	 * weldable: reachable by the arm and clear of the cell. A joint that has no seam, a seam the arm cannot reach, and a
	 * joint type that is not deposited are errors naming the seam, not skipped.
	 */
	static function checkWholeMission(cell:WeldingCell, scene:SceneArtifactData, state:AssemblyState, ready:ArmPose):Void {
		var weldment = cell.weldment();
		var found = RobotWelderPreview.seamsOf(cell).require();
		var mission = scene.mission;
		var chains = WeldSeams.chains(found);
		if (mission == null || mission.steps.length != chains.length)
			throw 'The default mission should weld the weldment\'s ${chains.length} runs, one step each, got ${mission == null ? 0 : mission.steps.length}';
		var welded = new Map<String, Int>();
		var length = 0.0;
		var carrier = WeldMetal.carrierOf(cell, weldment);
		for (step in mission.steps) {
			var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast step.weld;
			if (step.kind != "weld" || weld == null) throw "The default mission should only weld";
			if (weld.frame != weldment.reference) throw 'A weld should be placed by the reference member ${weldment.reference}, got ${weld.frame}';
			if (weld.metal != carrier) throw 'A weld should lay its metal on $carrier, got ${weld.metal}';
			for (index in 0...weld.path.length) {
				var segment = weld.path[index];
				welded.set(segment.seam, (welded.exists(segment.seam) ? welded.get(segment.seam) : 0) + 1);
				if (index > 0) for (axis in 0...3)
					near(segment.start.position[axis], weld.path[index - 1].stop.position[axis], "a run's sides run on from one another", 1e-9);
				length += Math.sqrt(Math.pow(segment.stop.position[0] - segment.start.position[0], 2) + Math.pow(segment.stop.position[1] - segment.start.position[1], 2)
					+ Math.pow(segment.stop.position[2] - segment.start.position[2], 2));
			}
		}
		for (seam in found) if (welded.get(seam.name()) != 1) throw 'Seam ${seam.name()} should be welded once, not ${welded.exists(seam.name()) ? welded.get(seam.name()) : 0} times';
		var count = 0;
		for (_ in welded.keys()) count++;
		if (count != found.length) throw 'The mission welds $count seams, the weldment has ${found.length}';
		var expected = 0.0;
		for (seam in found) expected += seam.length();
		near(length * 1000, expected, "the mission welds the length of all the seams", 1e-6);

		// The order beats the order the runs were found in, in the air travel plus the cost of turning the torch.
		var home = RobotWelderPreview.readyPlace(cell);
		var generated = RobotWelderPreview.mission(cell, 0.001);
		var foundOrder = [for (chain in chains) new machinekit.welding.WeldingMission.WeldRun(chain.seams, chain.closed)];
		var baseline = WeldingMission.measure(foundOrder, home);
		var ordered = WeldingMission.measure(generated.runs, home);
		var cost = function(m:{air:Float, turned:Float}) return m.air + WeldingMission.TURN_COST * m.turned;
		if (!(cost(ordered) <= cost(baseline) + 1e-9)) throw 'The order should not be worse than the order found: ${cost(ordered)} against ${cost(baseline)}';

		// Every seam is weldable by the arm in this cell.
		var access = new CellAccess(cell, state, ready);
		var checked = RobotWelderPreview.mission(cell, 0.001, access);
		checked.require();
		if (checked.steps.length != mission.steps.length) throw "A mission of weldable seams should have a step per run";

		// A seam the arm cannot weld is an error naming it, and the mission is not generated.
		var refused = found[found.length - 1].name();
		var rejected = RobotWelderPreview.mission(cell, 0.001, new RejectingAccess(refused, "out of reach"));
		if (!rejected.diagnostics.hasErrors() || rejected.steps.length != 0) throw "A seam that cannot be welded should stop the mission";
		var message = "";
		try rejected.require() catch (error:Dynamic) message = Std.string(error);
		if (message.indexOf(refused) < 0 || message.indexOf("out of reach") < 0) throw 'The error should name the seam and why: $message';

		// A declared joint with no seam is reported too: the plate and the beam lie apart.
		var broken = cell.weldment().join("work/basePlate", "work/beam", 5);
		var missing = RobotWelderPreview.mission(cell, 0.001, null, broken);
		var text = "";
		try missing.require() catch (error:Dynamic) text = Std.string(error);
		if (text.indexOf("work/beam") < 0 || missing.steps.length != 0) throw 'A joint with no seam should stop the mission, naming the members: $text';
		Sys.println('robot welder: the default mission welds ${found.length} seams (${Math.round(expected)} mm) in ${generated.runs.length} runs; ' +
			'air travel ${Math.round(ordered.air)} mm and ${Math.round(ordered.turned * 180 / Math.PI)} degrees of turning against ${Math.round(baseline.air)} mm and ${Math.round(baseline.turned * 180 / Math.PI)} in the order found');
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
		// The neck and the nozzle are parts of their own, mated at the bend and on the neck's end: the contact tip ends a stickout short of the wire tip.
		var nozzle = state.worldPose("arm/tool/nozzle");
		var end = AssemblyFrames.transformPoint(nozzle, 0, 0, WeldingTorch.NOZZLE_LENGTH + WeldingTorch.TIP_PROTRUSION);
		near(end.x, tcp.x - wire.x * WeldingTorch.STICKOUT, "the contact tip ends a stickout short of the wire tip (x)", 1e-6);
		near(end.y, tcp.y - wire.y * WeldingTorch.STICKOUT, "the contact tip ends a stickout short of the wire tip (y)", 1e-6);
		near(end.z, tcp.z - wire.z * WeldingTorch.STICKOUT, "the contact tip ends a stickout short of the wire tip (z)", 1e-6);
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
		var workpiece = state.worldPose(cell.weldment().reference);
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
		var workpiece = state.worldPose(cell.weldment().reference);
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
					if (TORCH_PARTS.indexOf(id) < 0) for (other in fixed) checkApart(cell, state, id, other);
					if (TORCH_PARTS.indexOf(id) < 0 && !StringTools.startsWith(id, "arm/tool/")) for (part in TORCH_PARTS) checkApart(cell, state, part, id);
				}
				for (part in TORCH_PARTS) for (other in fixed) checkApart(cell, state, part, other);
			} catch (error:Dynamic) {
				throw 'At ${entry.label}: $error';
			}
		}
		pose(state, ready);
	}

	public static function checkApart(cell:WeldingCell, state:AssemblyState, a:String, b:String):Void {
		var volume = 0.0;
		if (clearance != null) volume = clearance.volume(cell, state, "Robot welder", a, b);
		else PosedParts.scope(parts -> { volume = parts.volume(cell, state, "Robot welder", a, b); });
		if (volume > 1e-3) throw '$a hits $b: ${Math.round(volume)} mm³';
	}

	/** Starting poses for the solver: the ready pose, and the arm leaning further out and in, tool kept down. */
	public static function seeds(ready:ArmPose):Array<ArmPose> {
		var result = [ready];
		for (shoulder in [0.4, 0.8, 1.2]) for (elbow in [-1.4, -0.9, -0.4]) for (turn in [0.0, 0.5, -0.5])
			result.push([turn, shoulder, elbow, 0, Math.PI - (shoulder - elbow), 0]);
		return result;
	}

	public static function pose(state:AssemblyState, angles:ArmPose):Void {
		for (i in 0...JOINTS.length) state.setJoint(JOINTS[i], angles[i]);
		state.forwardKinematics();
	}

	static function round2(value:Float):Float return Math.round(value * 100) / 100;

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

/**
 * Whether the arm in this cell can weld a seam: at its start, middle and end, and `approach` out along the wire from each,
 * the damped-least-squares IK finds the wire tip with the wire along the seam frame's, and at those poses the torch
 * (neck, nozzle) and the arm clear the fixed items and the workpiece. The same questions as the cell's reach and clearance
 * checks, asked of every seam in the direction it is welded. A planner on the real runtime decides the roll and the exact
 * path (`WeldPathPlanner`); this is the design check that no seam is out of the arm's reach or buried in the cell.
 */
class CellAccess implements WeldAccess {
	final cell:WeldingCell;
	final state:AssemblyState;
	final ready:ArmPose;
	final limits:Array<{lower:Float, upper:Float}>;
	final moving:Array<String>;
	final fixed:Array<String>;
	final torch:String;
	final approach:Float;

	public function new(cell:WeldingCell, state:AssemblyState, ready:ArmPose) {
		this.cell = cell;
		this.state = state;
		this.ready = ready;
		limits = [for (spec in cell.arm.specs) {lower: spec.lower, upper: spec.upper}];
		moving = [for (entry in cell.components()) if (StringTools.startsWith(entry.id, "arm/")) entry.id];
		moving.push("feeder");
		fixed = ["table", "source", "cylinder", "clamp", "fixtureNear", "fixtureFar"].concat(cell.weldment().members);
		var arc = EndEffectorControls.derive(cell.arm.tool, RobotWelderPreview.TOOL_PREFIX).arcs[0];
		torch = RobotWelderPreview.TOOL_PREFIX + "/" + arc.member;
		approach = WeldingRecipe.fillet(cell.weldment().joints[0].legSize, RobotWelderPreview.wireOf(cell)).approach;
	}

	public function problem(seam:WeldSeam):Null<String> {
		var workpiece = state.worldPose(cell.weldment().reference);
		var result:Null<String> = null;
		for (fraction in [0.0, 0.5, 1.0]) for (out in [0.0, approach]) if (result == null) {
			var frame = seam.frameAtParameter(fraction).transformed(workpiece);
			var wire = frame.wire();
			var target = {x: frame.position.x - wire.x * out, y: frame.position.y - wire.y * out, z: frame.position.z - wire.z * out};
			var where = '${Math.round(fraction * 100)}% along, ${Math.round(out)} mm out, at ${Math.round(target.x)}, ${Math.round(target.y)}, ${Math.round(target.z)}';
			var found:Null<Array<Float>> = null;
			for (seed in RobotWelderChecks.seeds(ready)) if (found == null)
				found = ArmIk.solve(state, RobotWelderChecks.JOINTS, limits, target, {x: wire.x, y: wire.y, z: wire.z}, seed);
			if (found == null) {
				result = 'out of reach of the torch ($where)';
				continue;
			}
			RobotWelderChecks.pose(state, found);
			try {
				for (id in moving) {
					var tool = RobotWelderChecks.TORCH_PARTS.indexOf(id) >= 0;
					if (!tool) for (other in fixed) RobotWelderChecks.checkApart(cell, state, id, other);
					if (!tool && !StringTools.startsWith(id, RobotWelderPreview.TOOL_PREFIX + "/"))
						for (part in RobotWelderChecks.TORCH_PARTS) RobotWelderChecks.checkApart(cell, state, part, id);
				}
				for (part in RobotWelderChecks.TORCH_PARTS) for (other in fixed) RobotWelderChecks.checkApart(cell, state, part, other);
			} catch (error:Dynamic) {
				result = 'the arm or torch hits the cell ($where): $error';
			}
		}
		RobotWelderChecks.pose(state, ready);
		return result;
	}
}

/** An access that refuses one seam, for the check that an unweldable seam is reported. */
class RejectingAccess implements WeldAccess {
	final name:String;
	final reason:String;

	public function new(name:String, reason:String) {
		this.name = name;
		this.reason = reason;
	}

	public function problem(seam:WeldSeam):Null<String> return seam.name() == name ? reason : null;
}

/** Standalone check of the robot welder example. */
function main():Void RobotWelderChecks.run();
