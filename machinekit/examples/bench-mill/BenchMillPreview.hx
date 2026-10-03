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
	/** Mechanical preview until MT5 supplies the jaw's real clamping force. */
	public static function enclosure():Bytes
		return SceneArtifact.encode(AssemblyPreview.scene(new EnclosedBenchMill(), ASSEMBLY_ID));

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
			controller: {stepTickHz: BenchMill.STEP_TICK_HZ}};
		return SceneArtifact.encode(scene);
	}

}

class BenchMillChecks {
	public static function run():Void {
		var mill = new BenchMill();
		var model = new AssemblyModel("mm");
		mill.addTo(model, "");
		var state = new AssemblyState(model.definition(BenchMillPreview.ASSEMBLY_ID));
		for (x in [0.0, 250]) for (y in [0.0, 150]) for (z in [-250.0, 0]) {
			state.setJoint("x", x); state.setJoint("y", y); state.setJoint("z", z);
			state.forwardKinematics();
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
				var expected = 2 * Math.PI / 5 * [x, y, z][index];
				if (Math.abs(state.joint(id + "-turn") - expected) > 1e-6) throw '$id ratio is wrong';
			}
		}
		for (spec in mill.specs) {
			if (!(mill.axisOvertravel(spec.id) > 0)) throw "Mill rail has no overtravel";
			var suffix = spec.id.toUpperCase();
			var screw:machinekit.motion.BallScrew = null;
			var motor:machinekit.motion.ServoMotor = null;
			for (entry in mill.components()) {
				if (entry.id == "screw" + suffix) screw = cast entry.component;
				if (entry.id == "motor" + suffix) motor = cast entry.component;
			}
			if (screw == null || motor == null) throw "Mill axis has no screw or servo";
			var rapid = Math.min(screw.criticalSpeed(Fixed, Simple), motor.rating.maxSpeed) * 5 / (2 * Math.PI);
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

}

/** Standalone mechanical and CAM checks, also called by MachineKit smoke. */
function main():Void { BenchMillChecks.run(); EnclosedMillChecks.run(); }

/** Clear openings and a datum taken from the actual vise parts. */
class EnclosedMillChecks {
	public static function run():Void {
		var cell = new EnclosedBenchMill(), state = cell.state();
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
		var head = state.worldPose("mill/head");
		if (head.z < cell.opening.z + cell.openingHeight) throw "Retracted head crosses the doorway";
		var vise = cell.mill.vise;
		if (vise == null) throw "Enclosed mill has no vise";
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
		var gauge = state.worldConnector("mill/spindle", "gaugeLine");
		var offset = cell.mill.workOffset();
		if (Math.abs(offset[0] - datum.x + gauge.x) > 1e-9 || Math.abs(offset[1] - datum.y + gauge.y) > 1e-9 ||
			Math.abs(offset[2] - datum.z - BenchMill.STOCK_HEIGHT + gauge.z) > 1e-9)
			throw "G54 does not follow the vise datum";
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
		var definition:materia.assembly.AssemblyDefinition = cast scene.assemblyDefinition;
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
