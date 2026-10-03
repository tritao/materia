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
	static function posed(mill:BenchMill, state:AssemblyState, id:String):Part {
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
function main():Void BenchMillChecks.run();
