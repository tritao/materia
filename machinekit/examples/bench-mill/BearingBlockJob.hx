import CadKit;
import cadkit.Face;
import cadkit.modeling.Part;
import camkit.CamContour;
import camkit.CamJob;
import cnckit.CncController;
import cnckit.CncWriter;
import toolpathkit.path.Point3;
import toolpathkit.setup.Setup;
import toolpathkit.setup.SetupStock;

typedef BearingBlockSetup = {
 var tools:Array<toolpathkit.tool.Tool>;
 var fixtures:Array<toolpathkit.setup.Fixture>;
 /** Machine coordinates in millimetres; work origin is the blank's front-left top corner. */
 var workOffset:Array<Float>;
 var start:Array<Float>;
}

/** The single-tool bearing-block program reads pocket outlines and floors from its solid. */
class BearingBlockJob {
	/**
	 * How closely the part's round edges are followed, in metres: CAM samples them within this, and fits
	 * its cutting lines back into arcs within it, so circles are cut as G2/G3.
	 */
	static inline final CHORD = 0.00002;
	/**
	 * How far cutting moves may round the corners left between them (G64 P), in metres: half the 0.02 mm
	 * the app counts as on target.
	 */
	static inline final BLEND = 0.00001;

	/** The G-code that makes the plate's features on `mill`, read from its faces. */
	public static function program(router:BenchMill, plate:BearingBlock, part:Part, ?loadPosition:Array<Float>):String {
    return generate({tools:router.tools(),fixtures:router.fixtures(),workOffset:router.workOffset(),
      start:[for(spec in router.specs)spec.initial]},plate,part,loadPosition);
  }

  public static function generate(binding: BearingBlockSetup, plate:BearingBlock, part:Part, ?loadPosition:Array<Float>):String {
    var width=plate.width,depth=plate.depth,height=plate.height;
		var faces = part.shape.faces().all();
		// Work coordinates put the stock's front-left top corner at zero.
		var shiftX = width / 2, shiftY = depth / 2, top = height;
		var upward = [for (face in faces) if (face.surfaceKind() == CadKit.SurfaceKind.Plane &&
			face.normal().get_z() > 1 - 1e-9) face];
		var topFace = [for (face in upward) if (Math.abs(face.center().get_z() - top) < 1e-6) face];
		if (topFace.length != 1) throw "The bearing block should have one top face";
		var outlines = CamContour.fromFaceBoundaries(topFace[0], "mm", CHORD).slice(1);
		if (outlines.length == 0) throw "The bearing block's top face has no recesses";
		var tools = binding.tools, mill = tools[0];
		var setup = new Setup("1", new Point3(0, 0, 0),
			new SetupStock(0.0, width / 1000, 0.0, depth / 1000, 0.0, -height / 1000, 0.02, binding.fixtures));
		// The machine starts at its initial pose, which in work coordinates is above the stock's middle.
		var offset = binding.workOffset;
		var start = binding.start;
		var job = new CamJob(0.02, 8000,
			new Point3((start[0] - offset[0]) / 1000, (start[1] - offset[1]) / 1000, (start[2] - offset[2]) / 1000),
			BLEND, CHORD);
		for (outline in outlines) {
			var floor = floorBelow(upward, outline, top);
			// The outline in work coordinates, and its depth to the floor found in the part.
			var shifted = new CamContour([for (point in outline.vertices)
				new Point3(point.x + shiftX / 1000, point.y + shiftY / 1000, 0.0)]);
			job.pocket(shifted, mill, (floor - top) / 1000, 0.02, 0.003, 0.002, 0.005);
		}
		var controller = new CncController();
		for (tool in tools) controller.toolLibrary.set(tool);
		var program = job.finish(setup);
		if (loadPosition != null) {
			var frame = toolpathkit.path.ToolpathFrame.of(program);
			var last = new Point3(start[0] / 1000, start[1] / 1000, start[2] / 1000);
			for (op in program.ops) {
				frame.advance(op);
				switch op {
					case Move(_, geometry, _, _, _):
						var p = toolpathkit.path.GeometryTools.pointAt(geometry, toolpathkit.path.GeometryTools.length(geometry));
						var shift = frame.toMachine();
						// Scene G54 is external to the CAM setup's work-coordinate origin.
						last = new Point3(p.x + shift[0] + offset[0] / 1000,
							p.y + shift[1] + offset[1] / 1000, p.z + shift[2] + offset[2] / 1000);
					case _:
				}
			}
			var end = program.ops.pop();
			var raised = new Point3(last.x, last.y, loadPosition[2] / 1000);
			var load = new Point3(loadPosition[0] / 1000, loadPosition[1] / 1000, loadPosition[2] / 1000);
			var span = toolpathkit.path.Provenance.cam(program.ops.length + 1, "vise-load-position");
			program.ops.push(toolpathkit.path.ToolpathOp.MachineMove(Rapid, Line(last, raised), 0, 0, span));
			program.ops.push(toolpathkit.path.ToolpathOp.MachineMove(Rapid, Line(raised, load), 0, 0, span));
			program.ops.push(end);
		}
		return CncWriter.write(program, controller);
	}

	/** Height of the floor under a recess outline: the upward face whose centre lies inside it. */
	static function floorBelow(upward:Array<Face>, outline:CamContour, top:Float):Float {
		var cx = 0.0, cy = 0.0;
		for (point in outline.vertices) { cx += point.x * 1000; cy += point.y * 1000; }
		cx /= outline.vertices.length;
		cy /= outline.vertices.length;
		for (face in upward) {
			var center = face.center();
			if (center.get_z() < top - 1e-6 && Math.abs(center.get_x() - cx) < 1e-3 && Math.abs(center.get_y() - cy) < 1e-3)
				return center.get_z();
		}
		throw 'No floor under the recess at ($cx, $cy)';
	}
}
