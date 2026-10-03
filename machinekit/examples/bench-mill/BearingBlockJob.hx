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

/** The single-tool bearing-block program reads pocket outlines and floors from its solid. */
class BearingBlockJob {
	/** The stock block, as the mill clamps it, in millimetres. */
	static final WIDTH = BenchMill.STOCK_WIDTH;
	static final DEPTH = BenchMill.STOCK_DEPTH;
	static final HEIGHT = BenchMill.STOCK_HEIGHT;
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
	public static function program(router:BenchMill, plate:BearingBlock, part:Part):String {
		var faces = part.shape.faces().all();
		// Work coordinates put the stock's front-left top corner at zero.
		var shiftX = WIDTH / 2, shiftY = DEPTH / 2, top = HEIGHT;
		var upward = [for (face in faces) if (face.surfaceKind() == CadKit.SurfaceKind.Plane &&
			face.normal().get_z() > 1 - 1e-9) face];
		var topFace = [for (face in upward) if (Math.abs(face.center().get_z() - top) < 1e-6) face];
		if (topFace.length != 1) throw "The bearing block should have one top face";
		var outlines = CamContour.fromFaceBoundaries(topFace[0], "mm", CHORD).slice(1);
		if (outlines.length == 0) throw "The bearing block's top face has no recesses";
		var tools = router.tools(), mill = tools[0];
		var setup = new Setup("1", new Point3(0, 0, 0),
			new SetupStock(0.0, WIDTH / 1000, 0.0, DEPTH / 1000, 0.0, -HEIGHT / 1000, 0.02, router.fixtures()));
		// The machine starts at its initial pose, which in work coordinates is above the stock's middle.
		var offset = router.workOffset();
		var start = [for (spec in router.specs) spec.initial];
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
		return CncWriter.write(job.finish(setup), controller);
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
