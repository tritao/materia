import CadKit;
import cadkit.Face;
import cadkit.modeling.Part;
import camkit.CamContour;
import camkit.CamJob;
import cnckit.CncController;
import cnckit.CncWriter;
import toolpathkit.path.Point3;
import toolpathkit.setup.Fixture;
import toolpathkit.setup.Setup;
import toolpathkit.setup.SetupStock;

/**
 * Programs the router's job from the part it makes. The motor plate is modelled in the stock block's
 * frame; CAM reads its top face, whose inner boundaries are the recesses' outlines, finds each
 * recess's floor below it, and pockets each outline down to its floor with the router's end mill,
 * keeping clear of the step clamps. Then the drill makes the screws' clearance holes from the
 * counterbore floors through into the spoilboard. Work zero (G54) is the stock's front-left top
 * corner, and each tool is programmed at its tip through G43.
 */
class MountPlateJob {
	/** The stock block, as the router clamps it, in millimetres. */
	static final WIDTH = CncRouter.STOCK_WIDTH;
	static final DEPTH = CncRouter.STOCK_DEPTH;
	static final HEIGHT = CncRouter.STOCK_HEIGHT;
	/** How far the drill's full diameter goes past the plate's underside, in millimetres. */
	static inline final BREAKTHROUGH = 0.5;

	/** The G-code that makes the plate's features on `router`, read from its faces. */
	public static function program(router:CncRouter, plate:NemaMountPlate, part:Part):String {
		var faces = part.shape.faces().all();
		// Work coordinates put the stock's front-left top corner at zero.
		var shiftX = WIDTH / 2, shiftY = DEPTH / 2, top = HEIGHT;
		var upward = [for (face in faces) if (face.surfaceKind() == CadKit.SurfaceKind.Plane &&
			face.normal().get_z() > 1 - 1e-9) face];
		var topFace = [for (face in upward) if (Math.abs(face.center().get_z() - top) < 1e-6) face];
		if (topFace.length != 1) throw "The motor plate should have one top face";
		var outlines = CamContour.fromFaceBoundaries(topFace[0], "mm", 0.00002).slice(1);
		if (outlines.length == 0) throw "The motor plate's top face has no recesses";
		var tools = router.tools(), mill = tools[0], drill = tools[1];
		if (Math.abs(router.drill.diameter - plate.holeDiameter) > 1e-9)
			throw 'The router\'s ${router.drill.diameter} mm drill does not make the plate\'s ${plate.holeDiameter} mm holes';
		// The drill's point and then its full diameter go through into the spoilboard, which is there to be cut.
		var drillDepth = HEIGHT + TwistDrill.pointHeight(router.drill.diameter) + BREAKTHROUGH;
		// Step clamps hold the stock's left and right edges: bar and bolt, 18 mm above its top.
		var clamps = [new Fixture("clamp-left", -0.042, 0.008, 0.0325, 0.0575, 0.0, 0.018),
			new Fixture("clamp-right", 0.112, 0.162, 0.0325, 0.0575, 0.0, 0.018)];
		var setup = new Setup("1", new Point3(0, 0, 0),
			new SetupStock(0.0, WIDTH / 1000, 0.0, DEPTH / 1000, 0.0, -drillDepth / 1000, 0.02, clamps));
		// The machine starts at its initial pose, which in work coordinates is above the stock's middle.
		var offset = CncRouter.workOffset();
		var start = [for (spec in router.specs) spec.initial];
		var job = new CamJob(0.02, 12000,
			new Point3((start[0] - offset[0]) / 1000, (start[1] - offset[1]) / 1000, (start[2] - offset[2]) / 1000));
		for (outline in outlines) {
			var floor = floorBelow(upward, outline, top);
			// The outline in work coordinates, and its depth to the floor found in the part.
			var shifted = new CamContour([for (point in outline.vertices)
				new Point3(point.x + shiftX / 1000, point.y + shiftY / 1000, 0.0)]);
			job.pocket(shifted, mill, (floor - top) / 1000, 0.02, 0.003, 0.002, 0.005);
		}
		var holes = [for (point in plate.motor.boltPattern())
			new Point3((point.x + shiftX) / 1000, (point.y + shiftY) / 1000, -plate.counterboreDepth / 1000)];
		job.drill(holes, drill, -drillDepth / 1000, 0.002, 0.002);
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
