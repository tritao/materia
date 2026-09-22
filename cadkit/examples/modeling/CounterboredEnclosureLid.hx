import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.HoleFeature;
import cadkit.parametric.features.LinearPatternFeature;
import cadkit.parametric.recording.DocumentBuilder;

/** Resizable lid with four face-attached, through counterbored mounting holes. */
class CounterboredEnclosureLid {
	public final document:Document;
	public var lid(default, null):BoxFeature;
	public var hole(default, null):HoleFeature;
	public var holes(default, null):LinearPatternFeature;
	public var finish(default, null):BooleanFeature;

	private static inline var EDGE_CLEARANCE:Float = 6;

	public function new() {
		document = DocumentBuilder.build(function(builder) {
			var width = builder.dimension("lid.width", 60);
			var depth = builder.dimension("lid.depth", 40);
			var thickness = builder.dimension("lid.thickness", 6);
			var localX = builder.dimension("holes.local-x", 0);
			var localY = builder.dimension("holes.local-y", 0);
			var boreDiameter = builder.dimension("holes.bore-diameter", 4);
			var recessDiameter = builder.dimension("holes.recess-diameter", 8);
			var recessDepth = builder.dimension("holes.recess-depth", 2);
			var columns = builder.dimension("holes.columns", 2);
			var rows = builder.dimension("holes.rows", 2);
			var spacingX = builder.dimension("holes.spacing-x", width.value - 2 * EDGE_CLEARANCE);
			var spacingY = builder.dimension("holes.spacing-y", depth.value - 2 * EDGE_CLEARANCE);

			lid = builder.box(width, depth, thickness);
			hole = builder.throughCounterbore(lid,
				new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1), Vector.X(),
				localX, localY, boreDiameter, recessDiameter, recessDepth);
			holes = builder.linearPattern(hole, columns, spacingX, Vector.X(), rows, spacingY, Vector.Y());
			finish = builder.subtract(lid, holes);
			builder.output(finish);
		});
	}

	public function resize(width:Float, depth:Float, thickness:Float):Void {
		if (width <= 2 * EDGE_CLEARANCE + 8 || depth <= 2 * EDGE_CLEARANCE + 8 || thickness <= 2)
			throw new ParametricError("lid dimensions must contain the counterbored holes");
		var transaction = document.beginTransaction();
		try {
			document.parameter("lid.width").set(width);
			document.parameter("lid.depth").set(depth);
			document.parameter("lid.thickness").set(thickness);
			document.parameter("holes.spacing-x").set(width - 2 * EDGE_CLEARANCE);
			document.parameter("holes.spacing-y").set(depth - 2 * EDGE_CLEARANCE);
			document.recompute();
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
		transaction.commit();
	}

	public function close():Void {
		document.close();
	}
}
