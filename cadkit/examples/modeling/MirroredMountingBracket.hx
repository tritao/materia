import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.HoleFeature;
import cadkit.parametric.features.MirrorFeature;
import cadkit.parametric.recording.DocumentBuilder;

/** Half-bracket workflow with fused body symmetry and mirrored counterbore tools. */
class MirroredMountingBracket {
	public final document:Document;
	public var half(default, null):BoxFeature;
	public var body(default, null):MirrorFeature;
	public var counterbore(default, null):HoleFeature;
	public var counterbores(default, null):MirrorFeature;
	public var finish(default, null):BooleanFeature;

	public function new() {
		document = DocumentBuilder.build(function(builder) {
			var halfWidth = builder.dimension("bracket.half-width", 40);
			var depth = builder.dimension("bracket.depth", 30);
			var height = builder.dimension("bracket.height", 8);
			var holeLocalX = builder.dimension("holes.local-x", 14);
			var holeLocalY = builder.dimension("holes.local-y", 0);
			var boreDiameter = builder.dimension("holes.bore-diameter", 4);
			var recessDiameter = builder.dimension("holes.recess-diameter", 8);
			var recessDepth = builder.dimension("holes.recess-depth", 2);

			half = builder.box(halfWidth, depth, height);
			body = builder.mirror(half, new Vector(), Vector.X(), "fuse");
			counterbore = builder.throughCounterbore(half,
				new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1), Vector.X(),
				holeLocalX, holeLocalY, boreDiameter, recessDiameter, recessDepth);
			counterbores = builder.mirror(counterbore, new Vector(), Vector.X(), "both");
			finish = builder.subtract(body, counterbores);
			builder.output(finish);
		});
	}

	public function resize(totalWidth:Float, holeSpacing:Float):Void {
		if (totalWidth <= 20 || holeSpacing <= 8 || holeSpacing >= totalWidth - 8)
			throw new ParametricError("bracket width and hole spacing must preserve edge clearance");
		var halfWidth = totalWidth / 2;
		var localX = holeSpacing / 2 - halfWidth / 2;
		var transaction = document.beginTransaction();
		try {
			document.parameter("bracket.half-width").set(halfWidth);
			document.parameter("holes.local-x").set(localX);
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
