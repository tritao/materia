import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.NamedParameter;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.PocketFeature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/** Resizable enclosure with a blind top recess and two through-holes on one side. */
class AttachedPocketEnclosure {
	public final document:Document;
	public var enclosure(default, null):BoxFeature;
	public var topProfile(default, null):ConstrainedSketchFeature;
	public var topPocket(default, null):PocketFeature;
	public var sideProfile(default, null):ConstrainedSketchFeature;
	public var finish(default, null):PocketFeature;

	public function new() {
		document = DocumentBuilder.build(function(builder) {
			var width = builder.dimension("enclosure.width", 60);
			var depth = builder.dimension("enclosure.depth", 40);
			var height = builder.dimension("enclosure.height", 30);
			var recessWidth = builder.dimension("recess.width", 30);
			var recessDepth = builder.dimension("recess.depth", 20);
			var recessHeight = builder.dimension("recess.height", 5);
			var holeRadius = builder.dimension("side-holes.radius", 3);

			enclosure = builder.box(width, depth, height);
			var topDimensions:Map<String, NamedParameter> = new Map();
			topDimensions.set("recess.width", recessWidth);
			topDimensions.set("recess.depth", recessDepth);
			topProfile = builder.attachedConstrainedSketch(topSketch(), enclosure,
				new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1),
				Vector.X(), topDimensions);
			topPocket = builder.pocket(enclosure, topProfile, recessHeight);

			var sideDimensions:Map<String, NamedParameter> = new Map();
			sideDimensions.set("side-holes.radius", holeRadius);
			sideProfile = builder.attachedConstrainedSketch(sideSketch(), topPocket,
				new SelectionRecipe("face", "plane", Vector.X(), "max", Vector.X(), 1),
				Vector.Y(), sideDimensions);
			finish = builder.pocketThroughAll(topPocket, sideProfile);
			builder.output(finish);
		});
	}

	private static function topSketch():ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		for (point in [
			new SketchPoint("recess.bl", -15, -10), new SketchPoint("recess.br", 15, -10),
			new SketchPoint("recess.tr", 15, 10), new SketchPoint("recess.tl", -15, 10)
		])
			sketch.addPoint(point);
		sketch.addEntity(SketchEntity.line("recess.bottom", "recess.bl", "recess.br"))
			.addEntity(SketchEntity.line("recess.right", "recess.br", "recess.tr"))
			.addEntity(SketchEntity.line("recess.top", "recess.tr", "recess.tl"))
			.addEntity(SketchEntity.line("recess.left", "recess.tl", "recess.bl"));
		sketch.addConstraint(SketchConstraint.fixed("recess.origin", "recess.bl"))
			.addConstraint(SketchConstraint.horizontal("recess.bottom.horizontal", "recess.bottom"))
			.addConstraint(SketchConstraint.horizontal("recess.top.horizontal", "recess.top"))
			.addConstraint(SketchConstraint.vertical("recess.right.vertical", "recess.right"))
			.addConstraint(SketchConstraint.vertical("recess.left.vertical", "recess.left"))
			.addConstraint(SketchConstraint.distance("recess.width", "recess.bl", "recess.br", 30))
			.addConstraint(SketchConstraint.distance("recess.depth", "recess.br", "recess.tr", 20));
		return sketch;
	}

	private static function sideSketch():ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		sketch.addPoint(new SketchPoint("side-hole.left", -10, 0))
			.addPoint(new SketchPoint("side-hole.right", 10, 0))
			.addEntity(SketchEntity.circle("side-hole.left.circle", "side-hole.left", 3))
			.addEntity(SketchEntity.circle("side-hole.right.circle", "side-hole.right", 3))
			.addConstraint(SketchConstraint.fixed("side-hole.left.fixed", "side-hole.left"))
			.addConstraint(SketchConstraint.fixed("side-hole.right.fixed", "side-hole.right"))
			.addConstraint(SketchConstraint.radius("side-holes.radius", "side-hole.left.circle", 3))
			.addConstraint(SketchConstraint.equal("side-holes.equal", "side-hole.left.circle", "side-hole.right.circle"));
		return sketch;
	}

	public function resize(width:Float, depth:Float, height:Float):Void {
		if (width <= 40 || depth <= 25 || height <= 15)
			throw new ParametricError("enclosure dimensions must contain the recess and side holes");
		var transaction = document.beginTransaction();
		try {
			document.parameter("enclosure.width").set(width);
			document.parameter("enclosure.depth").set(depth);
			document.parameter("enclosure.height").set(height);
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
