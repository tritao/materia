import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.NamedParameter;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/** Constrained rectangular bracket with a tangent capsule slot and two holes. */
class ConstrainedSlottedBracket {
	public final document:Document;
	public var profile(default, null):ConstrainedSketchFeature;
	public var extrusion(default, null):ExtrudeFeature;
	public var finish(default, null):FilletFeature;

	public function new() {
		document = DocumentBuilder.build(function(builder) {
			var width = builder.dimension("bracket.width", 80);
			var height = builder.dimension("bracket.height", 50);
			var holeRadius = builder.dimension("holes.radius", 3);
			var thickness = builder.dimension("bracket.thickness", 6);
			var filletRadius = builder.dimension("fillet.radius", 2);
			var dimensions:Map<String, NamedParameter> = new Map();
			dimensions.set("body.width", width);
			dimensions.set("body.height", height);
			dimensions.set("hole.radius", holeRadius);
			profile = builder.constrainedSketch(makeSketch(width.value, height.value, holeRadius.value), dimensions);
			extrusion = builder.extrude(profile, thickness);
			finish = builder.fillet(extrusion, filletRadius,
				new SelectionRecipe("edge", "line", Vector.Z(), "ends", Vector.X(), 4));
			builder.output(finish);
		});
	}

	private static function makeSketch(width:Float, height:Float, holeRadius:Float):ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		var halfWidth = width / 2;
		var halfHeight = height / 2;
		for (point in [
			new SketchPoint("body.bl", -halfWidth, -halfHeight), new SketchPoint("body.br", halfWidth, -halfHeight),
			new SketchPoint("body.tr", halfWidth, halfHeight), new SketchPoint("body.tl", -halfWidth, halfHeight),
			new SketchPoint("axis.x0", -1, 0), new SketchPoint("axis.x1", 1, 0),
			new SketchPoint("axis.y0", 0, -1), new SketchPoint("axis.y1", 0, 1),
			new SketchPoint("slot.left", -10, 0), new SketchPoint("slot.right", 10, 0),
			new SketchPoint("slot.bl", -10, -5), new SketchPoint("slot.br", 10, -5),
			new SketchPoint("slot.tl", -10, 5), new SketchPoint("slot.tr", 10, 5),
			new SketchPoint("hole.left", -28, 0), new SketchPoint("hole.right", 28, 0)
		])
			sketch.addPoint(point);

		sketch.addEntity(SketchEntity.line("body.bottom", "body.bl", "body.br"))
			.addEntity(SketchEntity.line("body.right", "body.br", "body.tr"))
			.addEntity(SketchEntity.line("body.top", "body.tr", "body.tl"))
			.addEntity(SketchEntity.line("body.left", "body.tl", "body.bl"))
			.addEntity(SketchEntity.line("axis.x", "axis.x0", "axis.x1", true))
			.addEntity(SketchEntity.line("axis.y", "axis.y0", "axis.y1", true))
			.addEntity(SketchEntity.line("slot.bottom", "slot.bl", "slot.br"))
			.addEntity(SketchEntity.arc("slot.rightArc", "slot.right", 5, -Math.PI / 2, Math.PI / 2))
			.addEntity(SketchEntity.line("slot.top", "slot.tl", "slot.tr"))
			.addEntity(SketchEntity.arc("slot.leftArc", "slot.left", 5, Math.PI / 2, 3 * Math.PI / 2))
			.addEntity(SketchEntity.circle("hole.leftCircle", "hole.left", holeRadius))
			.addEntity(SketchEntity.circle("hole.rightCircle", "hole.right", holeRadius));

		for (id in ["axis.x0", "axis.x1", "axis.y0", "axis.y1", "slot.left", "slot.right", "slot.bl", "slot.br", "slot.tl",
			"slot.tr", "hole.left", "hole.right"])
			sketch.addConstraint(SketchConstraint.fixed("fixed." + id, id));
		sketch.addConstraint(SketchConstraint.horizontal("body.bottom.horizontal", "body.bottom"))
			.addConstraint(SketchConstraint.horizontal("body.top.horizontal", "body.top"))
			.addConstraint(SketchConstraint.vertical("body.left.vertical", "body.left"))
			.addConstraint(SketchConstraint.vertical("body.right.vertical", "body.right"))
			.addConstraint(SketchConstraint.distance("body.width", "body.bl", "body.br", width))
			.addConstraint(SketchConstraint.distance("body.height", "body.br", "body.tr", height))
			.addConstraint(SketchConstraint.symmetric("body.bottom.centered", "body.bl", "body.br", "axis.y"))
			.addConstraint(SketchConstraint.symmetric("body.right.centered", "body.br", "body.tr", "axis.x"));
		sketch.addConstraint(SketchConstraint.tangent("slot.tangent.bottomRight", "slot.bottom", "slot.rightArc"))
			.addConstraint(SketchConstraint.tangent("slot.tangent.topRight", "slot.top", "slot.rightArc"))
			.addConstraint(SketchConstraint.tangent("slot.tangent.topLeft", "slot.top", "slot.leftArc"))
			.addConstraint(SketchConstraint.tangent("slot.tangent.bottomLeft", "slot.bottom", "slot.leftArc"))
			.addConstraint(SketchConstraint.radius("hole.radius", "hole.leftCircle", holeRadius))
			.addConstraint(SketchConstraint.equal("hole.equal", "hole.leftCircle", "hole.rightCircle"));
		return sketch;
	}

	public function resize(width:Float, height:Float, holeRadius:Float, thickness:Float, filletRadius:Float):Void {
		if (width <= 65 || height <= 25 || holeRadius <= 0 || holeRadius >= 8 || thickness <= 0 || filletRadius <= 0)
			throw new ParametricError("bracket dimensions must keep the slot and holes inside the body");
		var transaction = document.beginTransaction();
		try {
			document.parameter("bracket.width").set(width);
			document.parameter("bracket.height").set(height);
			document.parameter("holes.radius").set(holeRadius);
			document.parameter("bracket.thickness").set(thickness);
			document.parameter("fillet.radius").set(filletRadius);
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
