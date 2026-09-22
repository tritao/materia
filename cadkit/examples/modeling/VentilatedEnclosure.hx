import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.NamedParameter;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.LinearPatternFeature;
import cadkit.parametric.features.PocketFeature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/** Resizable enclosure with a workplane-aware row of through ventilation slots. */
class VentilatedEnclosure {
	public final document:Document;
	public var enclosure(default, null):BoxFeature;
	public var slot(default, null):ConstrainedSketchFeature;
	public var slots(default, null):LinearPatternFeature;
	public var finish(default, null):PocketFeature;

	public function new() {
		document = DocumentBuilder.build(function(builder) {
			var width = builder.dimension("enclosure.width", 60);
			var depth = builder.dimension("enclosure.depth", 40);
			var height = builder.dimension("enclosure.height", 5);
			var count = builder.dimension("vents.count", 5);
			var spacing = builder.dimension("vents.spacing", 8);
			enclosure = builder.box(width, depth, height);
			slot = builder.attachedConstrainedSketch(slotSketch(), enclosure,
				new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1),
				Vector.X(), new Map<String, NamedParameter>());
			slots = builder.linearPattern(slot, count, spacing, Vector.X());
			finish = builder.pocketThroughAll(enclosure, slots);
			builder.output(finish);
		});
	}

	private static function slotSketch():ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		for (point in [
			new SketchPoint("vent.bl", -2, -10), new SketchPoint("vent.br", 2, -10),
			new SketchPoint("vent.tr", 2, 10), new SketchPoint("vent.tl", -2, 10)
		])
			sketch.addPoint(point);
		sketch.addEntity(SketchEntity.line("vent.bottom", "vent.bl", "vent.br"))
			.addEntity(SketchEntity.line("vent.right", "vent.br", "vent.tr"))
			.addEntity(SketchEntity.line("vent.top", "vent.tr", "vent.tl"))
			.addEntity(SketchEntity.line("vent.left", "vent.tl", "vent.bl"));
		for (index in 0...4)
			sketch.addConstraint(SketchConstraint.fixed("vent.fixed" + index,
				["vent.bl", "vent.br", "vent.tr", "vent.tl"][index]));
		return sketch;
	}

	public function resize(width:Float, count:Int, spacing:Float):Void {
		if (width <= 0 || count < 1 || spacing <= 4 || (count - 1) * spacing + 4 >= width)
			throw new ParametricError("vent slots must be separated and remain inside the enclosure");
		var transaction = document.beginTransaction();
		try {
			document.parameter("enclosure.width").set(width);
			document.parameter("vents.count").set(count);
			document.parameter("vents.spacing").set(spacing);
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
