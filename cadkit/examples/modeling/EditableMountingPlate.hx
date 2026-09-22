import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.features.GridFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.modeling.Vector;
import sys.io.File;

/** Reusable document example: dimensions, hole spacing, thickness and fillets are parameters. */
class EditableMountingPlate {
	public final document:Document;
	public var outline(default, null):SketchFeature;
	public var hole(default, null):SketchFeature;
	public var holes(default, null):GridFeature;
	public var extrusion(default, null):ExtrudeFeature;
	public var finish(default, null):FilletFeature;

	public function new() {
		document = DocumentBuilder.build(function(builder) {
			var width = builder.dimension("plate.width", 80);
			var depth = builder.dimension("plate.depth", 50);
			var holeRadius = builder.dimension("holes.radius", 3);
			var spacingX = builder.dimension("holes.spacingX", 60);
			var spacingY = builder.dimension("holes.spacingY", 30);
			var thickness = builder.dimension("plate.thickness", 6);
			var filletRadius = builder.dimension("fillet.radius", 2);

			outline = builder.rectangle(width, depth);
			hole = builder.circle(holeRadius);
			holes = builder.grid(hole, 2, 2, spacingX, spacingY);
			var profile = builder.subtract(outline, holes);
			extrusion = builder.extrude(profile, thickness);
			finish = builder.fillet(extrusion, filletRadius, new SelectionRecipe("edge", "line", Vector.Z(), "ends", Vector.X(), 4));
			builder.output(finish);
		});
	}

	/** Validate a grouped edit through staged recompute before committing undo history. */
	public function resize(width:Float, depth:Float, spacingX:Float, spacingY:Float):Void {
		if (!Math.isFinite(width)
			|| !Math.isFinite(depth)
			|| !Math.isFinite(spacingX)
			|| !Math.isFinite(spacingY)
			|| spacingX <= 2 * hole.width.value
			|| spacingY <= 2 * hole.width.value
			|| width <= spacingX + 2 * (hole.width.value + finish.radius.value)
			|| depth <= spacingY + 2 * (hole.width.value + finish.radius.value))
			throw new ParametricError("holes must be separated and clear of the rounded plate boundary");
		var transaction = document.beginTransaction();
		try {
			document.parameter("plate.width").set(width);
			document.parameter("plate.depth").set(depth);
			document.parameter("holes.spacingX").set(spacingX);
			document.parameter("holes.spacingY").set(spacingY);
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

	static function main():Int {
		var model = new EditableMountingPlate();
		try {
			model.resize(100, 60, 70, 40);
			File.saveContent("editable-mounting-plate.json", DocumentCodec.encode(model.document));
			model.finish.currentShape().exportStep("editable-mounting-plate.step");
			model.document.undo();
			model.document.recompute();
			model.document.redo();
			model.document.recompute();
			var loaded = DocumentCodec.decode(File.getContent("editable-mounting-plate.json"));
			try {
				loaded.featureAt(loaded.featureCount() - 1).currentShape().exportStep("editable-mounting-plate-reloaded.step");
				loaded.close();
			} catch (error:Dynamic) {
				loaded.close();
				throw error;
			}
			model.close();
		} catch (error:Dynamic) {
			model.close();
			throw error;
		}
		return 0;
	}
}
