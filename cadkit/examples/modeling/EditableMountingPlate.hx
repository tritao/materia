import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.ParametricError;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.features.GridFeature;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.modeling.Vector;
import sys.io.File;

/** Reusable document example: dimensions, hole spacing, thickness and fillets are parameters. */
class EditableMountingPlate {
	public final document:Document;
	public final outline:SketchFeature;
	public final hole:SketchFeature;
	public final holes:GridFeature;
	public final extrusion:ExtrudeFeature;
	public final finish:FilletFeature;

	public function new() {
		document = new Document();
		try {
			outline = document.add(new SketchFeature("rectangle", 80, 50));
			hole = document.add(new SketchFeature("circle", 3));
			holes = document.add(new GridFeature(hole, 2, 2, 60, 30));
			var profile = document.add(new BooleanFeature(outline, holes, BooleanOperation.Cut));
			extrusion = document.add(new ExtrudeFeature(profile, 0, 0, 6));
			finish = document.add(new FilletFeature(extrusion, 2, null, null, new SelectionRecipe("edge", "line", Vector.Z(), "ends", Vector.X(), 4)));
			document.recompute();
		} catch (error:Dynamic) {
			document.close();
			throw error;
		}
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
			outline.width.set(width);
			outline.height.set(depth);
			holes.spacingX.set(spacingX);
			holes.spacingY.set(spacingY);
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
