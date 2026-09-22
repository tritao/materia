import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.ParametricError;
import cadkit.parametric.RecomputeError;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.features.WireFeature;
import cadkit.parametric.features.PolylineFeature;
import cadkit.parametric.features.LoftFeature;
import cadkit.parametric.features.SweepFeature;
import cadkit.parametric.features.OffsetFeature;
import cadkit.parametric.features.ShellFeature;
import cadkit.parametric.features.ProjectFeature;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.TransformFeature;
import cadkit.parametric.features.GridFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.ChamferFeature;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;

class DocumentModelingSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function near(actual:Float, expected:Float):Void {
		check(Math.abs(actual - expected) < 0.00001, "document measurement mismatch " + actual + " / " + expected);
	}

	static function valid(shape:Shape):Void {
		check(CadKit.shapeValidChecked(shape.borrowHandle()) != 0, "invalid feature result");
	}

	static function plateVolume(width:Float, depth:Float):Float {
		return (width * depth - (4 - Math.PI) * 4 - 36 * Math.PI) * 6;
	}

	public static function run():Void {
		var document = new Document();
		var disc = document.add(new SketchFeature("circle", 2));
		var wire = document.add(new WireFeature(disc));
		var top = document.add(new TransformFeature(wire, 0, 0, 5));
		var loft = document.add(new LoftFeature([wire, top]));
		var path = document.add(new PolylineFeature([new Vector(), new Vector(0, 0, 5)]));
		var sweep = document.add(new SweepFeature(disc, path));
		var offset = document.add(new OffsetFeature(wire, 1));
		var target = document.add(new SketchFeature("rectangle", 20, 20));
		var projected = document.add(new ProjectFeature(top, target, new Vector(0, 0, -1)));
		var box = document.add(new BoxFeature(10, 10, 10));
		var shell = document.add(new ShellFeature(box, -1, new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1)));
		var chamfer = document.add(new ChamferFeature(box, 0.5, null, null, new SelectionRecipe("edge", "line", Vector.Z(), "all", Vector.Z(), 4)));
		document.recompute();
		near(loft.currentShape().volume(), 20 * Math.PI);
		near(sweep.currentShape().volume(), 20 * Math.PI);
		near(shell.currentShape().volume(), 424);
		check(loft.provenance != null && sweep.provenance != null && offset.provenance != null && shell.provenance != null, "operation provenance");
		check(projected.provenance == null, "projection must not invent history");
		for (i in 0...document.featureCount())
			valid(document.featureAt(i).currentShape());
		var offsetEdge = offset.currentShape().subshape(CadKit.ShapeKind.Edge, 0);
		near(offsetEdge.edgeLength(), 6 * Math.PI);
		offsetEdge.close();
		var projectionEdge = projected.currentShape().subshape(CadKit.ShapeKind.Edge, 0);
		near(projectionEdge.edgeLength(), 4 * Math.PI);
		projectionEdge.close();
		var transaction = document.beginTransaction();
		disc.width.set(3);
		path.coordinate(1, 2).set(7);
		top.z.set(6);
		shell.thickness.set(-2);
		transaction.commit();
		document.recompute();
		near(loft.currentShape().volume(), 54 * Math.PI);
		near(sweep.currentShape().volume(), 63 * Math.PI);
		near(shell.currentShape().volume(), 712);
		check(document.undo(), "advanced undo");
		document.recompute();
		near(loft.currentShape().volume(), 20 * Math.PI);
		check(document.redo(), "advanced redo");
		document.recompute();
		near(sweep.currentShape().volume(), 63 * Math.PI);
		var serialized = DocumentCodec.encode(document);
		var loaded = DocumentCodec.decode(serialized);
		check(loaded.featureCount() == document.featureCount(), "feature graph roundtrip");
		for (i in 0...document.featureCount()) {
			var original = document.featureAt(i).currentShape();
			var restored = loaded.featureAt(i).currentShape();
			valid(restored);
			near(restored.area(), original.area());
			near(restored.volume(), original.volume());
		}
		loaded.close();
		var committed = loft.currentShape();
		var committedDisc = disc.currentShape();
		var failed = false;
		transaction = document.beginTransaction();
		disc.width.set(4);
		offset.distance.set(0);
		transaction.commit();
		try {
			document.recompute();
		} catch (error:Dynamic) {
			var failure:RecomputeError = cast error;
			failed = failure.feature.toInt() == offset.id.toInt();
		}
		check(failed && committed == loft.currentShape() && committedDisc == disc.currentShape() && !committed.isClosed(),
			"failed recompute preserves committed shapes");
		near(committed.volume(), 54 * Math.PI);
		near(committedDisc.area(), 9 * Math.PI);
		check(document.undo(), "failed parameter undo");
		document.recompute();
		valid(offset.currentShape());
		var ambiguous = new SelectionRecipe("face", "plane", Vector.Z(), "all", Vector.Z(), 1);
		var reported = false;
		try {
			var values = ambiguous.resolve(box.currentShape());
			for (value in values)
				value.close();
		} catch (error:Dynamic) {
			var failure:ParametricError = cast error;
			reported = failure.referenceState == ReferenceState.Ambiguous;
		}
		check(reported, "ambiguous selector must fail explicitly");
		var missing = new SelectionRecipe("edge", "circle", null, "all", Vector.Z(), 1);
		reported = false;
		try {
			var values = missing.resolve(box.currentShape());
			for (value in values)
				value.close();
		} catch (error:Dynamic) {
			var failure:ParametricError = cast error;
			reported = failure.referenceState == ReferenceState.Unresolved;
		}
		check(reported, "missing selector must fail explicitly");
		// Dependency arrays are immutable from the caller's perspective.
		var deps = loft.dependencyFeatures();
		deps.resize(0);
		check(loft.dependencies().length == 2, "loft dependencies must be copied");
		document.close();

		var splitDocument = new Document();
		var splitBase = splitDocument.add(new BoxFeature(10, 10, 10));
		var splitTool = splitDocument.add(new BoxFeature(2, 20, 20));
		var splitPlacement = splitDocument.add(new TransformFeature(splitTool, 20, -5, -5));
		var splitCut = splitDocument.add(new BooleanFeature(splitBase, splitPlacement, BooleanOperation.Cut));
		var splitFinish = splitDocument.add(new FilletFeature(splitCut, 0.5, null, null,
			new SelectionRecipe("edge", "line", Vector.Z(), "all", Vector.Z(), 4)));
		var booleanShell = splitDocument.add(new ShellFeature(splitCut, -1, new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1)));
		splitDocument.recompute();
		near(booleanShell.currentShape().volume(), 424);
		var oldFinish = splitFinish.currentShape();
		splitPlacement.x.set(4);
		reported = false;
		try {
			splitDocument.recompute();
		} catch (error:Dynamic) {
			var failure:RecomputeError = cast error;
			reported = failure.referenceState == ReferenceState.Ambiguous;
		}
		check(reported && oldFinish == splitFinish.currentShape(), "split topology must reject changed selection cardinality");
		check(splitDocument.undo(), "split undo");
		splitDocument.recompute();
		transaction = splitDocument.beginTransaction();
		splitTool.width.set(20);
		splitPlacement.x.set(-5);
		transaction.commit();
		reported = false;
		try {
			splitDocument.recompute();
		} catch (error:Dynamic) {
			var failure:RecomputeError = cast error;
			reported = failure.referenceState == ReferenceState.Unresolved;
		}
		check(reported, "removed geometry must reject missing selections");
		check(splitDocument.undo(), "removed geometry undo");
		splitDocument.recompute();
		valid(splitFinish.currentShape());
		splitDocument.close();

		var plate = new EditableMountingPlate();
		near(plate.finish.currentShape().volume(), plateVolume(80, 50));
		plate.resize(100, 60, 70, 40);
		near(plate.finish.currentShape().volume(), plateVolume(100, 60));
		var holeFaces = plate.holes.currentShape().faces();
		check(holeFaces.count() == 4, "grid face count");
		for (i in 0...holeFaces.count()) {
			var face = holeFaces.at(i);
			var center = face.center();
			near(Math.abs(center.get_x()), 35);
			near(Math.abs(center.get_y()), 20);
			face.close();
		}
		check(plate.document.undo(), "plate grouped undo");
		plate.document.recompute();
		near(plate.outline.width.value, 80);
		near(plate.holes.spacingX.value, 60);
		near(plate.finish.currentShape().volume(), plateVolume(80, 50));
		check(plate.document.redo(), "plate grouped redo");
		plate.document.recompute();
		near(plate.finish.currentShape().volume(), plateVolume(100, 60));
		var oldShape = plate.finish.currentShape();
		failed = false;
		try {
			plate.resize(-1, 60, 70, 40);
		} catch (error:Dynamic) {
			failed = true;
		}
		check(failed && plate.outline.width.value == 100 && oldShape == plate.finish.currentShape(), "failed edit rolls back parameters");
		var restored = DocumentCodec.decode(DocumentCodec.encode(plate.document));
		var restoredOutline:SketchFeature = cast restored.featureAt(0);
		var restoredGrid:GridFeature = cast restored.featureAt(2);
		var restoredFinish:FilletFeature = cast restored.featureAt(5);
		check(restoredFinish.selection != null && restoredFinish.selection.expectedCount == 4, "selection recipe persistence");
		transaction = restored.beginTransaction();
		restoredOutline.width.set(120);
		restoredGrid.spacingX.set(90);
		transaction.commit();
		restored.recompute();
		near(restoredFinish.currentShape().volume(), plateVolume(120, 60));
		check(restored.undo(), "loaded edit undo");
		restored.recompute();
		near(restoredFinish.currentShape().volume(), plateVolume(100, 60));
		check(restored.redo(), "loaded edit redo");
		restored.recompute();
		near(restoredFinish.currentShape().volume(), plateVolume(120, 60));
		restored.close();
		plate.close();
		// Serialized rules and dependencies must reject malformed input.
		failed = false;
		try {
			var bad = DocumentCodec.decode(StringTools.replace(serialized, '"position":"max"', '"position":"unknown"'));
			bad.close();
		} catch (error:Dynamic) {
			failed = true;
		}
		check(failed, "invalid serialized selector");
		failed = false;
		try {
			var bad = DocumentCodec.decode(StringTools.replace(serialized, '"sections":[{"id":2},{"id":3}]', '"sections":[{"id":2},{"id":999}]'));
			bad.close();
		} catch (error:Dynamic) {
			failed = true;
		}
		check(failed, "missing loft dependency");
	}
}
