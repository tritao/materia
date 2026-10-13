import cadkit.parametric.Document;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetFeature;
import cadkit.sheetmetal.WallOutline;

/** Graph publication must share the document's atomic shape and undo semantics. */
class SheetFeatureStagingSmoke {
	static var checks:Int = 0;
	static function check(condition:Bool, message:String):Void { checks++; if (!condition) throw message; }
	static function near(actual:Float, expected:Float, message:String):Void {
		check(Math.abs(actual - expected) < 1e-8, message + ": " + actual + " != " + expected);
	}
	public static function run():Int {
		var document = new Document();
		var source = document.add(new SheetStagingSource());
		var consumer = document.add(new SheetStagingConsumer(source));
		document.recompute();
		near(source.sheet().wall(2).outline.area(), 3750, "initial source graph");
		near(consumer.sheet().wall(2).outline.area(), 3750, "initial downstream graph");
		var external = source.sheet();
		var externalEdit = external.updateWall(2, WallOutline.rectangle(99, 100));
		near(source.sheet().wall(2).outline.area(), 3750, "committed graph cannot be mutated by a caller");
		var extra = external.addWall(WallOutline.rectangle(1, 1));
		near(extra.id, 3, "graph copy preserves next wall id");
		near(source.sheet().walls().length, 2, "copy allocation does not alter source graph");
		var transaction = document.beginTransaction();
		source.length.set(42);
		document.recompute();
		document.commitTransaction(transaction);
		near(consumer.sheet().wall(2).outline.area(), 4200, "consumer sees source staged graph");
		check(document.undo(), "undo graph parameter edit");
		document.recompute();
		near(consumer.sheet().wall(2).outline.area(), 3750, "undo graph recompute");
		check(document.redo(), "redo graph parameter edit");
		document.recompute();
		near(consumer.sheet().wall(2).outline.area(), 4200, "redo graph recompute");
		var sourceVolume = source.currentShape().volume();
		var consumerVolume = consumer.currentShape().volume();
		source.length.set(55);
		consumer.limit.set(5000);
		var failed = false;
		try { document.recompute(); } catch (_:Dynamic) { failed = true; }
		check(failed, "downstream failure rejects the staged recompute");
		near(source.sheet().wall(2).outline.area(), 4200, "failed recompute keeps source graph");
		near(consumer.sheet().wall(2).outline.area(), 4200, "failed recompute keeps downstream graph");
		near(source.currentShape().volume(), sourceVolume, "failed recompute keeps source solid");
		near(consumer.currentShape().volume(), consumerVolume, "failed recompute keeps downstream solid");
		consumer.limit.set(10000);
		document.recompute();
		near(consumer.sheet().wall(2).outline.area(), 5500, "recovery builds fresh staged graph");
		document.close();
		return checks;
	}
}

/** Test-only graph builder exercises publication without introducing a saved feature type. */
private class SheetStagingSource extends SheetFeature {
	public final length:Parameter;
	public function new() { super(); length = new Parameter(this, "test.length", 37.5, 0); }
	override public function buildSheet(context:EvaluationContext):Sheet {
		var graph = SheetDemoParts.p1();
		var wall = graph.updateWall(2, WallOutline.rectangle(length.value, 100));
		return graph;
	}
}

private class SheetStagingConsumer extends SheetFeature {
	final source:SheetFeature;
	public final limit:Parameter;
	public function new(source:SheetFeature) {
		super(); this.source = source; limit = new Parameter(this, "test.limit", 10000, 0);
	}
	override public function dependencies():Array<FeatureId> { return [source.id]; }
	override public function dependencyFeatures():Array<Feature> { return [source]; }
	override public function buildSheet(context:EvaluationContext):Sheet {
		var graph = source.sheetForEvaluation(context);
		if (graph.wall(2).outline.area() > limit.value)
			throw new SheetDiagnosis("sheet.test-limit", "Injected downstream failure");
		return graph;
	}
}
