import CadKit;
import cadkit.Shape;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.RecomputeError;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sheetmetal.CornerJoin;
import cadkit.sheetmetal.CornerRelief;
import cadkit.sheetmetal.FlatPattern;
import cadkit.sheetmetal.SheetCornerEdge;
import cadkit.sheetmetal.SheetCornerFeature;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetEdgeSelection;
import cadkit.sheetmetal.SheetFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/** Production corner chains preserve picks, shared dimensions, names and atomic save/edit behavior. */
class SheetCornerFeatureSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(actual:Float, expected:Float, label:String):Void {
		check(Math.isFinite(actual) && Math.abs(actual - expected) < 1e-8, label + ": " + actual + " != " + expected);
	}
	static function rectangle():ConstrainedSketch {
		var result = new ConstrainedSketch(); var points = [[0.0, 0.0], [80.0, 0.0], [80.0, 100.0], [0.0, 100.0]];
		for (i in 0...4) {
			var point = result.addPoint(new SketchPoint("p" + i, points[i][0], points[i][1]));
			var fixed = result.addConstraint(SketchConstraint.fixed("fixed" + i, "p" + i));
			var edge = result.addEntity(SketchEntity.line("e" + i, "p" + i, "p" + ((i + 1) % 4)));
		}
		return result;
	}
	static function pick(feature:SheetFeature, wall:Int, identity:String):Shape {
		var graph = feature.sheet(); var shape = feature.currentShape();
		for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = shape.subshape(CadKit.ShapeKind.Face, i);
			try {
				var selected = SheetEdgeSelection.identify(graph, face);
				if (selected.wall == wall && graph.wall(wall).outline.identity(selected.edge.loop, selected.edge.segment) == identity) return face;
			} catch (failure:SheetDiagnosis) { }
			face.close();
		}
		throw "Missing corner test side " + identity;
	}
	static function names(feature:SheetFeature):String {
		var result = feature.currentShape().elementNames(CadKit.ShapeKind.Face); result.sort(Reflect.compare); return result.join("\n");
	}
	public static function run():Int {
		checks = 0;
		for (join in [CornerJoin.Gap, CornerJoin.OverlapFirst, CornerJoin.OverlapSecond]) for (relief in [CornerRelief.Round, CornerRelief.Square]) {
			var builder = new DocumentBuilder(); var sketch = builder.constrainedSketch(rectangle(), new Map());
			var base = builder.sheetBase(sketch, SheetDemoParts.materialA()); base.document.recompute();
			var leg = builder.length("corner.leg", 25); var angle = builder.angle("corner.angle", Math.PI / 2);
			var radius = builder.length("corner.radius", 1);
			var firstFace = pick(base, 1, "sketch:e1");
			var first = builder.sheetEdgeFlange(base, [firstFace], leg, angle, radius); firstFace.close(); base.document.recompute();
			var secondFace = pick(first, 1, "sketch:e2");
			var second = builder.sheetEdgeFlange(first, [secondFace], leg, angle, radius); secondFace.close(); base.document.recompute();
			var sideA = pick(second, first.wallIds()[0], "flange:end"); var sideB = pick(second, second.wallIds()[0], "flange:start");
			var targetA = SheetCornerEdge.fromFace(second.sheet(), sideA); var targetB = SheetCornerEdge.fromFace(second.sheet(), sideB);
			sideA.close(); sideB.close();
			var gap = builder.length("corner.gap", 0.5); var size = builder.length("corner.relief", 8);
			var overlap = join == Gap ? null : builder.length("corner.overlap", 2);
			var corner = builder.sheetCorner(second, targetA, targetB, gap, relief, size, join, overlap);
			builder.output(corner); var document = builder.finish();
			try {
				check(corner.serializationType() == "sheet-corner", "corner is a production feature");
				var savedNames = names(corner); var originalArea = FlatPattern.of(corner.sheet()).area();
				var saved = DocumentCodec.encode(document); var loaded = DocumentCodec.decode(saved);
				try {
					var restored = Std.downcast(loaded.featureById(corner.id.toInt()), SheetCornerFeature);
					if (restored == null) throw "Wrong restored corner feature type";
					check(restored.join == join && restored.relief == relief, "wire enums retain corner arrangement and relief shape");
					check(restored.first.wall == targetA.wall && restored.first.identity == targetA.identity
						&& restored.second.wall == targetB.wall && restored.second.identity == targetB.identity, "saved corner picks retain authored identities");
					near(FlatPattern.of(restored.sheet()).area(), originalArea, "saved corner derives the same developed blank");
					check(names(restored) == savedNames, "saved corner derives the same native face names");
					check(DocumentCodec.encode(loaded) == saved, "current corner document round trip is byte identical");
				} catch (failure:Dynamic) { loaded.close(); throw failure; }
				loaded.close();
				gap.set(0.75); document.recompute();
				near(corner.gap.value, 0.75, "named gap binds to the corner parameter");
				check(names(corner) == savedNames, "gap edit retains native face identities");
				check(document.undo(), "gap edit is undoable"); document.recompute();
				near(FlatPattern.of(corner.sheet()).area(), originalArea, "undo restores exact corner development");
				check(document.redo(), "gap edit is redoable"); document.recompute();
				near(corner.gap.value, 0.75, "redo restores corner gap");
				leg.set(26); document.recompute();
				check(corner.sheet().walls().length == 3 && corner.sheet().bends().length == 2, "upstream flange edit retains corner graph identities");
				check(names(corner) == savedNames, "upstream flange edit retains corner native names");
				var published = corner.currentShape(); var beforeArea = FlatPattern.of(corner.sheet()).area();
				gap.set(-1); var refused = false;
				try { document.recompute(); }
				catch (failure:RecomputeError) {
					var diagnosis = Std.downcast(failure.cause, SheetDiagnosis);
					refused = diagnosis != null && diagnosis.code == "sheet.invalid-corner-dimension";
				}
				check(refused, "invalid gap names its failed corner dimension");
				check(corner.currentShape() == published, "failed corner edit retains the committed solid");
				near(FlatPattern.of(corner.sheet()).area(), beforeArea, "failed corner edit retains the committed graph");
				check(document.undo(), "failed corner edit remains undoable"); document.recompute();
			} catch (failure:Dynamic) { document.close(); throw failure; }
			document.close();
		}
		return checks;
	}
	static function main():Void { trace("Sheet corner feature smoke passed (" + run() + " assertions)"); }
}
