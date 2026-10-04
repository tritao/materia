import CadKit;
import StringTools;
import cadkit.Shape;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.RecomputeError;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Parameter;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sheetmetal.FlangePosition;
import cadkit.sheetmetal.LengthSpec;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetBaseFeature;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetEdgeFlangeFeature;
import cadkit.sheetmetal.SheetEdgeSelection;
import cadkit.sheetmetal.SheetFeature;
import cadkit.sheetmetal.SheetFlange;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.WallOutline;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/** Production flange chains, saved picks, stock defaults and atomic edits through the document API. */
class SheetFlangeSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(actual:Float, expected:Float, label:String):Void {
		check(Math.isFinite(actual) && Math.abs(actual - expected) < 1e-6, label + ": " + actual + " != " + expected);
	}
	static function profile(points:Array<Point2>, rename:Bool = false):ConstrainedSketch {
		var result = new ConstrainedSketch();
		for (i in 0...points.length) {
			var point = result.addPoint(new SketchPoint("p" + i, points[i].x, points[i].y));
			var fixed = result.addConstraint(SketchConstraint.fixed("fixed" + i, "p" + i));
			var edge = result.addEntity(SketchEntity.line(rename && i == 1 ? "renamed" : "e" + i,
				"p" + i, "p" + ((i + 1) % points.length)));
		}
		return result;
	}
	static function rectangle(width:Float, height:Float, rename:Bool = false):ConstrainedSketch {
		return profile([new Point2(0, 0), new Point2(width, 0), new Point2(width, height), new Point2(0, height)], rename);
	}
	static function pick(feature:SheetFeature, wall:Int, identity:String):Shape {
		var graph = feature.sheet();
		var shape = feature.currentShape();
		for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = shape.subshape(CadKit.ShapeKind.Face, i);
			try {
				var edge = SheetEdgeSelection.identify(graph, face);
				if (edge.wall == wall && graph.wall(wall).outline.identity(edge.edge.loop, edge.edge.segment) == identity) return face;
			} catch (error:Dynamic) { if (!Std.isOfType(error, SheetDiagnosis)) { face.close(); throw error; } }
			face.close();
		}
		throw "missing authored test side " + identity;
	}
	static function flange(document:Document, source:SheetFeature, faces:Array<Shape>, length:Float, angle:Float,
		position:FlangePosition = MaterialInside):SheetEdgeFlangeFeature {
		try {
			var result = document.add(new SheetEdgeFlangeFeature(source, faces, length, angle, null, LengthSpec.OuterSharp, position));
			for (face in faces) face.close();
			document.setOutput(result); document.recompute();
			return result;
		} catch (error:Dynamic) { for (face in faces) face.close(); throw error; }
	}
	static function names(feature:SheetFeature):String {
		var result = feature.currentShape().elementNames(CadKit.ShapeKind.Face); result.sort(Reflect.compare); return result.join("\n");
	}
	static function compare(actual:Sheet, reference:Sheet):Void {
		check(actual.walls().length == reference.walls().length && actual.bends().length == reference.bends().length, "document graph counts match the reference");
		var a = actual.evaluate(actual.folded()).bounds(); var b = reference.evaluate(reference.folded()).bounds();
		near(a.maxX - a.minX, b.maxX - b.minX, "document reference width");
		near(a.maxY - a.minY, b.maxY - b.minY, "document reference length");
		near(a.maxZ - a.minZ, b.maxZ - b.minZ, "document reference height");
		for (wall in actual.walls()) near(wall.outline.area(), reference.wall(wall.id).outline.area(), "document reference wall area");
		for (bend in actual.bends()) { near(bend.angle, reference.bend(bend.id).angle, "document bend angle"); near(bend.radius, reference.bend(bend.id).radius, "document bend radius"); }
	}
	public static function run():Int {
		var references = [SheetDemoParts.p1(), SheetDemoParts.p2(), SheetDemoParts.p3(), SheetDemoParts.p4(), SheetDemoParts.p5()];
		for (part in 0...5) {
			var document = new Document();
			var sketch:ConstrainedSketch;
			if (part == 4) {
				var loop = references[4].wall(1).outline.loops()[0];
				sketch = profile([for (segment in loop) segment.start]);
			} else sketch = rectangle(60, part == 0 ? 100 : 120);
			var source = document.add(new ConstrainedSketchFeature(sketch));
			var base = document.add(new SheetBaseFeature(source, SheetDemoParts.materialA()));
			document.recompute();
			var output:SheetEdgeFlangeFeature;
			if (part == 0) output = flange(document, base, [pick(base, 1, "sketch:e1")], 40, Math.PI / 2);
			else if (part == 2) {
				var first = flange(document, base, [pick(base, 1, "sketch:e1")], 40, Math.PI / 2);
				output = flange(document, first, [pick(first, 1, "sketch:e3")], 40, -Math.PI / 2);
			} else if (part == 4) {
				output = flange(document, base, [for (edge in [0, 3, 6, 9]) pick(base, 1, "sketch:e" + edge)], 25,
					Math.PI / 2, FlangePosition.BendOutside);
			} else {
				var first = flange(document, base, [pick(base, 1, "sketch:e1"), pick(base, 1, "sketch:e3")], 40, Math.PI / 2);
				if (part == 3) output = flange(document, first, [pick(first, 2, "flange:tip"), pick(first, 3, "flange:tip")], 20, -Math.PI / 2);
				else output = first;
			}
			// Grouped hat construction allocates both sides before the lips; reference construction interleaves them.
			if (part != 3) compare(output.sheet(), references[part]);
			else {
				var a = output.sheet().evaluate(output.sheet().folded()).bounds(); var b = references[part].evaluate(references[part].folded()).bounds();
				near(a.maxX - a.minX, b.maxX - b.minX, "hat outside width"); near(a.maxZ - a.minZ, b.maxZ - b.minZ, "hat height");
			}
			check(CadKit.shapeValidChecked(output.currentShape().borrowHandle()) != 0, "document P1–P5 solid valid");
			var encoded = DocumentCodec.encode(document);
			if (part == 0) {
				var malformed = StringTools.replace(encoded, '"8:gap1",0', '"8:gap1","not-a-length"');
				check(malformed != encoded, "crafted invalid typed length reaches the flange payload");
				var rejected = false;
				try { var invalid = DocumentCodec.decode(malformed); invalid.close(); } catch (error:Dynamic) { rejected = true; }
				check(rejected, "typed wire validation refuses a string in a numeric flange field");
				var previous = StringTools.replace(encoded, '"version":' + DocumentCodec.VERSION, '"version":11');
				var oldRejected = false;
				try { var invalid = DocumentCodec.decode(previous); invalid.close(); } catch (error:Dynamic) { oldRejected = true; }
				check(oldRejected, "document format has one supported version and no older sheet reader");
			}
			var loaded = DocumentCodec.decode(encoded);
			var decoded = Std.downcast(loaded.featureById(output.id.toInt()), SheetEdgeFlangeFeature); if (decoded == null) throw "unexpected loaded feature type";
			check(names(decoded) == names(output), "flange codec preserves all native names");
			compare(decoded.sheet(), output.sheet());
			if (part == 0) {
				var originalNames = names(output);
				output.length.set(45); document.recompute();
				near(output.sheet().evaluate(output.sheet().folded()).bounds().maxZ + 1.5, 45, "flange length edit");
				check(names(output) == originalNames, "length edit preserves face names");
				check(document.undo(), "undo flange edit"); document.recompute(); compare(output.sheet(), references[0]);
				check(document.redo(), "redo flange edit"); document.recompute(); near(output.length.value, 45, "redo flange length");
				source.replaceSketch(rectangle(60, 100, true)); document.recompute();
				check(output.faceReferences()[0].isResolved(), "production flange follows a renamed surviving edge");
				base.thickness.set(3); document.recompute(); near(output.sheet().bend(1).radius, 2, "default radius derives from current material thickness");
				var before = output.currentShape().volume();
				source.replaceSketch(profile([new Point2(0, 0), new Point2(60, 0), new Point2(0, 100)]));
				var failed = false;
				try { document.recompute(); } catch (error:Dynamic) { failed = Std.isOfType(error, RecomputeError); }
				check(failed && output.faceReferences()[0].state == ReferenceState.Unresolved, "removed production flange edge fails unresolved");
				near(output.currentShape().volume(), before, "failed flange chain retains the committed solid");
				near(output.sheet().bend(1).radius, 2, "failed flange chain retains the committed graph");
			}
			loaded.close(); document.close();
		}
		var plain = new Sheet(SheetDemoParts.materialA()); var parent = plain.addWall(WallOutline.rectangle(60, 100));
		for (ends in [[0.0, 0.0, 0.0, 0.0], [3.0, 4.0, 0.0, 0.0], [3.0, 4.0, 2.0, 5.0], [0.0, 0.0, 2.0, 5.0]]) {
			var graph = SheetFlange.add(plain, 1, new WallEdge(1), 40, Math.PI / 2, 1, LengthSpec.Leg,
				FlangePosition.BendOutside, 0, ends[0], ends[1], ends[2], ends[3]);
			near(graph.wall(2).outline.area(), 40 * (100 - ends[0] - ends[1] + ends[2] + ends[3]), "gap and extension wall area");
			var bend = graph.bend(1);
			near(graph.wall(1).outline.segment(bend.edgeA.loop, bend.edgeA.segment).length(), 100 - ends[0] - ends[1], "end gaps own bend width");
			var shape = SheetSolid.build(graph, graph.folded());
			check(CadKit.shapeValidChecked(shape.borrowHandle()) != 0, "gap and extension folded solid valid"); shape.close();
			var flat = SheetSolid.build(graph, graph.flat());
			check(CadKit.shapeValidChecked(flat.borrowHandle()) != 0, "gap and extension flat solid valid"); flat.close();
		}
		var builder = new DocumentBuilder();
		var length = builder.length("flange.length", 40); var angle = builder.angle("flange.angle", Math.PI / 2);
		var radius = builder.length("flange.radius", 1);
		var sketch = builder.constrainedSketch(rectangle(60, 100), new Map());
		var base = builder.sheetBase(sketch, SheetDemoParts.materialA()); base.document.recompute();
		var face = pick(base, 1, "sketch:e1");
		var output = builder.sheetEdgeFlange(base, [face], length, angle, radius, LengthSpec.OuterSharp, FlangePosition.MaterialInside);
		face.close(); builder.output(output); var built = builder.finish();
		var saved = DocumentCodec.decode(DocumentCodec.encode(built));
		saved.parameter("flange.radius").set(1.3); saved.recompute();
		var restored = Std.downcast(saved.featureById(output.id.toInt()), SheetEdgeFlangeFeature); if (restored == null) throw "unexpected loaded feature type";
		near(restored.sheet().bend(1).radius, 1.3, "explicit named radius binding round trip");
		saved.close(); built.close();
		var changingDocument = new Document();
		var changingSource = changingDocument.add(new SheetVariableTopologySource()); changingDocument.recompute();
		var changingOutput = flange(changingDocument, changingSource, [pick(changingSource, 1, "0:1")], 40, Math.PI / 2);
		var originalWall = changingOutput.wallIds()[0]; var originalBend = changingOutput.bendIds()[0];
		var originalTip = pick(changingOutput, originalWall, "flange:tip");
		var originalTipName = originalTip.elementName(CadKit.ShapeKind.Face, 0); originalTip.close();
		changingSource.enableExtraWall(); changingDocument.recompute();
		check(changingOutput.wallIds()[0] == originalWall && changingOutput.bendIds()[0] == originalBend,
			"upstream topology growth cannot renumber a feature's authored output identities");
		check(changingOutput.sheet().walls().length == 3 && changingOutput.sheet().wall(originalWall).id == originalWall,
			"upstream and downstream authored identities remain distinct");
		var currentTip = pick(changingOutput, originalWall, "flange:tip");
		check(currentTip.elementName(CadKit.ShapeKind.Face, 0) == originalTipName, "downstream face name survives upstream output-count changes");
		currentTip.close(); changingDocument.close();
		return checks;
	}
}

/** Test-only authored topology change: its new identities are reserved after the downstream flange already exists. */
private class SheetVariableTopologySource extends SheetFeature {
	final extra:Parameter;
	var extraWall:Int = 0;
	var extraBend:Int = 0;
	public function new() {
		super();
		extra = new Parameter(this, "extra", 0, -1, true, 1);
	}
	public function enableExtraWall():Void {
		if (extraWall == 0) {
			extraWall = identityLedger().reserveWalls("upstream:" + id.toInt(), 1)[0];
			extraBend = identityLedger().reserveBends("upstream:" + id.toInt(), 1)[0];
		}
		extra.set(1);
	}
	override public function buildSheet(context:EvaluationContext):Sheet {
		var graph = new Sheet(SheetDemoParts.materialA());
		var root = graph.addWall(WallOutline.rectangle(60, 100));
		if (extra.value == 1)
			graph = SheetFlange.add(graph, root.id, new WallEdge(3), 40, Math.PI / 2, 1, LengthSpec.Leg,
				FlangePosition.BendOutside, 0, 0, 0, 0, 0, extraWall, extraBend);
		return graph;
	}
}
