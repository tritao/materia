import CadKit;
import cadkit.Shape;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.RecomputeError;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sheetmetal.SheetBaseFeature;
import cadkit.sheetmetal.SheetEdgeFlangeFeature;
import cadkit.sheetmetal.SheetEdgeSelection;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.LengthSpec;
import cadkit.sheetmetal.FlangePosition;
import cadkit.sheetmetal.ReliefSizing;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchConstraint;
import cadkit.sheetmetal.ReliefGeometry;
import cadkit.sheetmetal.ReliefType;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.SheetDiagnosis;

/** Exact bend-end slots preserve straight attachments and explicit cap radii. */
class SheetReliefSmoke {
	static var assertions:Int = 0;
	static function check(value:Bool, message:String):Void { assertions++; if (!value) throw message; }
	static function near(actual:Float, expected:Float, message:String):Void { check(Math.abs(actual - expected) < 1e-7, message); }
	static function sketch():ConstrainedSketch {
		var result = new ConstrainedSketch(); var points = [[0.0, 0.0], [20.0, 0.0], [20.0, 10.0], [0.0, 10.0]];
		for (i in 0...4) {
			var point = result.addPoint(new SketchPoint("p" + i, points[i][0], points[i][1]));
			var fixed = result.addConstraint(SketchConstraint.fixed("fixed" + i, "p" + i));
			var line = result.addEntity(SketchEntity.line("edge" + i, "p" + i, "p" + ((i + 1) % 4)));
		}
		return result;
	}
	static function pick(base:SheetBaseFeature):Shape {
		var graph = base.sheet(); var shape = base.currentShape();
		for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = shape.subshape(CadKit.ShapeKind.Face, i);
			try {
				var edge = SheetEdgeSelection.identify(graph, face);
				if (graph.wall(edge.wall).outline.identity(edge.edge.loop, edge.edge.segment) == "sketch:edge0") return face;
			} catch (error:Dynamic) { if (!Std.isOfType(error, SheetDiagnosis)) { face.close(); throw error; } }
			face.close();
		}
		throw "missing relief test attachment";
	}
	static function documentCases():Void {
		for (type in [ReliefType.Rectangular, ReliefType.Obround]) for (sizing in [ReliefSizing.Absolute, ReliefSizing.ThicknessRatio]) {
			var document = new Document(); var source = document.add(new ConstrainedSketchFeature(sketch()));
			var base = document.add(new SheetBaseFeature(source, SheetDemoParts.materialA())); document.recompute();
			var face = pick(base);
			var flange = document.add(new SheetEdgeFlangeFeature(base, [face], 10, Math.PI / 2, null, Leg, BendOutside,
				0, 3, 4, 0, 0, null, null, null, type, sizing, sizing == Absolute ? 2 : 1, sizing == Absolute ? 3 : 2));
			face.close(); document.setOutput(flange); document.recompute();
			var graph = flange.sheet(); var width = sizing == Absolute ? 2.0 : 1.5;
			var removed = type == Rectangular ? 2 * width * 3 : 2 * width * (3 - width / 2) + Math.PI * width * width / 4;
			near(graph.wall(1).outline.area(), 200 - removed, "document derives exact parent relief area");
			near(graph.wall(flange.wallIds()[0]).outline.area(), 130, "reliefs preserve requested flange width through extensions");
			near(graph.evaluate(graph.folded()).bend(graph.bends()[0].id).width, 13 - 2 * width, "reliefs trim bend ends explicitly");
			check(CadKit.shapeValidChecked(flange.currentShape().borrowHandle()) != 0, "relieved folded solid is valid");
			var flat = SheetSolid.build(graph, graph.flat()); check(CadKit.shapeValidChecked(flat.borrowHandle()) != 0, "relieved flat solid is valid"); flat.close();
			var loaded = DocumentCodec.decode(DocumentCodec.encode(document));
			var restored = Std.downcast(loaded.featureById(flange.id.toInt()), SheetEdgeFlangeFeature);
			if (restored == null) throw "unexpected restored relief feature";
			check(restored.reliefType == type && restored.reliefSizing == sizing, "typed relief enums round trip");
			near(restored.reliefWidth.value, flange.reliefWidth.value, "authored relief dimension round trips");
			check(restored.currentShape().elementNames(CadKit.ShapeKind.Face).join("|") == flange.currentShape().elementNames(CadKit.ShapeKind.Face).join("|"), "relief native names round trip");
			var originalNames = flange.currentShape().elementNames(CadKit.ShapeKind.Face).join("|");
			if (sizing == ThicknessRatio) {
				base.thickness.set(3); document.recompute(); var changed = flange.sheet();
				near(changed.evaluate(changed.folded()).bend(changed.bends()[0].id).width, 7, "ratio relief width follows thickness");
				check(CadKit.shapeValidChecked(flange.currentShape().borrowHandle()) != 0, "thickness-derived relief rebuild remains valid");
				check(flange.currentShape().elementNames(CadKit.ShapeKind.Face).join("|") == originalNames, "relief names survive thickness-derived geometry edits");
			}
			var volume = flange.currentShape().volume(); var parentArea = flange.sheet().wall(1).outline.area();
			flange.reliefWidth.set(100); var refused = false;
			try { document.recompute(); } catch (error:Dynamic) {
				var failure = Std.downcast(error, RecomputeError);
				if (failure != null) { var diagnosis = Std.downcast(failure.cause, SheetDiagnosis); if (diagnosis != null) refused = diagnosis.code == "sheet.invalid-relief"; }
			}
			check(refused, "oversized relief has a specific diagnosis");
			near(flange.currentShape().volume(), volume, "failed relief edit retains the committed solid");
			near(flange.sheet().wall(1).outline.area(), parentArea, "failed relief edit retains the committed graph");
			check(document.undo(), "invalid relief edit can be undone"); document.recompute();
			loaded.close(); document.close();
		}
	}
	public static function run():Int {
		assertions = 0;
		var source = WallOutline.rectangle(20, 10); var boundary = source.segment(0, 0);
		var rectangle = ReliefGeometry.cut(source, boundary, 0, 0, Rectangular, 2, 3, "test");
		near(rectangle.area(), 188, "two rectangular reliefs remove width times depth");
		var rounded = ReliefGeometry.cut(source, boundary, 0, 0, Obround, 2, 3, "test");
		near(rounded.area(), 192 - Math.PI, "obround relief area retains exact semicircular caps");
		var arcs = 0;
		for (segment in rounded.loops()[0]) if (!segment.isLine()) { arcs++; near(segment.radius, 1, "relief cap radius is half the width"); }
		check(arcs == 2, "both round relief caps survive as arcs");
		var internal = ReliefGeometry.cut(source, boundary, 3, 4, Rectangular, 2, 3, "internal");
		near(internal.area(), 188, "internal relief slots retain material beyond the gaps");
		var invalid = false;
		try { var unused = ReliefGeometry.cut(source, boundary, 0, 0, Obround, 4, 1, "invalid"); }
		catch (error:Dynamic) { var diagnosis = Std.downcast(error, SheetDiagnosis); if (diagnosis != null) invalid = diagnosis.code == "sheet.invalid-relief"; }
		check(invalid, "round cap must fit the explicit depth");
		var builder = new DocumentBuilder(); var length = builder.length("relief.leg", 10); var angle = builder.angle("relief.angle", Math.PI / 2);
		var sourceFeature = builder.constrainedSketch(sketch(), new Map()); var base = builder.sheetBase(sourceFeature, SheetDemoParts.materialA()); base.document.recompute();
		var face = pick(base); var output = builder.sheetEdgeFlange(base, [face], length, angle, null, Leg, BendOutside,
			0, 3, 4, 0, 0, Obround, ThicknessRatio, 1, 2); face.close(); builder.output(output); var built = builder.finish();
		check(output.reliefType == Obround && output.reliefSizing == ThicknessRatio, "builder keeps typed relief choices");
		near(output.sheet().evaluate(output.sheet().folded()).bend(output.bendIds()[0]).width, 10, "builder derives attachment from ratio dimensions"); built.close();
		documentCases();
		return assertions;
	}
	static function main():Void { trace("Sheet relief smoke passed (" + run() + " assertions)"); }
}
