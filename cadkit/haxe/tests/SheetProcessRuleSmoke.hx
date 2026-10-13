import CadKit;
import cadkit.Shape;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.RecomputeError;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sheetmetal.BendDimensions;
import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.FlatPattern;
import cadkit.sheetmetal.SheetBaseFeature;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetEdgeFlangeFeature;
import cadkit.sheetmetal.SheetEdgeSelection;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.process.CalibrationEntry;
import cadkit.sheetmetal.process.CalibrationTable;
import cadkit.sheetmetal.process.ProcessBendRule;
import cadkit.sheetmetal.process.ProcessTooling;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/** Production Process-rule feature rebuilding, calibrated development and current-format persistence. */
class SheetProcessRuleSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(actual:Float, expected:Float, label:String):Void {
		check(Math.isFinite(actual) && Math.abs(actual - expected) < 1e-8, label + ": " + actual + " != " + expected);
	}
	static function rectangle():ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		var points = [[0.0, 0.0], [60.0, 0.0], [60.0, 100.0], [0.0, 100.0]];
		for (i in 0...4) {
			var point = sketch.addPoint(new SketchPoint("p" + i, points[i][0], points[i][1]));
			var fixed = sketch.addConstraint(SketchConstraint.fixed("fixed" + i, "p" + i));
			var edge = sketch.addEntity(SketchEntity.line("e" + i, "p" + i, "p" + ((i + 1) % 4)));
		}
		return sketch;
	}
	static function pick(base:SheetBaseFeature):Shape {
		var graph = base.sheet(); var shape = base.currentShape();
		for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = shape.subshape(CadKit.ShapeKind.Face, i);
			try {
				var selected = SheetEdgeSelection.identify(graph, face);
				if (selected.wall == 1 && graph.wall(1).outline.identity(selected.edge.loop, selected.edge.segment) == "sketch:e1") return face;
			} catch (_:SheetDiagnosis) {}
			face.close();
		}
		throw "Missing process-rule test edge";
	}
	public static function run():Int {
		var tools = new ProcessTooling("acute", "v12", 0.6, Math.PI / 6, 12, Math.PI / 6, 0.5);
		var table = new CalibrationTable([new CalibrationEntry("lot-a", 1.5, "acute", "v12", 1, 2, 0.125, 2)]);
		var setup = new ProcessBendRule(tools, table, "lot-a");
		var stock = new SheetMaterial("process-steel", 1.5, BendRule.Process(setup), {uts: 400, yield: 250, youngs: 205000});
		near(stock.defaultRadius(Math.PI / 2), 2, "calibrated tooling determines default radius");
		near(BendDimensions.allowance(stock, 2, Math.PI / 2, 100), Math.PI / 2 * 2.75, "process neutral length");
		near(BendDimensions.allowance(stock, 2, -Math.PI / 2, 100), Math.PI / 2 * 2.75, "down process neutral length");
		var mismatch = false;
		try { var allowance = BendDimensions.allowance(stock, 1, Math.PI / 2, 100); }
		catch (failure:SheetDiagnosis) { mismatch = failure.code == "sheet.process-radius-mismatch"; }
		check(mismatch, "explicit incompatible radius is diagnosed");
		var document = new Document();
		try {
			var source = document.add(new ConstrainedSketchFeature(rectangle()));
			var base = document.add(new SheetBaseFeature(source, stock));
			document.recompute();
			var face = pick(base);
			var flange:SheetEdgeFlangeFeature;
			try { flange = document.add(new SheetEdgeFlangeFeature(base, [face], 30, Math.PI / 2)); }
			catch (error:Dynamic) { face.close(); throw error; }
			face.close(); document.setOutput(flange); document.recompute();
			var graph = flange.sheet(); var bend = graph.bends()[0];
			near(bend.radius, 2, "feature radius derives from process on rebuild");
			near(FlatPattern.of(graph).bends()[0].allowance, Math.PI / 2 * 2.75, "flat pattern uses process allowance");
			var encoded = DocumentCodec.encode(document);
			var loaded = DocumentCodec.decode(encoded);
			try {
				var restored = Std.downcast(loaded.featureById(flange.id.toInt()), SheetEdgeFlangeFeature);
				if (restored == null) throw "Wrong restored process feature type";
				check(restored.useMaterialRadius, "saved radius remains derived");
				near(restored.sheet().bends()[0].radius, 2, "process calibration survives document round trip");
				near(FlatPattern.of(restored.sheet()).bends()[0].allowance, Math.PI / 2 * 2.75, "saved process develops the same blank");
			} catch (error:Dynamic) { loaded.close(); throw error; }
			loaded.close();
			base.thickness.set(2); document.recompute();
			near(flange.sheet().bends()[0].radius, 0.16 * 12 * 400 / 410, "changed stock no longer matches the measured thickness");
			var oldShape = flange.currentShape(); var failed = false;
			flange.angle.set(2.8);
			try document.recompute() catch (_:RecomputeError) failed = true;
			check(failed && flange.currentShape().sameAs(oldShape), "unsupported process edit retains committed solid");
			check(document.undo(), "undo unsupported angle"); document.recompute();
			check(document.undo(), "undo stock change"); document.recompute();
			near(flange.sheet().bends()[0].radius, 2, "undo restores calibrated graph");
		} catch (error:Dynamic) { document.close(); throw error; }
		document.close();
		return checks;
	}
	static function main():Void { trace("Sheet process rule smoke passed (" + run() + " assertions)"); }
}
