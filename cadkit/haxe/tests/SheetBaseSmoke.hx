import CadKit;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.RecomputeError;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.KFactorTable;
import cadkit.sheetmetal.SheetBaseFeature;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.SheetSide;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchConstraint;

/** Production sheet-base creation, stock persistence, source edits and workplane semantics. */
class SheetBaseSmoke {
	static var checks:Int = 0;
	static function check(condition:Bool, message:String):Void { checks++; if (!condition) throw message; }
	static function near(actual:Float, expected:Float, message:String):Void {
		check(Math.isFinite(actual) && Math.abs(actual - expected) < 1e-5, message + ": " + actual + " != " + expected);
	}
	static function rectangle(plane:Plane, units:String = "mm", hole:Bool = false):ConstrainedSketch {
		var sketch = new ConstrainedSketch(plane, units);
		var points = [[0.0, 0.0], [10.0, 0.0], [10.0, 6.0], [0.0, 6.0]];
		for (i in 0...4) {
			var point = sketch.addPoint(new SketchPoint("p" + i, points[i][0], points[i][1]));
			var fixed = sketch.addConstraint(SketchConstraint.fixed("fixed" + i, "p" + i));
			var edge = sketch.addEntity(SketchEntity.line("e" + i, "p" + i, "p" + ((i + 1) % 4)));
		}
		if (hole) {
			var center = sketch.addPoint(new SketchPoint("center", 3, 3));
			var fixed = sketch.addConstraint(SketchConstraint.fixed("fixed-center", "center"));
			var circle = sketch.addEntity(SketchEntity.circle("hole", "center", 1));
			var radius = sketch.addConstraint(SketchConstraint.radius("hole-radius", "hole", 1));
		}
		return sketch;
	}
	public static function run():Int {
		var plane = new Plane(new Vector(7, 8, 9), Vector.Y(), Vector.X());
		for (side in [SheetSide.A, SheetSide.B]) {
			var document = new Document();
			var source = document.add(new ConstrainedSketchFeature(rectangle(plane, "cm", true)));
			var base = document.add(new SheetBaseFeature(source, SheetDemoParts.materialA(), side));
			document.recompute();
			var graph = base.sheet();
			near(graph.wall(1).outline.area(), 6000 - 100 * Math.PI, "canonical mm region with a hole");
			near(base.currentShape().volume(), (6000 - 100 * Math.PI) * 1.5, "sheet base volume");
			var origin = graph.evaluate(graph.folded()).wall(1).plane.origin;
			near(origin.x, side == SheetSide.A ? 7 : 8.5, "profile surface fixes root placement");
			near(origin.y, 8, "root y"); near(origin.z, 9, "root z");
			var bounds = base.currentShape().bounds();
			near(bounds.get_min().get_x(), side == SheetSide.A ? 5.5 : 7, "solid begins on the correct side of the profile");
			near(bounds.get_max().get_x(), side == SheetSide.A ? 7 : 8.5, "solid ends on the correct side of the profile");
			var encoded = DocumentCodec.encode(document);
			var loaded = DocumentCodec.decode(encoded);
			var decoded = Std.downcast(loaded.featureById(base.id.toInt()), SheetBaseFeature); if (decoded == null) throw "unexpected loaded feature type";
			check(decoded.side == side && decoded.materialId == "mild-steel", "base side and material identity round trip");
			check(decoded.currentShape().elementNames(CadKit.ShapeKind.Face).join("\n")
				== base.currentShape().elementNames(CadKit.ShapeKind.Face).join("\n"), "base names round trip");
			near(decoded.sheet().wall(1).outline.area(), graph.wall(1).outline.area(), "exact outline round trip");
			var mechanics = decoded.stock().mechanical; if (mechanics == null) throw "missing test mechanical data";
			near(mechanics.youngs, 205000, "material MPa round trip");
			base.thickness.set(2);
			document.recompute();
			near(base.sheet().thickness(), 2, "thickness parameter rebuilds stock");
			check(document.undo(), "undo base thickness"); document.recompute();
			near(base.sheet().thickness(), 1.5, "undo base graph thickness");
			check(document.redo(), "redo base thickness"); document.recompute();
			near(base.sheet().thickness(), 2, "redo base graph thickness");
			source.dimension("hole-radius").set(12);
			document.recompute();
			near(base.sheet().wall(1).outline.area(), 6000 - 144 * Math.PI, "staged source radius feeds exact sheet region");
			loaded.close(); document.close();
		}
		var builder = new DocumentBuilder();
		var thickness = builder.length("plate.thickness", 2);
		var source = builder.constrainedSketch(rectangle(Plane.XY()), new Map());
		var base = builder.sheetBase(source, new SheetMaterial("unspecified-strength", 1,
			BendRule.KFactor(new KFactorTable([0], [0.4])), null, null, null, Math.PI / 3), thickness);
		builder.output(base);
		var document = builder.finish();
		near(base.sheet().thickness(), 2, "builder binds named thickness");
		var grain = base.stock().grainAngle; if (grain == null) throw "builder lost grain";
		near(grain, Math.PI / 3, "named thickness preserves authored grain");
		var loaded = DocumentCodec.decode(DocumentCodec.encode(document));
		loaded.parameter("plate.thickness").set(3); loaded.recompute();
		var restored = Std.downcast(loaded.featureById(base.id.toInt()), SheetBaseFeature); if (restored == null) throw "unexpected loaded feature type";
		near(restored.sheet().thickness(), 3, "loaded named thickness binding");
		check(restored.stock().mechanical == null, "optional mechanical data stays absent");
		loaded.close(); document.close();
		var curved = new ConstrainedSketch();
		var center = curved.addPoint(new SketchPoint("center", 0, 0));
		var left = curved.addPoint(new SketchPoint("left", -10, 0));
		var right = curved.addPoint(new SketchPoint("right", 10, 0));
		var arc = curved.addEntity(SketchEntity.arc("arc", "center", 10, 0, Math.PI));
		var diameter = curved.addEntity(SketchEntity.line("diameter", "left", "right"));
		var curvedDocument = new Document();
		var curvedSource = curvedDocument.add(new ConstrainedSketchFeature(curved));
		var curvedBase = curvedDocument.add(new SheetBaseFeature(curvedSource, SheetDemoParts.materialA()));
		curvedDocument.recompute();
		near(curvedBase.sheet().wall(1).outline.area(), 50 * Math.PI, "partial arc base region");
		near(curvedBase.currentShape().volume(), 75 * Math.PI, "partial arc base solid");
		curvedDocument.close();
		var multiple = new ConstrainedSketch();
		var c1 = multiple.addPoint(new SketchPoint("c1", 0, 0));
		var c2 = multiple.addPoint(new SketchPoint("c2", 20, 0));
		var e1 = multiple.addEntity(SketchEntity.circle("circle1", "c1", 2));
		var e2 = multiple.addEntity(SketchEntity.circle("circle2", "c2", 2));
		var bad = new Document();
		var badSource = bad.add(new ConstrainedSketchFeature(multiple));
		var badBase = bad.add(new SheetBaseFeature(badSource, SheetDemoParts.materialA()));
		var diagnosed = false;
		try { bad.recompute(); } catch (error:Dynamic) {
			if (Std.isOfType(error, RecomputeError)) {
				var failure = Std.downcast(error, RecomputeError); if (failure == null) throw "unexpected error type";
				if (Std.isOfType(failure.cause, SheetDiagnosis)) {
					var diagnosis = Std.downcast(failure.cause, SheetDiagnosis); if (diagnosis == null) throw "unexpected error type";
					diagnosed = diagnosis.code == "sheet.base-region-count";
				}
			}
		}
		check(diagnosed, "multiple islands are refused with the exact diagnosis");
		bad.close();
		return checks;
	}
}
