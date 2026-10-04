import CadKit;
import cadkit.Shape;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.RecomputeError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sheetmetal.SheetBaseFeature;
import cadkit.sheetmetal.SheetCutFeature;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetSide;
import cadkit.sheetmetal.SheetWallCut;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.Point2;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchConstraint;

/** Through-wall cuts follow staged solved geometry and persistent support faces on either sheet side. */
class SheetCutSmoke {
	static var assertions:Int = 0;
	static function check(value:Bool, message:String):Void { assertions++; if (!value) throw message; }
	static function near(actual:Float, expected:Float, message:String):Void { check(Math.abs(actual - expected) < 1e-6, message); }
	static function rectangle(plane:Plane):ConstrainedSketch {
		var result = new ConstrainedSketch(plane); var points = [[0.0, 0.0], [20.0, 0.0], [20.0, 10.0], [0.0, 10.0]];
		for (i in 0...4) {
			var point = result.addPoint(new SketchPoint("p" + i, points[i][0], points[i][1]));
			var fixed = result.addConstraint(SketchConstraint.fixed("fixed" + i, "p" + i));
			var edge = result.addEntity(SketchEntity.line("edge" + i, "p" + i, "p" + ((i + 1) % 4)));
		}
		return result;
	}
	static function circle():ConstrainedSketch {
		var result = new ConstrainedSketch(); var center = result.addPoint(new SketchPoint("center", 0, 0));
		var fixed = result.addConstraint(SketchConstraint.fixed("fixed-center", "center"));
		var curve = result.addEntity(SketchEntity.circle("hole", "center", 1));
		var radius = result.addConstraint(SketchConstraint.radius("hole-radius", "hole", 1)); return result;
	}
	static function pick(base:SheetBaseFeature, side:SheetSide):Shape {
		var normal = base.sheet().evaluate(base.sheet().folded()).wall(1).plane.normal; var shape = base.currentShape();
		for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = shape.subshape(CadKit.ShapeKind.Face, i);
			if (face.surfaceKind() == CadKit.SurfaceKind.Plane) {
				var sign = Vector.fromNative(face.faceNormal()).dot(normal);
				if ((side == A && sign > 0.999) || (side == B && sign < -0.999)) return face;
			}
			face.close();
		}
		throw "missing test wall surface";
	}
	static function names(cut:SheetCutFeature):String { var result = cut.currentShape().elementNames(CadKit.ShapeKind.Face); result.sort(Reflect.compare); return result.join("|"); }
	public static function run():Int {
		assertions = 0;
		var plane = new Plane(new Vector(7, 8, 9), Vector.Y(), Vector.X());
		for (side in [SheetSide.A, SheetSide.B]) {
			var document = new Document(); var profile = document.add(new ConstrainedSketchFeature(rectangle(plane)));
			var base = document.add(new SheetBaseFeature(profile, SheetDemoParts.materialA())); document.recompute();
			var face = pick(base, side);
			var hole = document.add(new ConstrainedSketchFeature(circle(), base, null, plane.xDirection, 0, false, face)); face.close();
			var cut = document.add(new SheetCutFeature(base, hole, 1, side)); document.setOutput(cut); document.recompute();
			near(cut.sheet().wall(1).outline.area(), 200 - Math.PI, "exact attached circular cut area");
			near(cut.currentShape().volume(), (200 - Math.PI) * 1.5, "cut maps onto the rotated wall");
			check(CadKit.shapeValidChecked(cut.currentShape().borrowHandle()) != 0, "cut native solid is valid");
			var originalNames = names(cut); var loaded = DocumentCodec.decode(DocumentCodec.encode(document));
			var restored = Std.downcast(loaded.featureById(cut.id.toInt()), SheetCutFeature); if (restored == null) throw "unexpected saved cut type";
			check(restored.side == side && restored.wall == 1, "cut wall and side round trip"); check(names(restored) == originalNames, "cut face names round trip");
			hole.dimension("hole-radius").set(2); document.recompute(); near(cut.sheet().wall(1).outline.area(), 200 - 4 * Math.PI, "cut sees the staged sketch radius");
			check(names(cut) == originalNames, "cut radius edit retains authored boundary names");
			check(document.undo(), "undo cut sketch edit"); document.recompute(); near(cut.sheet().wall(1).outline.area(), 200 - Math.PI, "undo restores exact cut");
			check(document.redo(), "redo cut sketch edit"); document.recompute();
			base.thickness.set(3); document.recompute(); near(cut.currentShape().volume(), (200 - 4 * Math.PI) * 3, "attached side follows staged stock thickness");
			check(names(cut) == originalNames, "cut names survive thickness edits");
			var volume = cut.currentShape().volume(); hole.dimension("hole-radius").set(100); var refused = false;
			try { document.recompute(); } catch (error:Dynamic) { var failure = Std.downcast(error, RecomputeError); if (failure != null) {
				var diagnosis = Std.downcast(failure.cause, SheetDiagnosis); if (diagnosis != null) refused = diagnosis.code == "sheet.cut-empty";
			} }
			check(refused, "cut consuming the wall is diagnosed"); near(cut.currentShape().volume(), volume, "failed cut retains committed solid");
			near(cut.sheet().wall(1).outline.area(), 200 - 4 * Math.PI, "failed cut retains committed graph");
			loaded.close(); document.close();
		}
		var builder = new DocumentBuilder(); var profile = builder.constrainedSketch(rectangle(Plane.XY()), new Map());
		var base = builder.sheetBase(profile, SheetDemoParts.materialA()); base.document.recompute();
		var hole = builder.attachedConstrainedSketch(circle(), base, new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z()), Vector.X(), new Map());
		var output = builder.sheetCut(base, hole, 1); builder.output(output); var built = builder.finish();
		near(output.sheet().wall(1).outline.area(), 200 - Math.PI, "builder creates exact attached cut"); built.close();
		var part = SheetDemoParts.p1(); var edge = part.wall(1).outline.segment(0, 1); var x = edge.start.x;
		var tool = WallOutline.polygon([new Point2(x - 1, 40), new Point2(x + 1, 40), new Point2(x + 1, 42), new Point2(x - 1, 42)]);
		var crosses = false; try { var unused = SheetWallCut.apply(part, 1, tool, "bend-cut"); }
		catch (error:Dynamic) { var diagnosis = Std.downcast(error, SheetDiagnosis); if (diagnosis != null) crosses = diagnosis.code == "sheet.cut-bend-zone"; }
		check(crosses, "wall-only builder diagnoses bend-zone crossing instead of leaving uncut bend material");
		var mismatch = new Document(); var mismatchedBaseProfile = mismatch.add(new ConstrainedSketchFeature(rectangle(Plane.XY())));
		var mismatchedBase = mismatch.add(new SheetBaseFeature(mismatchedBaseProfile, SheetDemoParts.materialA())); mismatch.recompute();
		var shifted = circle(); var shiftedPlane = new ConstrainedSketch(Plane.XY().offset(1));
		for (point in shifted.points()) { var added = shiftedPlane.addPoint(point); }
		for (edge in shifted.entities()) { var added = shiftedPlane.addEntity(edge); }
		for (constraint in shifted.constraints()) { var added = shiftedPlane.addConstraint(constraint); }
		var shiftedProfile = mismatch.add(new ConstrainedSketchFeature(shiftedPlane)); var wrong = mismatch.add(new SheetCutFeature(mismatchedBase, shiftedProfile, 1));
		var planeRefused = false; try { mismatch.recompute(); } catch (error:Dynamic) {
			var failure = Std.downcast(error, RecomputeError); if (failure != null) { var diagnosis = Std.downcast(failure.cause, SheetDiagnosis); if (diagnosis != null) planeRefused = diagnosis.code == "sheet.cut-plane"; }
		}
		check(planeRefused, "cut sketch off the authored wall side is diagnosed"); mismatch.close();
		return assertions;
	}
	static function main():Void { trace("Sheet cut smoke passed (" + run() + " assertions)"); }
}
