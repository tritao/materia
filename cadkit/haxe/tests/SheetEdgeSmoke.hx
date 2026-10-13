import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.TopologyReference;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sheetmetal.SheetBaseFeature;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetEdgeSelection;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.WallOutline;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/** Wall-side picks keep their authored identity through contour reordering, renames and deletion. */
class SheetEdgeSmoke {
	static var checks:Int = 0;
	static function check(condition:Bool, message:String):Void { checks++; if (!condition) throw message; }
	static function outline(order:Array<Int>, renamed:Bool = false, triangle:Bool = false):ConstrainedSketch {
		var result = new ConstrainedSketch();
		var points = [[0.0, 0.0], [10.0, 0.0], [10.0, 6.0], [0.0, 6.0]];
		for (i in 0...4) {
			var point = result.addPoint(new SketchPoint("p" + i, points[i][0], points[i][1]));
			var fixed = result.addConstraint(SketchConstraint.fixed("fix" + i, "p" + i));
		}
		for (i in order) {
			var edge = result.addEntity(SketchEntity.line(renamed && i == 1 ? "renamed" : "e" + i,
				"p" + i, "p" + ((i + 1) % 4)));
		}
		if (triangle) { var diagonal = result.addEntity(SketchEntity.line("diagonal", "p1", "p3")); }
		return result;
	}
	static function side(shape:Shape, x:Float, y:Float):Shape {
		for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = shape.subshape(CadKit.ShapeKind.Face, i);
			var center = Vector.fromNative(face.center());
			if (Math.abs(center.x - x) < 1e-8 && Math.abs(center.y - y) < 1e-8 && Math.abs(center.z + 0.75) < 1e-8)
				return face;
			face.close();
		}
		throw "missing test wall-side face";
	}
	public static function run():Int {
		var document = new Document();
		var source = document.add(new ConstrainedSketchFeature(outline([0, 1, 2, 3])));
		var base = document.add(new SheetBaseFeature(source, SheetDemoParts.materialA()));
		document.recompute();
		var picked = side(base.currentShape(), 10, 3);
		var selected = SheetEdgeSelection.identify(base.sheet(), picked);
		check(selected.wall == 1 && base.sheet().wall(1).outline.identity(selected.edge.loop, selected.edge.segment) == "sketch:e1",
			"a resolved face maps to its authored graph segment");
		var reference = TopologyReference.fromShape(base, picked, base);
		var oldName = picked.elementName(CadKit.ShapeKind.Face, 0);
		source.replaceSketch(outline([2, 3, 0, 1])); document.recompute();
		var reordered = reference.resolveFor(base.currentShape());
		check(reordered.elementName(CadKit.ShapeKind.Face, 0) == oldName, "contour reordering preserves the edge name");
		var center = Vector.fromNative(reordered.center());
		check(Math.abs(center.x - 10) < 1e-8 && Math.abs(center.y - 3) < 1e-8, "reordered reference stays on the original side");
		reordered.close();
		source.replaceSketch(outline([2, 3, 0, 1], true)); document.recompute();
		var renamed = reference.resolveFor(base.currentShape());
		var renamedEdge = SheetEdgeSelection.identify(base.sheet(), renamed);
		check(base.sheet().wall(1).outline.identity(renamedEdge.edge.loop, renamedEdge.edge.segment) == "sketch:renamed",
			"a renamed surviving edge resolves through the existing geometric policy");
		renamed.close();
		source.replaceSketch(outline([0, 3], false, true)); document.recompute();
		var unresolved = false;
		try { var missing = reference.resolveFor(base.currentShape()); missing.close(); }
		catch (error:Dynamic) { unresolved = reference.state == ReferenceState.Unresolved; }
		check(unresolved, "a removed edge never resolves to the segment that replaced its contour index");
		document.close();
		for (sheet in [SheetDemoParts.p1(), SheetDemoParts.p2(), SheetDemoParts.p3()]) {
			var shape = SheetSolid.build(sheet, sheet.folded());
			var sides = 0;
			for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
				var face = shape.subshape(CadKit.ShapeKind.Face, i);
				try {
					var edge = SheetEdgeSelection.identify(sheet, face);
					check(sheet.wall(edge.wall).outline.segment(edge.edge.loop, edge.edge.segment).isLine(), "folded side maps to a straight segment");
					sides++;
				} catch (error:Dynamic) {
					if (!Std.isOfType(error, SheetDiagnosis)) { face.close(); shape.close(); throw error; }
				}
				face.close();
			}
			check(sides > 0, "folded sheet exposes selectable straight sides");
			shape.close();
		}
		return checks;
	}
}
