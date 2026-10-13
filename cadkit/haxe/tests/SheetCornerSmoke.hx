import CadKit;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.CornerJoin;
import cadkit.sheetmetal.CornerRelief;
import cadkit.sheetmetal.FlatPattern;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetContourEdit;
import cadkit.sheetmetal.SheetCorner;
import cadkit.sheetmetal.SheetCornerEdge;
import cadkit.sheetmetal.SheetCutGeometry;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetFlange;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.WallOutline;

/** Corner closure checks real slab positions, one blank, stable datums and atomic failures. */
class SheetCornerSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(actual:Float, expected:Float, label:String):Void {
		check(Math.isFinite(actual) && Math.abs(actual - expected) < 1e-8, label + ": " + actual + " != " + expected);
	}
	static function diagnosis(code:String, action:Void->Void):Void {
		checks++;
		try { action(); }
		catch (failure:SheetDiagnosis) { if (failure.code == code) return; throw failure; }
		throw "Expected " + code;
	}
	static function source():Sheet {
		var base = new Sheet(SheetDemoParts.materialA()); var wall = base.addWall(WallOutline.rectangle(80, 100));
		var first = SheetFlange.add(base, wall.id, new WallEdge(1), 25, Math.PI / 2, 1);
		return SheetFlange.add(first, wall.id, new WallEdge(2), 25, Math.PI / 2, 1);
	}
	public static function run():Int {
		checks = 0;
		var graph = source();
		var first = new SheetCornerEdge(2, "flange:end"); var second = new SheetCornerEdge(3, "flange:start");
		for (join in [CornerJoin.Gap, CornerJoin.OverlapFirst, CornerJoin.OverlapSecond]) {
			for (relief in [CornerRelief.Round, CornerRelief.Square]) {
				var closed = SheetCorner.apply(graph, first, second, join, 0.5, join == Gap ? 0 : 2, relief, 8, "closed-corner");
				var folded = closed.evaluate(closed.folded());
				var a = first.resolve(closed); var b = second.resolve(closed);
				var firstPoint = closed.wall(2).outline.segment(a.loop, a.segment).point(0.5);
				var secondPoint = closed.wall(3).outline.segment(b.loop, b.segment).point(0.5);
				// R=1, T=1.5: mate near face is base edge +1; far face is base edge +2.5.
				near(folded.wall(2).plane.toWorld(new Vector(firstPoint.x, firstPoint.y)).y,
					join == OverlapFirst ? 104.5 : 100.5, "first end clears or overlaps the real mate slab");
				near(folded.wall(3).plane.toWorld(new Vector(secondPoint.x, secondPoint.y)).x,
					join == OverlapSecond ? 84.5 : 80.5, "second end clears or overlaps the real mate slab");
				for (bend in closed.bends()) {
					near(bend.edgeA.span.length(), graph.bend(bend.id).edgeA.span.length(), "closure preserves nominal parent seam");
					near(bend.edgeB.span.length(), graph.bend(bend.id).edgeB.span.length(), "closure preserves nominal child seam");
				}
				var flat = FlatPattern.of(closed); var area = 0.0;
				for (domain in flat.domains()) area += domain.outline.area();
				near(flat.area(), area, "closed corner develops without overlapping material");
				check(flat.regions().length == 1, "closed corner remains one blank");
				var partial = closed.folded(); var changedAngle = partial.set(1, Math.PI / 4);
				for (state in [closed.folded(), closed.flat(), partial]) {
					var solid = SheetSolid.build(closed, state);
					try { check(CadKit.shapeValidChecked(solid.borrowHandle()) != 0, "closed corner has valid native geometry"); }
					catch (failure:Dynamic) { solid.close(); throw failure; }
					solid.close();
				}
			}
		}
		// A larger stand-off trims both ends; the matching larger relief repairs their seams atomically.
		var trimmed = SheetCorner.apply(graph, first, second, Gap, 4, 0, Round, 20, "trimmed-corner");
		check(trimmed.wall(2).outline.area() < graph.wall(2).outline.area(), "large gap trims the first flange");
		check(trimmed.wall(3).outline.area() < graph.wall(3).outline.area(), "large gap trims the second flange");
		var verified = trimmed.evaluate(trimmed.folded());
		diagnosis("sheet.corner-selection", function() { var invalid = SheetCorner.apply(graph, new SheetCornerEdge(2, "removed"), second, Gap, 0.5, 0, Round, 8, "invalid"); });
		diagnosis("sheet.corner-adjacency", function() { var invalid = SheetCorner.apply(graph, first, first, Gap, 0.5, 0, Round, 8, "invalid"); });
		diagnosis("sheet.cut-seam-mismatch", function() { var invalid = SheetCorner.apply(graph, first, second, Gap, 5, 0, Round, 8, "insufficient-relief"); });
		diagnosis("sheet.duplicate-contour-edit", function() { var invalid = SheetCutGeometry.applyEdits(graph,
			[new SheetContourEdit(2, graph.wall(2).outline), new SheetContourEdit(2, graph.wall(2).outline)], []); });
		near(graph.wall(1).outline.area(), 8000, "closure and rejected edits leave source root unchanged");
		near(graph.wall(2).outline.area(), 2500, "closure and rejected edits leave source first flange unchanged");
		near(graph.wall(3).outline.area(), 2000, "closure and rejected edits leave source second flange unchanged");
		return checks;
	}
	static function main():Void { trace("Sheet corner smoke passed (" + run() + " assertions)"); }
}
