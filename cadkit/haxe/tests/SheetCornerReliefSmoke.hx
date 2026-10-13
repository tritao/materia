import CadKit;
import cadkit.sheetmetal.BoundaryOrigin;
import cadkit.sheetmetal.CornerRelief;
import cadkit.sheetmetal.CornerReliefGeometry;
import cadkit.sheetmetal.FlatPattern;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetFlange;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.WallOutline;

/** Exact corner reliefs retain graph datums and material purpose through walls and bend strips. */
class SheetCornerReliefSmoke {
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
	public static function run():Int {
		checks = 0;
		var center = new Point2(80, 100);
		var round = CornerReliefGeometry.tool(center, 1, 0, Round, 8);
		near(round.area(), Math.PI * 16, "round relief is an exact diameter-eight circle");
		var square = CornerReliefGeometry.tool(center, Math.cos(0.7), Math.sin(0.7), Square, 8);
		near(square.area(), 64, "square retains its true area under rotation");
		diagnosis("sheet.invalid-corner-relief", function() { var invalid = CornerReliefGeometry.tool(center, 1, 0, Round, 0); });
		var base = new Sheet(SheetDemoParts.materialA()); var wall = base.addWall(WallOutline.rectangle(80, 100));
		var first = SheetFlange.add(base, wall.id, new WallEdge(1), 25, Math.PI / 2, 1);
		var source = SheetFlange.add(first, wall.id, new WallEdge(2), 25, Math.PI / 2, 1);
		for (type in [CornerRelief.Round, CornerRelief.Square]) {
			var result = CornerReliefGeometry.apply(source, 1, 2, type, 8, "corner-test");
			near(source.wall(1).outline.area(), 8000, "relief leaves source wall unchanged");
			near(result.wall(1).outline.area(), 8000 - (type == Round ? 4 * Math.PI : 16), "parent removes one quarter of the relief");
			check(result.walls().length == 3 && result.bends().length == 2, "relief retains wall and bend graph identities");
			for (bend in result.bends()) {
				check(bend.hasCuts(), "corner tool crosses each adjoining bend strip");
				near(bend.edgeA.span.length(), source.bend(bend.id).edgeA.span.length(), "nominal seam remains a datum");
				var count = 0;
				for (region in bend.materialRegions(result.bendAllowance(bend))) for (loop in region.origins()) for (origin in loop) {
					switch (origin) { case Relief(owner): check(owner == "corner-test", "bend relief retains authored owner"); count++; case _: }
				}
				check(count > 0, "bend contour retains relief purpose");
			}
			var wallReliefs = 0;
			for (loop in result.wall(1).outline.origins()) for (origin in loop) {
				switch (origin) { case Relief(owner): check(owner == "corner-test", "wall relief retains authored owner"); wallReliefs++; case _: }
			}
			check(wallReliefs > 0, "parent contour names corner relief");
			var flat = FlatPattern.of(result); var domainArea = 0.0;
			for (domain in flat.domains()) domainArea += domain.outline.area();
			near(flat.area(), domainArea, "relieved developed domains join without material overlap");
			check(flat.regions().length == 1 && flat.bends().length == 2, "relief leaves one blank with both bend guides");
			for (state in [result.folded(), result.flat()]) {
				var shape = SheetSolid.build(result, state);
				try { check(CadKit.shapeValidChecked(shape.borrowHandle()) != 0, "relieved corner builds valid native geometry"); }
				catch (failure:Dynamic) { shape.close(); throw failure; }
				shape.close();
			}
		}
		diagnosis("sheet.corner-adjacency", function() { var invalid = CornerReliefGeometry.apply(source, 1, 1, Round, 8, "invalid"); });
		return checks;
	}
	static function main():Void { trace("Sheet corner relief smoke passed (" + run() + " assertions)"); }
}
