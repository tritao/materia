import CadKit;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.OutlineBoolean;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.SheetDiagnosis;

/** Exact cut boundaries cover holes, open notches, circular intersections and refused disconnection. */
class SheetOutlineBooleanSmoke {
	static var assertions:Int = 0;
	static function check(value:Bool, message:String):Void { assertions++; if (!value) throw message; }
	static function near(actual:Float, expected:Float, message:String):Void { check(Math.abs(actual - expected) < 1e-7, message); }
	static function box(x:Float, y:Float, width:Float, height:Float):WallOutline {
		return WallOutline.polygon([new Point2(x, y), new Point2(x + width, y), new Point2(x + width, y + height), new Point2(x, y + height)]);
	}
	public static function run():Int {
		assertions = 0;
		var tiny = OutlineBoolean.subtract(WallOutline.rectangle(2e-5, 2e-5), box(1, 1, 1, 1), "tiny");
		check(Math.abs(tiny.area() / 4e-10 - 1) < 1e-9, "boundary assembly uses the wall's area tolerance rather than a length tolerance");
		var source = WallOutline.rectangle(20, 10);
		var hole = OutlineBoolean.subtract(source, box(5, 3, 4, 2), "hole");
		near(hole.area(), 192, "rectangular hole subtracts exact area"); check(hole.loops().length == 2, "interior cut creates a hole");
		for (i in 0...4) check(hole.identity(0, i) == source.identity(0, i), "untouched boundary retains identity");
		var notch = OutlineBoolean.subtract(source, box(-2, 3, 5, 4), "notch");
		near(notch.area(), 188, "open notch subtracts its intersection only"); check(notch.loops().length == 1, "open notch stays one boundary");
		var circle = new WallOutline([Segment.arc(new Point2(0, 5), 2, 0, 2 * Math.PI)]);
		var round = OutlineBoolean.subtract(source, circle, "round");
		near(round.area(), 200 - 2 * Math.PI, "half-circle notch has exact sector area");
		var arcCount = 0; for (segment in round.loops()[0]) if (!segment.isLine()) arcCount++;
		check(arcCount > 0, "round notch retains arcs rather than sampled chords");
		var disjoint = OutlineBoolean.subtract(source, box(30, 30, 5, 5), "outside"); near(disjoint.area(), 200, "disjoint cut leaves source intact");
		var nested = OutlineBoolean.subtract(hole, box(6, 3.5, 1, 1), "inside-hole"); near(nested.area(), 192, "cut within existing void changes no material");
		var coincident = OutlineBoolean.subtract(source, box(0, 0, 4, 10), "coincident"); near(coincident.area(), 160, "coincident boundaries do not duplicate segments");
		var disk = new WallOutline([Segment.arc(new Point2(0, 0), 5, 0, 2 * Math.PI)]);
		var secondDisk = new WallOutline([Segment.arc(new Point2(5, 0), 5, 0, 2 * Math.PI)]);
		var crescent = OutlineBoolean.subtract(disk, secondDisk, "lens");
		near(crescent.area(), 25 * Math.PI - 50 * Math.acos(0.5) + 12.5 * Math.sqrt(3), "circle-circle intersections retain exact lens area");
		var overlap = OutlineBoolean.subtract(hole, box(7, 2, 4, 4), "overlap");
		near(overlap.area(), 180, "overlapping cuts union their voids without double counting");
		var movedNotch = OutlineBoolean.subtract(source, box(-2, 3, 5, 3), "notch");
		check(notch.identities()[0].join("|") == movedNotch.identities()[0].join("|"), "authored notch fragment roles survive depth edits");
		for (outline in [round, crescent, overlap]) {
			var graph = new Sheet(SheetDemoParts.materialA()); var wall = graph.addWall(outline);
			var solid = SheetSolid.build(graph, graph.folded());
			check(CadKit.shapeValidChecked(solid.borrowHandle()) != 0, "subtracted exact region builds a valid native wall");
			near(solid.volume(), outline.area() * graph.thickness(), "native cut wall volume agrees with exact area"); solid.close();
		}
		var disconnected = false;
		try { var unused = OutlineBoolean.subtract(source, box(9, -1, 2, 12), "split"); }
		catch (error:Dynamic) { var diagnosis = Std.downcast(error, SheetDiagnosis); if (diagnosis != null) disconnected = diagnosis.code == "sheet.cut-disconnected"; }
		check(disconnected, "cut splitting a wall has a named diagnosis");
		var empty = false;
		try { var unused = OutlineBoolean.subtract(source, box(-1, -1, 22, 12), "all"); }
		catch (error:Dynamic) { var diagnosis = Std.downcast(error, SheetDiagnosis); if (diagnosis != null) empty = diagnosis.code == "sheet.cut-empty"; }
		check(empty, "cut consuming a wall has a named diagnosis");
		return assertions;
	}
	static function main():Void { trace("Outline difference smoke passed (" + run() + " assertions)"); }
}
