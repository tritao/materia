import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.BoundaryOrigin;
import cadkit.sheetmetal.FlangeGeometry;
import cadkit.sheetmetal.FlatPattern;
import cadkit.sheetmetal.OutlineBoolean;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.ReliefGeometry;
import cadkit.sheetmetal.ReliefType;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetProfile;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.WallOutlineEditor;

/** Boundary purpose follows exact material geometry, independently of authored naming and loop order. */
class SheetBoundaryOriginSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function key(origin:BoundaryOrigin):String {
		return switch (origin) { case Authored: "authored"; case Cut(owner): "cut " + owner; case Relief(owner): "relief " + owner; };
	}
	static function count(outline:WallOutline, cut:Bool):Int {
		var result = 0;
		for (roles in outline.origins()) for (origin in roles) switch (origin) {
			case Cut(_): if (cut) result++;
			case Relief(_): if (!cut) result++;
			case Authored: {}
		}
		return result;
	}
	public static function run():Int {
		checks = 0;
		var stock = WallOutline.rectangle(40, 20);
		var tool = new WallOutline([Segment.arc(new Point2(39, 10), 2, 0, 2 * Math.PI)]);
		var notch = OutlineBoolean.subtract(stock, tool, "notch");
		check(count(notch, true) > 0 && count(notch, false) == 0, "actual notch boundaries have typed cut purpose");
		for (roles in notch.origins()) for (origin in roles) switch (origin) {
			case Cut(owner): check(owner == "notch", "cut owner is authored data");
			case _: {}
		}
		var relief = ReliefGeometry.cut(stock, stock.segment(0, 1), 0, 0, Rectangular, 1, 2, "flange");
		check(count(relief, false) > 0 && count(relief, true) == 0, "relief slots retain their explicit purpose");
		var roles = stock.origins(); roles[0][1] = Cut("selected");
		check(key(stock.origin(0, 1)) == "authored", "origin access returns isolated arrays");
		var marked = new WallOutline(stock.loops()[0], [], stock.identities(), roles); roles[0][1] = Authored;
		check(key(marked.origin(0, 1)) == "cut selected", "outline owns supplied origin arrays");
		var spanned = FlangeGeometry.span(marked, new WallEdge(1), 2, 3);
		check(count(spanned.outline, true) == 3, "all authored-edge fragments retain cut purpose");
		check(count(WallOutlineEditor.offset(marked, new WallEdge(1), 1), true) == 1, "supporting-edge edits retain purpose");
		var reflected = SheetProfile.transform(notch, Plane.XY(), new Plane(new Vector(), Vector.X(), Vector.Z().scale(-1)));
		var originals:Map<String, String> = [];
		var ids = notch.identities();
		for (i in 0...ids.length) for (j in 0...ids[i].length) originals.set(ids[i][j], key(notch.origin(i, j)));
		var reflectedIds = reflected.identities();
		for (i in 0...reflectedIds.length) for (j in 0...reflectedIds[i].length)
			check(originals.get(reflectedIds[i][j]) == key(reflected.origin(i, j)), "normal reversal maps identities and purposes together");
		var sheet = new Sheet(SheetDemoParts.materialA()); var wall = sheet.addWall(relief);
		check(count(FlatPattern.of(sheet).regions()[0], false) == count(relief, false), "developed drawing retains relief purpose");
		var adjacent = SheetProfile.transform(stock, new Plane(new Vector(40, 0), Vector.X(), Vector.Z()), Plane.XY());
		adjacent = new WallOutline(adjacent.loops()[0], [], [["next:0", "next:1", "next:2", "next:3"]], adjacent.origins());
		var united = OutlineBoolean.union(notch, adjacent);
		check(united.length == 1 && count(united[0], true) > 0, "flat union retains visible cut boundaries");
		return checks;
	}
	static function main():Void { trace("Boundary origin smoke passed (" + run() + " assertions)"); }
}
