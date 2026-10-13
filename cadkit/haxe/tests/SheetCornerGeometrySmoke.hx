import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.CornerGeometry;
import cadkit.sheetmetal.CornerJoin;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.WallOutlineEditor;

/** Independent slab-distance checks cover orthogonal and oblique corner edge offsets. */
class SheetCornerGeometrySmoke {
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
		var outline = WallOutline.rectangle(20, 40);
		var edge = new WallEdge(1);
		var first = Plane.XY();
		var second = new Plane(new Vector(30, 0, 10), Vector.Z().scale(-1), Vector.X());
		var result = CornerGeometry.of(outline, edge, first, outline, edge, second, 2, Gap, 0.5);
		near(result.first, 7.5, "first edge stops before mate side B plus gap");
		near(result.second, -10.5, "second edge stops before mate side A plus gap");
		var lapped = CornerGeometry.of(outline, edge, first, outline, edge, second, 2, OverlapFirst, 0.5, 3);
		near(lapped.first, 13, "first overlap passes mate side A by the stated amount");
		near(lapped.second, result.second, "mate retains its stand-off");
		var reversed = CornerGeometry.of(outline, edge, second, outline, edge, first, 2, OverlapSecond, 0.5, 3);
		near(reversed.first, lapped.second, "swapping flange order preserves mate offset");
		near(reversed.second, lapped.first, "swapping flange order preserves overlap offset");
		var changed = WallOutlineEditor.offset(outline, edge, result.first);
		near(changed.segment(0, 1).start.x, 27.5, "supporting edge moves by the derived offset");
		check(changed.identity(0, 1) == outline.identity(0, 1), "corner edge retains its authored identity");
		near(outline.area(), 800, "derived corner geometry leaves source untouched");
		// At an oblique corner, check every depth endpoint against the mate slab independently.
		for (angle in [Math.PI / 6, Math.PI / 3, 2 * Math.PI / 3, 5 * Math.PI / 6]) {
			var mate = new Plane(new Vector(35, 0, 8), new Vector(Math.cos(angle), 0, -Math.sin(angle)),
				new Vector(Math.sin(angle), 0, Math.cos(angle)));
			var offsets = CornerGeometry.of(outline, edge, first, outline, edge, mate, 2, Gap, 0.75);
			var signed:Array<Float> = [];
			for (depth in [0.0, -2.0]) signed.push(first.toWorld(new Vector(20 + offsets.first, 20, depth)).subtract(mate.origin).dot(mate.normal));
			near(Math.max(signed[0], signed[1]), -2.75, "oblique entire thickness clears mate slab");
			signed = [];
			for (depth in [0.0, -2.0]) signed.push(-mate.toWorld(new Vector(20 + offsets.second, 20, depth)).z);
			near(Math.max(signed[0], signed[1]), -0.75, "other oblique flange clears the opposite slab face");
			var swapped = CornerGeometry.of(outline, edge, mate, outline, edge, first, 2, Gap, 0.75);
			near(swapped.first, offsets.second, "oblique gap is independent of selection order");
			near(swapped.second, offsets.first, "oblique mate gap is independent of selection order");
			var overlap = CornerGeometry.of(outline, edge, first, outline, edge, mate, 2, OverlapFirst, 0.75, 1.25);
			signed = [];
			for (depth in [0.0, -2.0]) signed.push(first.toWorld(new Vector(20 + overlap.first, 20, depth)).subtract(mate.origin).dot(mate.normal));
			near(Math.min(signed[0], signed[1]), 1.25, "oblique overlap clears far face over entire thickness");
		}
		diagnosis("sheet.corner-planes", function() { var invalid = CornerGeometry.of(outline, edge, first, outline, edge, first, 2, Gap, 0); });
		diagnosis("sheet.corner-boundary", function() { var invalid = CornerGeometry.of(outline, new WallEdge(0), first, outline, edge, second, 2, Gap, 0); });
		diagnosis("sheet.invalid-corner-dimension", function() { var invalid = CornerGeometry.of(outline, edge, first, outline, edge, second, 2, Gap, -1); });
		diagnosis("sheet.invalid-corner-dimension", function() { var invalid = CornerGeometry.of(outline, edge, first, outline, edge, second, 2, Gap, 0, 1); });
		diagnosis("sheet.invalid-corner-dimension", function() { var invalid = CornerGeometry.of(outline, edge, first, outline, edge, second, 2, OverlapFirst, 0); });
		return checks;
	}
	static function main():Void { trace("Sheet corner geometry smoke passed (" + run() + " assertions)"); }
}
