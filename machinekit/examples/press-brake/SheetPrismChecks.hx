import cadkit.modeling.Axis;
import cadkit.modeling.Location;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.WallOutline;
import machinekit.pressbrake.planning.ConvexPrism;
import machinekit.pressbrake.planning.PrismSatQuery;
import machinekit.pressbrake.planning.SheetPrismProfile;

/** Material decomposition preserves concavity and holes; curved boundaries report their approximation. */
class SheetPrismChecks {
	static var checks:Int = 0;
	public static function main():Void run();
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(actual:Float, expected:Float, label:String):Void {
		check(Math.isFinite(actual) && Math.abs(actual - expected) < 1e-8, label + ": " + actual + " != " + expected);
	}
	static function probe(x:Float, y:Float):ConvexPrism {
		return new ConvexPrism("probe", [new Vector(x - 0.1, y - 0.1, -1), new Vector(x + 0.1, y - 0.1, -1),
			new Vector(x + 0.1, y + 0.1, -1), new Vector(x - 0.1, y + 0.1, -1)], new Vector(0, 0, 0.5));
	}
	static function overlaps(profile:SheetPrismProfile, point:ConvexPrism):Bool {
		var query = new PrismSatQuery();
		for (piece in profile.pieces()) if (query.overlap(piece, point)) return true;
		return false;
	}
	static function prismVolume(profile:SheetPrismProfile):Float {
		var volume = 0.0;
		for (piece in profile.pieces()) {
			var vertices = piece.vertices(), count = Std.int(vertices.length / 2), area = 0.0;
			for (i in 0...count) {
				var next = vertices[(i + 1) % count]; area += vertices[i].x * next.y - next.x * vertices[i].y;
			}
			volume += Math.abs(area * piece.extrusion.z) / 2;
		}
		return volume;
	}
	public static function run():Void {
		checks = 0;
		var rectangle = SheetPrismProfile.of("rectangle", WallOutline.rectangle(10, 10), 1.5);
		near(rectangle.area, 100, "Rectangle decomposition conserves area");
		near(prismVolume(rectangle), 150, "Rectangle prisms conserve stock volume");
		near(rectangle.curveError, 0, "Lines are not approximated");
		var concaveOutline = WallOutline.polygon([new Point2(0, 0), new Point2(10, 0), new Point2(10, 3),
			new Point2(3, 3), new Point2(3, 10), new Point2(0, 10)]);
		var concave = SheetPrismProfile.of("concave", concaveOutline, 1.5);
		near(concave.area, 51, "Concave decomposition preserves missing quadrant");
		near(prismVolume(concave), 76.5, "Concave stock volume");
		check(!overlaps(concave, probe(8, 8)), "Concave relief remains empty");
		check(overlaps(concave, probe(1, 8)), "Concave material remains present");
		var hole = [Segment.line(new Point2(2, 2), new Point2(2, 8)),
			Segment.line(new Point2(2, 8), new Point2(8, 8)),
			Segment.line(new Point2(8, 8), new Point2(8, 2)), Segment.line(new Point2(8, 2), new Point2(2, 2))];
		var holedOutline = new WallOutline(WallOutline.rectangle(10, 10).loops()[0], [hole]);
		var holed = SheetPrismProfile.of("holed", holedOutline, 1.5);
		near(holed.area, 64, "Hole area is subtracted");
		near(prismVolume(holed), 96, "Hole stock volume");
		check(!overlaps(holed, probe(5, 5)), "Hole does not acquire a filled convex hull");
		check(overlaps(holed, probe(1, 1)), "Material outside hole is present");
		check(overlaps(holed, probe(2, 5)), "Probe penetrating hole boundary hits material");
		var transform = Location.translation(new Vector(1000, 2000, 3000)).compose(Location.rotation(Axis.X(), 0.3));
		var transformedProbe = probe(5, 5).placed(transform), query = new PrismSatQuery();
		var hit = false;
		for (piece in holed.pieces()) if (query.overlap(piece.placed(transform), transformedProbe)) hit = true;
		check(!hit, "Rigid placement preserves the hole");
		var annulusOutline = new WallOutline([Segment.arc(new Point2(5, 5), 5, 0, 2 * Math.PI)],
			[[Segment.arc(new Point2(5, 5), 2, 0, -2 * Math.PI)]]);
		var annulus = SheetPrismProfile.of("annulus", annulusOutline, 1.5, 0.01);
		check(annulus.curveError > 0 && annulus.curveError <= 0.01 + 1e-12, "Circular chord error is explicit and bounded");
		check(Math.abs(annulus.area - 21 * Math.PI) <= 2 * Math.PI * 7 * 0.01, "Annulus area lies within independent perimeter/error bound");
		check(!overlaps(annulus, probe(5, 5)), "Circular hole remains empty");
		check(overlaps(annulus, probe(8.5, 5)), "Circular wall material is present");
		check(!overlaps(annulus, probe(10.2, 5)), "Outside circle remains empty");
		var pieces = holed.pieces(), original = pieces.length; var removed = pieces.pop();
		check(holed.pieces().length == original, "Caller cannot mutate decomposition array");
		trace("Sheet prism checks passed (" + checks + " assertions)");
	}
}
