import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sheetmetal.FlatPattern;
import cadkit.sheetmetal.OutlineBoolean;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.SheetBaseFeature;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetCutGeometry;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.WallOutline;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/** Exact flat blank union, independent tray contour, interrupted fold lines and authored grain frames. */
class SheetFlatPatternSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(actual:Float, expected:Float, label:String):Void { check(Math.abs(actual - expected) < 1e-8, label + ": expected " + expected + ", got " + actual); }
	static function rectangle(width:Float, height:Float):ConstrainedSketch {
		var sketch = new ConstrainedSketch(); var points = [[0.0, 0.0], [width, 0.0], [width, height], [0.0, height]];
		for (i in 0...4) {
			var p = sketch.addPoint(new SketchPoint("p" + i, points[i][0], points[i][1]));
			var fixed = sketch.addConstraint(SketchConstraint.fixed("fixed" + i, "p" + i));
			var e = sketch.addEntity(SketchEntity.line("edge" + i, "p" + i, "p" + ((i + 1) % 4)));
		}
		return sketch;
	}
	public static function run():Int {
		checks = 0;
		for (part in [SheetDemoParts.p1(), SheetDemoParts.p2(), SheetDemoParts.p3(), SheetDemoParts.p4(), SheetDemoParts.p5()]) {
			var pattern = FlatPattern.of(part); near(pattern.regions().length, 1, "developed material has one connected blank");
			var area = 0.0; for (wall in part.walls()) area += wall.outline.area(); for (bend in part.bends()) area += part.bendAllowance(bend) * bend.edgeA.span.length();
			near(pattern.area(), area, "blank exact area equals its material domains");
			var last = 0;
			for (bend in pattern.bends()) { check(bend.id > last, "bend table follows permanent id order"); last = bend.id; near(bend.length(), part.bend(bend.id).edgeA.span.length(), "uncut material fold length"); }
		}
		var tray = FlatPattern.of(SheetDemoParts.p5()); var allowance = Math.PI / 2 * (1 + 0.38 * 1.5); var depth = 22.5 + allowance;
		near(tray.area(), 145 * 95 - 16 + 464 * depth, "hand-computed P5 blank area");
		var bounds = tray.bounds(); near(bounds.minX, -depth, "P5 left flange extent"); near(bounds.maxX, 145 + depth, "P5 right flange extent");
		near(bounds.minY, -depth, "P5 lower flange extent"); near(bounds.maxY, 95 + depth, "P5 upper flange extent");
		var expectedTray = WallOutline.polygon([
			new Point2(2, -depth), new Point2(143, -depth), new Point2(143, 2),
			new Point2(145 + depth, 2), new Point2(145 + depth, 93), new Point2(143, 93),
			new Point2(143, 95 + depth), new Point2(2, 95 + depth), new Point2(2, 93),
			new Point2(-depth, 93), new Point2(-depth, 2), new Point2(2, 2)
		]);
		near(OutlineBoolean.regions(tray.regions()[0], expectedTray, "expected-tray").length, 0, "P5 has no material outside the hand-computed outline");
		near(OutlineBoolean.regions(expectedTray, tray.regions()[0], "actual-tray").length, 0, "P5 fills the hand-computed outline");
		var source = SheetDemoParts.p1(); var bend = source.bends()[0]; var ba = source.bendAllowance(bend);
		var circle = new WallOutline([Segment.arc(new Point2(57.5 + ba / 2, 50), 2, 0, 2 * Math.PI)], [], [["circle"]]);
		var cut = SheetCutGeometry.apply(source, 1, circle, "flat-hole"); var pattern = FlatPattern.of(cut);
		near(pattern.area(), FlatPattern.of(source).area() - 4 * Math.PI, "one exact circle crosses three developed domains");
		near(pattern.regions()[0].loops().length, 2, "cross-bend cut becomes one exact blank hole");
		near(pattern.bends()[0].lines().length, 2, "fold marking avoids the cut hole"); near(pattern.bends()[0].length(), 96, "effective material bend length");
		var marker = pattern.bends()[0].lines(); marker.pop(); near(pattern.bends()[0].lines().length, 2, "pattern owns marker arrays");
		var ring = new WallOutline([Segment.arc(new Point2(0, 0), 5, 0, 2 * Math.PI)], [[Segment.arc(new Point2(0, 0), 4, 0, -2 * Math.PI)]]);
		var innerRing = new WallOutline([Segment.arc(new Point2(0, 0), 2, 0, 2 * Math.PI)], [[Segment.arc(new Point2(0, 0), 1, 0, -2 * Math.PI)]]);
		near(OutlineBoolean.union(ring, innerRing).length, 2, "nested disconnected rings retain the nearest hole owner");
		var stock = SheetDemoParts.materialA(); var grained = new SheetMaterial(stock.id, stock.thickness, stock.bendRule, stock.mechanical,
			stock.defaultRadiusRatio, stock.minRadiusRatio, Math.PI / 2);
		var document = new Document(); var profile = document.add(new ConstrainedSketchFeature(rectangle(20, 10)));
		var base = document.add(new SheetBaseFeature(profile, grained)); document.setOutput(base); document.recompute();
		var grainPattern = FlatPattern.of(base.sheet()); var grainBounds = grainPattern.bounds();
		near(grainBounds.maxX - grainBounds.minX, 10, "blank X follows grain"); near(grainBounds.maxY - grainBounds.minY, 20, "grain frame rotates the stock rectangle exactly");
		var loaded = DocumentCodec.decode(DocumentCodec.encode(document)); var restored = Std.downcast(loaded.featureById(base.id.toInt()), SheetBaseFeature);
		if (restored == null) throw "unexpected saved grained base";
		var angle = restored.stock().grainAngle; if (angle == null) throw "lost authored grain";
		near(angle, Math.PI / 2, "typed grain snapshot round trip");
		var diagonalStock = new SheetMaterial(stock.id, stock.thickness, stock.bendRule, stock.mechanical,
			stock.defaultRadiusRatio, stock.minRadiusRatio, -Math.PI / 4);
		var diagonal = diagonalStock.grainAngle; if (diagonal == null) throw "missing diagonal grain";
		near(diagonal, 3 * Math.PI / 4, "undirected grain normalizes modulo pi");
		var diagonalSheet = new Sheet(diagonalStock); var diagonalWall = diagonalSheet.addWall(WallOutline.rectangle(20, 10));
		var diagonalPattern = FlatPattern.of(diagonalSheet); var diagonalBounds = diagonalPattern.bounds();
		near(diagonalBounds.maxX - diagonalBounds.minX, 30 / Math.sqrt(2), "arbitrary grain blank width");
		near(diagonalBounds.maxY - diagonalBounds.minY, 30 / Math.sqrt(2), "arbitrary grain blank height");
		near(diagonalPattern.plane.xDirection.x, -1 / Math.sqrt(2), "exported plane follows diagonal rolling axis");
		var isolatedDomains = pattern.domains(); var domainCount = isolatedDomains.length; isolatedDomains.pop();
		near(pattern.domains().length, domainCount, "pattern owns mapped domain arrays");
		loaded.close(); document.close(); return checks;
	}
	static function main():Void { trace("Flat pattern smoke passed (" + run() + " assertions)"); }
}
