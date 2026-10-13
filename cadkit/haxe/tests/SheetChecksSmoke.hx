import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.SegmentDistance;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetChecks;
import cadkit.sheetmetal.FlangeDepth;
import cadkit.sheetmetal.BendAttachment;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetCheckKind;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.SheetCutGeometry;
import cadkit.sheetmetal.ReliefGeometry;
import cadkit.sheetmetal.ReliefType;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.WallEdge;

/** Independent clearance queries and crafted parts exercise every manufacturability finding. */
class SheetChecksSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(a:Float, b:Float, label:String):Void { check(Math.abs(a - b) < 1e-8, label); }
	public static function run():Int {
		checks = 0;
		for (part in [SheetDemoParts.p1(), SheetDemoParts.p2(), SheetDemoParts.p3(), SheetDemoParts.p4(), SheetDemoParts.p5()])
			check(SheetChecks.of(part).findings().length == 0, "P1-P5 have no manufacturability findings");
		var short = new Sheet(SheetDemoParts.materialA()); var base = short.addWall(WallOutline.rectangle(30, 30));
		var child = SheetDemoParts.flange(short, base, 1, 2, Math.PI / 2);
		var shortReport = SheetChecks.of(short);
		check(shortReport.has(ShortFlange), "short flange finding");
		for (finding in shortReport.findings()) if (finding.kind == ShortFlange) {
			check(finding.assumed && finding.code == "sheet.short-flange", "placeholder tooling rule is explicitly assumed");
			switch (finding.measure) { case DistanceMm(actual, minimum): near(actual, 2, "actual flange leg"); near(minimum, 8.4, "0.7V minimum with default V=8T"); case _: throw "wrong flange measurement"; }
			var geometry = finding.geometry(); var length = geometry.length; geometry.pop(); near(finding.geometry().length, length, "finding owns geometry arrays");
		}
		var tapered = SheetDemoParts.p1(); var taperedBend = tapered.bends()[0];
		var taper = tapered.updateWallMaterial(taperedBend.wallB, WallOutline.polygon([new Point2(0, 0), new Point2(2, 0), new Point2(20, 100), new Point2(0, 100)]));
		near(FlangeDepth.minimum(tapered.wall(taperedBend.wallB).outline, taperedBend.edgeB), 2, "minimum tapered leg is not its maximum envelope");
		check(SheetChecks.of(tapered).has(ShortFlange), "short tapered end triggers tooling finding");
		var curved = SheetDemoParts.p1(); var curvedBend = curved.bends()[0];
		var curvedWall = curved.updateWallMaterial(curvedBend.wallB, new WallOutline([
			Segment.line(new Point2(0, 0), new Point2(20, 0)), Segment.line(new Point2(20, 0), new Point2(20, 32)),
			Segment.arc(new Point2(20, 50), 18, -Math.PI / 2, -Math.PI), Segment.line(new Point2(20, 68), new Point2(20, 100)),
			Segment.line(new Point2(20, 100), new Point2(0, 100)), Segment.line(new Point2(0, 100), new Point2(0, 0))]));
		near(FlangeDepth.minimum(curvedWall.outline, curvedBend.edgeB), 2, "arc notch minimum uses an analytic extremum");
		check(SheetChecks.of(curved).has(ShortFlange), "arc notch shortens the effective flange");
		var stock = SheetDemoParts.materialA();
		var ring = new Sheet(stock); var ringWalls = [for (i in 0...4) ring.addWall(WallOutline.rectangle(10, 3))];
		for (i in 0...4) { var joined = ring.addBend(ringWalls[i].id, new WallEdge(1), ringWalls[(i + 1) % 4].id, new WallEdge(3), Math.PI / 2, 1); }
		var rip = ring.rip(4); var ringReport = SheetChecks.of(ring); var freeStrip = false;
		for (finding in ringReport.findings()) if (finding.kind == ShortFlange) switch (finding.measure) {
			case DistanceMm(actual, _): if (actual == 0) freeStrip = true;
			case _: {}
		}
		check(freeStrip, "ripped strip does not borrow the detached wall's return length");
		var limited = new Sheet(new SheetMaterial(stock.id, stock.thickness, stock.bendRule, stock.mechanical, 2, 1));
		var root = limited.addWall(WallOutline.rectangle(30, 30)); var flange = SheetDemoParts.flange(limited, root, 1, 20, Math.PI / 2, 1);
		check(SheetChecks.of(limited).has(SmallRadius), "explicit material radius minimum");
		var repaired = limited.updateBend(limited.bends()[0].id, Math.PI / 2, 1.5);
		check(SheetChecks.of(limited).findings().length == 0, "radius equal to the material minimum is accepted");
		var grained = new Sheet(new SheetMaterial(stock.id, stock.thickness, stock.bendRule, stock.mechanical, null, null, Math.PI / 2));
		var grainRoot = grained.addWall(WallOutline.rectangle(30, 30)); var grainFlange = SheetDemoParts.flange(grained, grainRoot, 1, 20, Math.PI / 2);
		check(SheetChecks.of(grained).has(AlongGrain), "parallel grain finding");
		for (offset in [0.0, 0.001]) {
			var angled = new Sheet(new SheetMaterial(stock.id, stock.thickness, stock.bendRule, stock.mechanical, null, null, Math.PI / 2 - Math.PI / 12 - offset));
			var angledRoot = angled.addWall(WallOutline.rectangle(30, 30)); var angledFlange = SheetDemoParts.flange(angled, angledRoot, 1, 20, Math.PI / 2);
			check(SheetChecks.of(angled).has(AlongGrain) == (offset == 0), "15-degree grain boundary is inclusive");
		}
		var drilled = SheetDemoParts.p1(); var hole = drilled.cutHole(1, [Segment.arc(new Point2(55, 50), 1, 0, -2 * Math.PI)]);
		var drilledReport = SheetChecks.of(drilled);
		check(drilledReport.has(CutNearBend), "exact hole clearance finding");
		for (finding in drilledReport.findings()) if (finding.kind == CutNearBend) {
			check(finding.assumed, "hole clearance rule is explicitly assumed");
			switch (finding.measure) { case DistanceMm(actual, minimum): near(actual, 1.5 + Math.PI / 4 * 1.57, "independent exact hole-edge clearance"); near(minimum, 4.75, "2.5T plus R"); case _: throw "wrong clearance measurement"; }
		}
		var notched = SheetCutGeometry.apply(SheetDemoParts.p1(), 1, new WallOutline([Segment.arc(new Point2(57.5, -1), 2, 0, 2 * Math.PI)]), "end-notch");
		check(SheetChecks.of(notched).has(CutNearBend), "outer cut notch also triggers clearance");
		var far = SheetCutGeometry.apply(SheetDemoParts.p1(), 1, new WallOutline([Segment.arc(new Point2(-1, 50), 2, 0, 2 * Math.PI)]), "far-notch");
		check(SheetChecks.of(far).findings().length == 0, "far notch is silent");
		var relieved = new Sheet(stock); var raw = WallOutline.rectangle(30, 30);
		var reliefRoot = relieved.addWall(ReliefGeometry.cut(raw, raw.segment(0, 1), 0, 0, Rectangular, 1, 2, "relief"));
		var selected = -1; var identities = reliefRoot.outline.identities()[0];
		for (i in 0...identities.length) if (identities[i] == "0:1") selected = i;
		if (selected < 0) throw "lost authored attachment";
		var reliefFlange = SheetDemoParts.flange(relieved, reliefRoot, selected, 20, Math.PI / 2);
		check(SheetChecks.of(relieved).findings().length == 0, "explicit relief boundaries are exempt from cut clearance");
		var internal = SheetDemoParts.p1(); var bend = internal.bends()[0]; var allowance = internal.bendAllowance(bend);
		var lobe = internal.updateWallMaterial(bend.wallB, WallOutline.polygon([
			new Point2(-allowance, -5), new Point2(37.5, -5), new Point2(37.5, 100), new Point2(0, 100), new Point2(0, 0), new Point2(-allowance, 0)]));
		check(SheetChecks.of(internal).has(MissingRelief), "fold line terminates inside the developed blank");
		var overlapping = new Sheet(stock); var overlapRoot = overlapping.addWall(WallOutline.rectangle(20, 20));
		for (edge in [0, 1]) {
			var side = SheetDemoParts.flange(overlapping, overlapRoot, edge, 20, Math.PI / 2);
			var extended = overlapping.updateWallMaterial(side.id, WallOutline.polygon([new Point2(0, -10), new Point2(20, -10), new Point2(20, 30), new Point2(0, 30)]));
		}
		check(SheetChecks.of(overlapping).has(FlatOverlap), "extended adjacent flanges overlap when developed");
		var line = Segment.line(new Point2(-10, 0), new Point2(10, 0));
		near(SegmentDistance.toLine(Segment.arc(new Point2(0, 5), 2, 0, 2 * Math.PI), line), 3, "circle clearance uses its arc, not its chord");
		near(SegmentDistance.toLine(Segment.arc(new Point2(0, 5), 2, 0, Math.PI), line), 5, "excluded radial minimum falls back to arc endpoints");
		near(SegmentDistance.support(Segment.arc(new Point2(0, 0), 2, 0, 2 * Math.PI), new Point2(0.6, 0.8)), 2, "arbitrary-direction support is exact");
		var refused = false;
		try { var missing = FlangeDepth.minimum(WallOutline.rectangle(10, 10), new BendAttachment("missing", Segment.line(new Point2(20, 0), new Point2(20, 10)))); }
		catch (error:SheetDiagnosis) { refused = error.code == "sheet.flange-depth"; }
		check(refused, "unmeasurable attachment is diagnosed rather than assigned infinite depth");
		return checks;
	}
	static function main():Void { trace("Sheet checks smoke passed (" + run() + " assertions)"); }
}
