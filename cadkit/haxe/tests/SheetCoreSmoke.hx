import cadkit.modeling.Location;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.KFactorTable;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.OutlineGeometry;
import cadkit.sheetmetal.OutlineDifference;
import cadkit.sheetmetal.BendSeam;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.SheetState;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.WallOutline;

/** Native-free graph, exact curves and folding invariants for SM1. */
class SheetCoreSmoke {
	static var checks:Int = 0;
	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-9):Void {
		checks++;
		if (!Math.isFinite(actual) || Math.abs(actual - expected) > tolerance)
			throw message + ": expected " + expected + ", got " + actual;
	}
	static function vector(actual:Vector, expected:Vector, message:String):Void {
		near(actual.x, expected.x, message + " x");
		near(actual.y, expected.y, message + " y");
		near(actual.z, expected.z, message + " z");
	}
	static function frame(actual:Location, expected:Location, message:String):Void {
		vector(actual.plane.origin, expected.plane.origin, message + " origin");
		vector(actual.plane.xDirection, expected.plane.xDirection, message + " x direction");
		vector(actual.plane.normal, expected.plane.normal, message + " normal");
	}
	static function failure(code:String, action:Void->Void):Void {
		checks++;
		try { action(); } catch (error:Dynamic) {
			if (Std.isOfType(error, SheetDiagnosis)) {
				var diagnosis = Std.downcast(error, SheetDiagnosis); if (diagnosis == null) throw "unexpected error type";
				if (diagnosis.code == code) return;
			}
			throw "Expected " + code + ", got " + Std.string(error);
		}
		throw "Expected " + code;
	}
	public static function run():Int {
		outlines();
		var parts = [SheetDemoParts.p1(), SheetDemoParts.p2(), SheetDemoParts.p3(), SheetDemoParts.p4(), SheetDemoParts.p5()];
		var widths = [60.0, 60.0, 60.0, 97.0, 150.0];
		var lengths = [100.0, 120.0, 120.0, 120.0, 100.0];
		var heights = [40.0, 40.0, 78.5, 40.0, 25.0];
		for (i in 0...parts.length) {
			var sheet = parts[i];
			var folded = sheet.evaluate(sheet.folded());
			var bounds = folded.bounds();
			near(bounds.maxX - bounds.minX, widths[i], "P" + (i + 1) + " width");
			near(bounds.maxY - bounds.minY, lengths[i], "P" + (i + 1) + " length");
			near(bounds.maxZ - bounds.minZ, heights[i], "P" + (i + 1) + " height");
			invariants(sheet);
		}
		materialSeams();
		closure();
		diagnoses();
		return checks;
	}
	static function outlines():Void {
		var circle = Segment.arc(new Point2(4, 5), 3, 0, 2 * Math.PI);
		var region = new WallOutline([circle]);
		near(region.area(), 9 * Math.PI, "full-circle exact area");
		near(circle.length(), 6 * Math.PI, "full-circle exact length");
		near(circle.point(0.25).y, 8, "arc point");
		var hole = [Segment.arc(new Point2(4, 5), 1, 0, -2 * Math.PI)];
		var cut = region.withHole(hole);
		near(cut.area(), 8 * Math.PI, "hole exact area");
		near(OutlineGeometry.contains(hole, new Point2(4, 5)) ? 1 : 0, 1, "clockwise full-circle winding at seam height");
		failure("sheet.invalid-outline", function() { var value = region.withHole([Segment.arc(new Point2(20, 5), 1, 0, -2 * Math.PI)]); });
		failure("sheet.invalid-outline", function() { var value = region.withHole([Segment.arc(new Point2(6.5, 5), 1, 0, -2 * Math.PI)]); });
		failure("sheet.invalid-outline", function() { var value = cut.withHole([Segment.arc(new Point2(4.5, 5), 1, 0, -2 * Math.PI)]); });
		failure("sheet.invalid-outline", function() { var value = cut.withHole([Segment.arc(new Point2(4, 5), 0.5, 0, -2 * Math.PI)]); });
		var straight = WallOutline.rectangle(10, 10);
		var straightCut = straight.withHole([Segment.arc(new Point2(5, 5), 2, 0, -2 * Math.PI)]);
		near(straightCut.area(), 100 - 4 * Math.PI, "line/arc region");
		failure("sheet.invalid-outline", function() { var value = straight.withHole([Segment.arc(new Point2(9, 5), 2, 0, -2 * Math.PI)]); });
		// Positive area alone does not prove a simple boundary.
		failure("sheet.invalid-outline", function() { var value = WallOutline.polygon([new Point2(0, 0), new Point2(10, 0),
			new Point2(0, 8), new Point2(10, 10), new Point2(0, 10)]); });
		var loops = cut.loops();
		loops[0].pop();
		near(cut.loops()[0].length, 1, "outline owns loop arrays");
		var curved = new Sheet(SheetDemoParts.materialA());
		var w = curved.addWall(cut);
		var bb = curved.evaluate(curved.flat()).bounds();
		near(bb.minX, 1, "arc bounds min x");
		near(bb.maxY, 8, "arc bounds max y");
		near(bb.minZ, -1.5, "side B is negative Z");
		var snapshot = curved.evaluate(curved.flat());
		var edit = curved.updateWall(w.id, WallOutline.rectangle(1, 1));
		near(snapshot.bounds().maxY, 8, "evaluation keeps an immutable geometry snapshot");
		failure("sheet.invalid-outline", function() { var value = WallOutline.polygon([new Point2(0, 0), new Point2(0, 1), new Point2(1, 0)]); });
		failure("sheet.invalid-outline", function() { var value = new WallOutline([Segment.line(new Point2(0, 0), new Point2(1, 0))]); });
	}
	static function invariants(sheet:Sheet):Void {
		var flat = sheet.evaluate(sheet.flat());
		for (w in sheet.walls()) {
			near(flat.wall(w.id).plane.origin.z, 0, "flat walls share side-A plane");
			vector(flat.wall(w.id).plane.normal, Vector.Z(), "flat wall normal");
		}
		for (fraction in [0.0, 1e-10, 0.25, 0.5, 1.0]) {
			var state = new SheetState();
			for (b in sheet.bends()) state.set(b.id, b.angle * fraction);
			var evaluated = sheet.evaluate(state);
			for (b in sheet.bends()) {
				var zone = evaluated.bend(b.id);
				near(zone.allowance, sheet.bendAllowance(b), "fixed neutral length");
				if (fraction > 0) {
					var q = zone.curvedNeutralRadius();
					near(q * Math.abs(zone.angle), sheet.bendAllowance(b), "neutral arc length");
					if (fraction == 1) {
						var r = zone.curvedInnerRadius();
						near(r, b.radius, "target inner radius");
					}
					if (fraction >= 0.25) {
						var offset = sheet.bendAllowance(b) / Math.abs(b.angle) - b.radius;
						var depth = b.angle > 0 ? offset : sheet.thickness() - offset;
						var axis = zone.axis();
						var start = zone.point(0, 0, depth).subtract(axis);
						var end = zone.point(1, 0, depth).subtract(axis);
						var measuredAngle = Math.abs(Math.atan2(start.cross(end).dot(zone.frame.plane.yDirection), start.dot(end)));
						near(measuredAngle * start.length(), sheet.bendAllowance(b), "neutral length measured from strip geometry");
						near(end.length(), start.length(), "neutral radius is constant along strip");
					}
				}
				// Far strip surface meets the child's attachment at both endpoints, with no hidden gap.
				var childEdge = b.edgeB.span;
				var child = evaluated.wall(b.wallB);
				vector(zone.point(1), child.plane.toWorld(new Vector(childEdge.end.x, childEdge.end.y, 0)), "child far edge end");
				vector(zone.point(1, zone.width), child.plane.toWorld(new Vector(childEdge.start.x, childEdge.start.y, 0)), "child far edge start");
				vector(zone.point(1, 0, sheet.thickness()), child.plane.toWorld(new Vector(childEdge.end.x, childEdge.end.y, -sheet.thickness())), "child side B tangent");
			}
			// Re-rooting traverses inverse bend transforms. Only the world placement changes.
			var last = sheet.walls()[sheet.walls().length - 1].id;
			var reversed = sheet.evaluate(state, last);
			var rebase = evaluated.wall(last).inverse();
			for (w in sheet.walls()) frame(reversed.wall(w.id), rebase.compose(evaluated.wall(w.id)), "reverse-root frame");
		}
		var folded = sheet.evaluate(sheet.folded());
		var unfolded = sheet.evaluate(sheet.flat());
		for (w in sheet.walls()) frame(unfolded.wall(w.id), flat.wall(w.id), "fold-unfold return");
		var b = sheet.bends()[0];
		var oldId = b.id;
		var edit = sheet.updateBend(b.id, b.angle * 0.75, b.radius * 1.2);
		near(edit.id, oldId, "bend id survives angle and radius edits");
		var updated = sheet.evaluate(sheet.folded());
		var restored = sheet.updateBend(b.id, b.angle, b.radius);
		var again = sheet.evaluate(sheet.folded());
		for (w in sheet.walls()) frame(again.wall(w.id), folded.wall(w.id), "restored edit frame");
	}
	static function materialSeams():Void {
		var sheet = SheetDemoParts.p1(); var bend = sheet.bends()[0];
		var allowance = sheet.bendAllowance(bend);
		var hole = new WallOutline([Segment.arc(new Point2(allowance / 2, 50), allowance / 4, 0, 2 * Math.PI)]);
		var cut = sheet.cutBend(bend.id, hole, "interior-hole");
		near(cut.materialRegions(allowance)[0].area(), allowance * 100 - Math.PI * allowance * allowance / 16, "developed circular bend hole area");
		var evaluated = sheet.evaluate(sheet.folded());
		near(evaluated.bend(bend.id).allowance, allowance, "cut keeps neutral length");
		var tools = cut.cutTools(); tools.pop();
		near(cut.cutTools().length, 1, "bend owns cut array");
		var split = SheetDemoParts.p1(); var original = split.bends()[0];
		var slot = WallOutline.polygon([new Point2(-1, 40), new Point2(allowance + 1, 40),
			new Point2(allowance + 1, 60), new Point2(-1, 60)]);
		var trimmed = split.cutBend(original.id, slot, "cross-slot");
		var regions = trimmed.materialRegions(allowance);
		near(regions.length, 2, "bend material may have connected patches joined by walls");
		near(regions[0].area() + regions[1].area(), allowance * 80, "split strip exact area");
		failure("sheet.cut-seam-mismatch", function() { var value = split.evaluate(split.flat()); });
		for (wallId in [original.wallA, original.wallB]) {
			var wall = split.wall(wallId); var span = wallId == original.wallA ? original.edgeA.span : original.edgeB.span;
			var x = span.start.x;
			var tool = WallOutline.polygon([new Point2(x - 1, 40), new Point2(x + 1, 40),
				new Point2(x + 1, 60), new Point2(x - 1, 60)]);
			var outline = OutlineDifference.subtract(wall.outline, tool, "cross-slot");
			var changed = split.updateWallMaterial(wallId, outline);
			near(BendSeam.intervals(outline, span, true).length, 2, "interrupted nominal seam intervals");
		}
		var flat = split.evaluate(split.flat());
		near(split.bend(original.id).edgeA.span.length(), 100, "material cut retains bend datum");
		frame(flat.wall(original.wallB), SheetDemoParts.p1().evaluate(SheetDemoParts.p1().flat()).wall(original.wallB), "material cuts keep child transform");
		var copy = split.copy(); var removed = copy.cutBend(original.id, WallOutline.rectangle(allowance, 100), "remove-strip");
		failure("sheet.cut-seam-disconnected", function() { var value = copy.evaluate(copy.flat()); });
		near(split.bend(original.id).cutTools().length, 1, "copy isolates bend edits");
	}
	static function closure():Void {
		var sheet = new Sheet(SheetDemoParts.materialA());
		var walls = [for (i in 0...4) sheet.addWall(WallOutline.rectangle(10, 3))];
		var bends = [for (i in 0...4) sheet.addBend(walls[i].id, new WallEdge(1), walls[(i + 1) % 4].id,
			new WallEdge(3), Math.PI / 2, 1)];
		var closed = sheet.evaluate(sheet.folded());
		var changed = sheet.updateWall(walls[1].id, WallOutline.rectangle(11, 3));
		near(changed.id, walls[1].id, "wall identity survives outline edit");
		failure("sheet.closure-gap", function() { var value = sheet.evaluate(sheet.folded()); });
		var restore = sheet.updateWall(walls[1].id, walls[1].outline);
		var rip = sheet.rip(bends[3].id);
		var flat = sheet.evaluate(sheet.flat());
		near(flat.bend(bends[3].id).allowance, sheet.bendAllowance(bends[3]), "rip retains the bend strip's material");
		for (w in sheet.walls()) near(flat.wall(w.id).plane.origin.z, 0, "ripped ring unfolds");
		failure("sheet.already-ripped", function() { var value = sheet.rip(bends[3].id); });
		var second = sheet.rip(bends[1].id);
		failure("sheet.disconnected", function() { var value = sheet.evaluate(sheet.flat()); });
	}
	static function diagnoses():Void {
		var sheet = SheetDemoParts.p1();
		var extra = sheet.addWall(WallOutline.rectangle(5, 5));
		near(extra.id, 3, "monotone wall allocation");
		failure("sheet.disconnected", function() { var value = sheet.evaluate(sheet.flat()); });
		failure("sheet.bend-edge-mismatch", function() { var value = sheet.addBend(1, new WallEdge(0), 3, new WallEdge(3), 1, 1); });
		var part = SheetDemoParts.p1();
		failure("sheet.bend-edge-in-use", function() { var value = part.addBend(1, new WallEdge(1), 2, new WallEdge(3), 1, 1); });
		failure("sheet.invalid-state", function() { var value = part.evaluate(new SheetState()); });
		failure("sheet.invalid-state", function() { var value = part.evaluate(new SheetState().set(1, -0.1)); });
		failure("sheet.invalid-state", function() { var value = part.evaluate(new SheetState().set(1, 2)); });
		failure("sheet.missing-bend", function() { var value = part.evaluate(part.flat().set(999, 0)); });
		var previous = part.wall(2);
		failure("sheet.bend-edge-mismatch", function() { var value = part.updateWall(2, WallOutline.rectangle(10, 99)); });
		near(part.wall(2).outline.area(), previous.outline.area(), "failed edit keeps graph");
		var hole = [Segment.arc(new Point2(10, 10), 2, 0, -2 * Math.PI)];
		var cut = part.cutHole(1, hole);
		near(cut.outline.area(), 5750 - 4 * Math.PI, "wall cut is graph data");
		near(cut.id, 1, "hole cut preserves wall identity");
		var absent = new Sheet(new SheetMaterial("unknown", 1));
		var base = absent.addWall(WallOutline.rectangle(2, 2));
		failure("sheet.no-bend-rule", function() { var value = absent.evaluate(absent.flat()); });
		var circular = new Sheet(SheetDemoParts.materialA());
		var round = circular.addWall(new WallOutline([Segment.arc(new Point2(0, 0), 3, 0, 2 * Math.PI)]));
		var other = circular.addWall(WallOutline.rectangle(4, 3));
		failure("sheet.bend-edge-mismatch", function() { var value = circular.addBend(round.id, new WallEdge(0), other.id, new WallEdge(3), 1, 1); });
	}
}
