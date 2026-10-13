package cadkit.sheetmetal;

import cadkit.modeling.Vector;
import cadkit.sheetmetal.FlatDomain.FlatOwner;
import cadkit.sheetmetal.process.AirBending;
import cadkit.sheetmetal.process.ProcessTooling;

/** Pure manufacturability checks retain exact offending geometry and explicit assumed rules. */
class SheetChecks {
	static function interior(flat:FlatPattern, point:Point2):Bool {
		for (region in flat.regions()) if (region.contains(point)) {
			for (loop in region.loops()) for (edge in loop) if (SegmentDistance.point(edge, point) <= 1e-8) return false;
			return true;
		}
		return false;
	}
	public static function of(sheet:Sheet, ?tooling:ProcessTooling):SheetCheckReport {
		var flat = FlatPattern.of(sheet); var findings:Array<SheetFinding> = []; var domains = flat.domains();
		for (i in 0...domains.length) for (j in (i + 1)...domains.length) {
			var regions = OutlineBoolean.intersection(domains[i].outline, domains[j].outline, "overlap");
			for (region in regions) if (region.area() > 1e-8) findings.push(new SheetFinding(FlatOverlap,
				"Developed material overlaps between " + domains[i].identity() + " and " + domains[j].identity(), false, AreaMm2(region.area()),
				[Material(domains[i].owner, region), Material(domains[j].owner, region)]));
		}
		var evaluated = sheet.evaluate(sheet.flat());
		for (bend in sheet.bends()) {
			var zone = evaluated.bend(bend.id); var marks:Array<SheetCheckGeometry> = [];
			var start = flat.plane.toLocal(zone.frame.plane.toWorld(new Vector(zone.allowance / 2, 0)));
			var end = flat.plane.toLocal(zone.frame.plane.toWorld(new Vector(zone.allowance / 2, zone.width)));
			var guide = Segment.line(new Point2(start.x, start.y), new Point2(end.x, end.y)); marks.push(BendMark(bend.id, guide));
			var minimumRadius = sheet.material.minRadiusRatio;
			if (minimumRadius != null && bend.radius < minimumRadius * sheet.thickness() - 1e-8)
				findings.push(new SheetFinding(SmallRadius, "Bend " + bend.id + " has an inner radius below the material minimum", false,
					DistanceMm(bend.radius, minimumRadius * sheet.thickness()), marks));
			var grainLimit = Math.PI / 12;
			if (flat.material.grainAngle != null) {
				var angle = Math.acos(Math.min(1, Math.abs(guide.end.x - guide.start.x) / guide.length()));
				if (angle <= grainLimit + 1e-10) findings.push(new SheetFinding(AlongGrain,
					"Bend " + bend.id + " lies within 15 degrees of the rolling direction", false, AngleRadians(angle, grainLimit), marks));
			}
			for (point in [guide.start, guide.end]) if (interior(flat, point)) findings.push(new SheetFinding(MissingRelief,
				"Bend " + bend.id + " ends at (" + point.x + ", " + point.y + ") inside developed material without an opening", false, InteriorPoint(point), [BendMark(bend.id, guide), BendEnd(bend.id, point)]));
			var opening = tooling != null ? tooling.vWidth : switch (sheet.material.bendRule) {
				case Process(setup): setup.tooling.vWidth;
				case _: AirBending.defaultOpening(sheet.thickness());
			};
			var minimum = AirBending.minimumFlange(opening);
			for (side in 0...2) {
				if (side == 1 && sheet.isRipped(bend.id)) {
					var geometry = marks.copy();
					for (domain in domains) switch (domain.owner) { case Bend(id): if (id == bend.id) geometry.push(Material(domain.owner, domain.outline)); case _: {} }
					findings.push(new SheetFinding(ShortFlange, "Ripped bend " + bend.id + " has no straight return beyond its curved strip for the assumed 0.7V tooling minimum", true,
						DistanceMm(0, minimum), geometry));
					continue;
				}
				var wall = sheet.wall(side == 0 ? bend.wallA : bend.wallB); var attachment = side == 0 ? bend.edgeA : bend.edgeB;
				var extent = FlangeDepth.minimum(wall.outline, attachment);
				if (extent < minimum - 1e-8) {
					var mapped = SheetProfile.transform(wall.outline, evaluated.wall(wall.id).plane, flat.plane);
					findings.push(new SheetFinding(ShortFlange, "Wall " + wall.id + " at bend " + bend.id + " is shorter than the assumed 0.7V tooling minimum", true,
						DistanceMm(extent, minimum), [Material(FlatOwner.Wall(wall.id), mapped), BendMark(bend.id, guide)]));
				}
			}
			var clearance = 2.5 * sheet.thickness() + bend.radius;
			for (region in flat.regions()) {
				var loops = region.loops();
				for (i in 0...loops.length) for (j in 0...loops[i].length) {
					var origin = region.origin(i, j); var candidate = switch (origin) { case Cut(_): true; case Relief(_): false; case Authored: i > 0; };
					if (!candidate) continue;
					var distance = SegmentDistance.toLine(loops[i][j], guide);
					if (distance < clearance - 1e-8) findings.push(new SheetFinding(CutNearBend,
						"Cut boundary is closer to bend " + bend.id + " than the assumed 2.5T + R clearance", true,
						DistanceMm(distance, clearance), [Boundary(region.identity(i, j), origin, loops[i][j]), BendMark(bend.id, guide)]));
				}
			}
		}
		return new SheetCheckReport(flat, findings);
	}
}
