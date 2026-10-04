package cadkit.sheetmetal;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.FlatDomain.FlatOwner;
import cadkit.sheetmetal.BendSeam.SeamInterval;

/** A fresh exact developed blank. Its X axis follows material grain when grain is authored. */
class FlatPattern {
	public final material:SheetMaterial;
	public final plane:Plane;
	final domainData:Array<FlatDomain>;
	final regionData:Array<WallOutline>;
	final bendData:Array<FlatBend>;
	function new(material:SheetMaterial, plane:Plane, domains:Array<FlatDomain>, regions:Array<WallOutline>, bends:Array<FlatBend>) {
		this.material = material; this.plane = plane; domainData = domains.copy(); regionData = regions.copy(); bendData = bends.copy();
	}
	public function domains():Array<FlatDomain> { return domainData.copy(); }
	public function regions():Array<WallOutline> { return regionData.copy(); }
	public function bends():Array<FlatBend> { return bendData.copy(); }
	public function area():Float { var result = 0.0; for (region in regionData) result += region.area(); return result; }
	public function bounds():SheetBounds {
		var result = new SheetBounds();
		for (region in regionData) for (segment in region.loops()[0]) for (point in segment.extrema()) result.include(new Vector(point.x, point.y));
		return result;
	}
	static function identified(outline:WallOutline, owner:String):WallOutline {
		var identities = outline.identities();
		for (i in 0...identities.length) for (j in 0...identities[i].length) identities[i][j] = owner + ":edge:" + identities[i][j];
		var loops = outline.loops(); return new WallOutline(loops[0], loops.slice(1), identities, outline.origins());
	}
	public static function of(sheet:Sheet):FlatPattern {
		var evaluated = sheet.evaluate(sheet.flat()); var plane = sheet.placement.plane; var grain = sheet.material.grainAngle;
		if (grain != null) plane = new Plane(plane.origin, plane.xDirection.scale(Math.cos(grain)).add(plane.yDirection.scale(Math.sin(grain))), plane.normal);
		var domains:Array<FlatDomain> = []; var bends:Array<FlatBend> = [];
		for (wall in sheet.walls()) domains.push(new FlatDomain(FlatOwner.Wall(wall.id),
			identified(SheetProfile.transform(wall.outline, evaluated.wall(wall.id).plane, plane), "wall:" + wall.id)));
		for (bend in sheet.bends()) {
			var zone = evaluated.bend(bend.id); var material = bend.materialRegions(zone.allowance);
			for (region in material) domains.push(new FlatDomain(FlatOwner.Bend(bend.id),
				identified(SheetProfile.transform(region, zone.frame.plane, plane), "bend:" + bend.id)));
			var guide = Segment.line(new Point2(zone.allowance / 2, 0), new Point2(zone.allowance / 2, zone.width));
			var intervals:Array<SeamInterval> = [];
			for (region in material) for (interval in BendSeam.materialIntervals(region, guide)) intervals.push(interval);
			var lines:Array<Segment> = [];
			for (interval in BendSeam.union(intervals)) {
				var a = guide.point(interval.first); var b = guide.point(interval.last);
				var start = plane.toLocal(zone.frame.plane.toWorld(new Vector(a.x, a.y))); var end = plane.toLocal(zone.frame.plane.toWorld(new Vector(b.x, b.y)));
				lines.push(Segment.line(new Point2(start.x, start.y), new Point2(end.x, end.y)));
			}
			bends.push(new FlatBend(bend.id, bend.angle, bend.radius, zone.allowance, zone.width, sheet.isRipped(bend.id), lines));
		}
		bends.sort(function(a:FlatBend, b:FlatBend):Int { return a.id < b.id ? -1 : a.id > b.id ? 1 : 0; });
		var regions = OutlineBoolean.unionMany([for (domain in domains) domain.outline]);
		return new FlatPattern(sheet.material, plane, domains, regions, bends);
	}
}
