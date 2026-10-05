package machinekit.pressbrake.planning;

import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.Bend;
import cadkit.sheetmetal.BendZone;
import cadkit.sheetmetal.SheetDiagnosis;

/** Exact uncut bend material against a convex obstacle, for any bend-axis direction. */
class PressBendPrismQuery {
	final sections:PressSectionQuery;
	public function new() { sections = new PressSectionQuery(); }

	/** Null requires a query retaining the cut's along-axis variation. The obstacle must be exact material. */
	public function overlap(record:Bend, zone:BendZone, placement:Location, obstacle:ConvexPrism):Null<Bool> {
		if (record.id != zone.id)
			throw new SheetDiagnosis("press.bend-query", "Bend record and evaluated zone must have the same identity");
		if (record.hasCuts()) return null;
		// Put the bend axis on X. Its material cap is constant along that axis, so projecting
		// the obstacle after clipping to the finite extrusion preserves existence of overlap.
		var axes = new Location(new Plane(new Vector(), new Vector(0, 1, 0), new Vector(0, 0, -1)));
		var canonical = axes.compose(zone.frame.inverse());
		var mapped = obstacle.placed(canonical.compose(placement.inverse()));
		var obstacleSection = PressPrismSection.of(mapped, 0, zone.width);
		if (obstacleSection == null) return false;
		var material = PressBendSection.of(record, zone, canonical, 0, zone.width);
		if (material == null)
			throw new SheetDiagnosis("press.bend-query", "Canonical bend axis did not align with the extrusion", ["bend:" + record.id]);
		for (section in material) if (sections.overlap(section, obstacleSection)) return true;
		return false;
	}
}
