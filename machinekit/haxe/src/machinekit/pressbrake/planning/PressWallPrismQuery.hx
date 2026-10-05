package machinekit.pressbrake.planning;

import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.WallOutline;

/** Exact line/arc wall extrusion against a convex obstacle at any relative orientation. */
class PressWallPrismQuery {
	final sections:PressSectionQuery;
	public function new() { sections = new PressSectionQuery(); }
	public function overlap(outline:WallOutline, thickness:Float, placement:Location, obstacle:ConvexPrism):Bool {
		if (!Math.isFinite(thickness) || thickness <= 0)
			throw new SheetDiagnosis("press.wall-query", "Wall material needs finite positive thickness");
		// Wall-local X/Y become canonical Y/Z without changing the exact outline. Its
		// normal becomes X; clip the obstacle to the physical [-thickness, 0] extrusion.
		var axes = new Location(new Plane(new Vector(), new Vector(0, 1, 0), new Vector(1, 0, 0)));
		var mapped = obstacle.placed(axes.compose(placement.inverse()));
		var section = PressPrismSection.of(mapped, -thickness, 0);
		return section != null && sections.overlap(outline, section);
	}
}
