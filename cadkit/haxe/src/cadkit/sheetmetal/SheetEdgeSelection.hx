package cadkit.sheetmetal;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.parametric.TopologyFingerprint;

/** Identifies the exact graph segment behind a resolved straight wall-side face. Names remain opaque. */
class SheetEdgeSelection {
	public static function identify(sheet:Sheet, face:Shape):SheetEdge {
		if (face.kind() != CadKit.ShapeKind.Face || face.surfaceKind() != CadKit.SurfaceKind.Plane)
			throw new SheetDiagnosis("sheet.flange-edge-selection", "A flange requires a straight wall-side face");
		var selected = TopologyFingerprint.withoutName(TopologyFingerprint.capture(face));
		var evaluated = sheet.evaluate(sheet.folded());
		var candidates:Array<SheetEdge> = [];
		for (wall in sheet.walls()) {
			var placement = evaluated.wall(wall.id);
			var loops = wall.outline.loops();
			for (loop in 0...loops.length)
				for (index in 0...loops[loop].length) {
					var segment = loops[loop][index];
					if (!segment.isLine()) continue;
					var midpoint = segment.point(0.5);
					var center = placement.plane.toWorld(new Vector(midpoint.x, midpoint.y, -sheet.thickness() / 2));
					var normal = placement.direction(new Vector(segment.end.y - segment.start.y,
						segment.start.x - segment.end.x, 0).normalized());
					var expected = TopologyFingerprint.fromData(CadKit.ShapeKind.Face, CadKit.SurfaceKind.Plane,
						CadKit.CurveKind.Unknown, center.x, center.y, center.z, normal.x, normal.y, normal.z,
						segment.length() * sheet.thickness(), false);
					if (selected.scoreAgainst(expected) >= 0.5)
						candidates.push(new SheetEdge(wall.id, new WallEdge(index, loop)));
				}
		}
		if (candidates.length != 1)
			throw new SheetDiagnosis("sheet.flange-edge-selection", "Resolved face matches " + candidates.length
				+ " straight graph segments; a flange requires exactly one");
		return candidates[0];
	}
}

/** A wall identity and its current outline segment, derived from a resolved persistent face reference. */
class SheetEdge {
	public final wall:Int;
	public final edge:WallEdge;
	public function new(wall:Int, edge:WallEdge) {
		this.wall = wall;
		this.edge = edge;
	}
}
