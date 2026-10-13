package machinekit.pressbrake.planning;

import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;

/** Physical material checks retain approximation uncertainty instead of guessing contact. */
class PressMaterialQuery {
	final prisms:SheetCollisionQuery;
	final bends:PressBendPrismQuery;
	final walls:PressWallPrismQuery;
	public function new(prisms:SheetCollisionQuery) {
		if (prisms == null) throw new SheetDiagnosis("press.material-query", "Material collision checks need a prism query");
		this.prisms = prisms; bends = new PressBendPrismQuery(); walls = new PressWallPrismQuery();
	}
	/** False proves clearance, true proves material overlap, null requests geometric refinement. */
	public function overlap(sheet:Sheet, pose:PressSheetPose, material:PressCollisionBody, obstacle:PressCollisionBody):Null<Bool> {
		switch material.owner {
			case SheetWall(id):
				if (obstacle.error == 0) {
					for (piece in obstacle.pieces())
						if (walls.overlap(sheet.wall(id).outline, sheet.thickness(), pose.wall(id), piece)) return true;
					return false;
				}
			case SheetBend(id):
				if (obstacle.error == 0 && !sheet.bend(id).hasCuts()) {
					for (piece in obstacle.pieces()) {
						var overlap = bends.overlap(sheet.bend(id), pose.evaluation.bend(id), pose.placement, piece);
						if (overlap == null)
							throw new SheetDiagnosis("press.material-query", "Uncut exact bend query unexpectedly requested refinement", ["bend:" + id]);
						if (overlap) return true;
					}
					return false;
				}
			case _: throw new SheetDiagnosis("press.material-query", "Material query requires an evaluated sheet wall or bend");
		}
		// Distance is 1-Lipschitz: moving either approximated surface by its error can
		// close at most the sum of those errors between the two convex pieces.
		var uncertainty = material.error + obstacle.error;
		var unresolved = false;
		for (a in material.pieces()) for (b in obstacle.pieces()) {
			if (prisms.separation(a, b) > uncertainty + 1e-8) continue;
			// The boundary-deformation contract keeps each face-inset piece inside physical
			// material. Artificial decomposition faces only make this certificate smaller.
			var innerA = a.inset(material.error), innerB = b.inset(obstacle.error);
			if (innerA != null && innerB != null && prisms.overlap(innerA, innerB)) return true;
			unresolved = true;
		}
		return unresolved ? null : false;
	}
}
