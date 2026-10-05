package machinekit.pressbrake.planning;

import cadkit.modeling.Vector;

/** Interim exact separating-axis test for convex prisms; no RobotKit or collision-branch dependency. */
class PrismSatQuery implements SheetCollisionQuery {
	public final penetrationTolerance:Float;
	public function new(penetrationTolerance:Float = 1e-8) {
		if (!Math.isFinite(penetrationTolerance) || penetrationTolerance < 0)
			throw "Collision penetration tolerance must be finite nonnegative mm";
		this.penetrationTolerance = penetrationTolerance;
	}
	public function overlap(a:ConvexPrism, b:ConvexPrism):Bool {
		var pointsA = a.vertices(), pointsB = b.vertices(), origin = pointsA[0];
		for (axis in a.faceNormals()) if (separated(pointsA, pointsB, origin, axis)) return false;
		for (axis in b.faceNormals()) if (separated(pointsA, pointsB, origin, axis)) return false;
		for (edgeA in a.edgeDirections()) for (edgeB in b.edgeDirections()) {
			var cross = edgeA.cross(edgeB), length = cross.length();
			// Only an exactly parallel pair has no axis; nearly parallel directions still matter.
			if (length > 0 && separated(pointsA, pointsB, origin, cross.scale(1 / length))) return false;
		}
		return true;
	}
	function separated(a:Array<Vector>, b:Array<Vector>, origin:Vector, axis:Vector):Bool {
		var lowA = Math.POSITIVE_INFINITY, highA = Math.NEGATIVE_INFINITY;
		var lowB = Math.POSITIVE_INFINITY, highB = Math.NEGATIVE_INFINITY;
		for (point in a) {
			var distance = point.subtract(origin).dot(axis);
			lowA = Math.min(lowA, distance); highA = Math.max(highA, distance);
		}
		for (point in b) {
			var distance = point.subtract(origin).dot(axis);
			lowB = Math.min(lowB, distance); highB = Math.max(highB, distance);
		}
		return highA <= lowB + penetrationTolerance || highB <= lowA + penetrationTolerance;
	}
}
