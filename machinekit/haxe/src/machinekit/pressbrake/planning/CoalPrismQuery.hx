package machinekit.pressbrake.planning;

import cadkit.modeling.Vector;
import cadkit.sheetmetal.SheetDiagnosis;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPair;
import collisionkit.CollisionPose;
import collisionkit.native.NativeCollisionWorld;

/** Convex material queries through CollisionKit, in press-frame mm. */
class CoalPrismQuery implements SheetCollisionQuery {
	public final penetrationTolerance:Float;
	public function new(penetrationTolerance:Float = 1e-8) {
		if (!Math.isFinite(penetrationTolerance) || penetrationTolerance < 0)
			throw "Collision penetration tolerance must be finite nonnegative mm";
		this.penetrationTolerance = penetrationTolerance;
	}

	public function overlap(a:ConvexPrism, b:ConvexPrism):Bool
		return separation(a, b) < -penetrationTolerance;

	/** Signed convex distance; positive separation certifies clearance before curved-material refinement. */
	public function separation(a:ConvexPrism, b:ConvexPrism):Float {
		var pointsA = a.vertices(), pointsB = b.vertices(), origin = pointsA[0];
		// Cancel a common translation before the native query, retaining small physical gaps.
		var world = new NativeCollisionWorld();
		var distance:Float;
		try {
			var bodyA = world.addBody(0), bodyB = world.addBody(0);
			var objectA = world.add(bodyA, CollisionPose.identity(), CollisionGeometry.Convex(points(pointsA, origin)));
			var objectB = world.add(bodyB, CollisionPose.identity(), CollisionGeometry.Convex(points(pointsB, origin)));
			var measured = world.pairDistances([new CollisionPair(objectA, objectB, bodyA, bodyB)]);
			if (measured.length != 1 || !Math.isFinite(measured[0].distance))
				throw new SheetDiagnosis("press.collision-query", "CollisionKit did not return one finite convex distance", [a.id, b.id]);
			distance = measured[0].distance;
		} catch (failure:Dynamic) {
			world.dispose();
			throw failure;
		}
		world.dispose();
		return distance;
	}

	static function points(vertices:Array<Vector>, origin:Vector):Array<Float> {
		var result:Array<Float> = [];
		for (vertex in vertices) {
			var point = vertex.subtract(origin);
			result.push(point.x); result.push(point.y); result.push(point.z);
		}
		return result;
	}
}
