package machinekit.pressbrake.planning;

import cadkit.modeling.Vector;
import cadkit.sheetmetal.SheetDiagnosis;
import collisionkit.CollisionDistance;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPair;
import collisionkit.CollisionPose;
import collisionkit.native.NativeCollisionWorld;

/** Convex material queries through CollisionKit, in press-frame mm. */
class CoalPrismQuery implements SheetCollisionQuery {
	// Coal's absolute EPA tolerance is 1e-6 native units. Micrometres keep it
	// below the declared mm penetration tolerance without lowering its solver tolerance.
	static inline var NATIVE_SCALE:Float = 1000;
	public final penetrationTolerance:Float;
	public function new(penetrationTolerance:Float = 1e-8) {
		if (!Math.isFinite(penetrationTolerance) || penetrationTolerance < 0)
			throw "Collision penetration tolerance must be finite nonnegative mm";
		this.penetrationTolerance = penetrationTolerance;
	}

	public function overlap(a:ConvexPrism, b:ConvexPrism):Bool
		return separation(a, b) < -penetrationTolerance;

	/** Positive support gaps certify clearance independently of the solver's distance estimate. */
	public function separation(a:ConvexPrism, b:ConvexPrism):Float {
		var sampled = query(a, b), distance = sampled.distance / NATIVE_SCALE;
		if (distance <= 0) return distance;
		var direction = new Vector(sampled.normal[0], sampled.normal[1], sampled.normal[2]);
		var length = direction.length();
		if (!Math.isFinite(length) || length <= 0)
			throw new SheetDiagnosis("press.collision-query", "CollisionKit returned no separating direction", [a.id, b.id]);
		var normal = direction.scale(1 / length), pointsA = a.vertices(), pointsB = b.vertices(), origin = pointsA[0];
		var lowA = Math.POSITIVE_INFINITY, highA = Math.NEGATIVE_INFINITY;
		var lowB = Math.POSITIVE_INFINITY, highB = Math.NEGATIVE_INFINITY;
		for (point in pointsA) {
			var projection = point.subtract(origin).dot(normal);
			lowA = Math.min(lowA, projection); highA = Math.max(highA, projection);
		}
		for (point in pointsB) {
			var projection = point.subtract(origin).dot(normal);
			lowB = Math.min(lowB, projection); highB = Math.max(highB, projection);
		}
		// An unconverged direction can fail to prove clearance. Request refinement
		// rather than using the estimated witness-point distance as a lower bound.
		return Math.max(0, Math.max(lowB - highA, lowA - highB));
	}

	function query(a:ConvexPrism, b:ConvexPrism):CollisionDistance {
		var pointsA = a.vertices(), pointsB = b.vertices(), origin = pointsA[0];
		// Cancel a common translation before the native query, retaining small physical gaps.
		var world = new NativeCollisionWorld();
		var sampled:CollisionDistance;
		try {
			var bodyA = world.addBody(0), bodyB = world.addBody(0);
			var objectA = world.add(bodyA, CollisionPose.identity(), CollisionGeometry.Convex(points(pointsA, origin)));
			var objectB = world.add(bodyB, CollisionPose.identity(), CollisionGeometry.Convex(points(pointsB, origin)));
			var measured = world.pairDistances([new CollisionPair(objectA, objectB, bodyA, bodyB)]);
			if (measured.length != 1 || !Math.isFinite(measured[0].distance))
				throw "CollisionKit did not return one finite convex distance";
			sampled = measured[0];
			// Coal's hard solver fallback has a finite negative scalar but NaN witnesses.
			for (values in [sampled.pointA, sampled.pointB, sampled.normal]) {
				if (values.length != 3) throw "CollisionKit returned an incomplete convex witness";
				for (value in values) if (!Math.isFinite(value)) throw "CollisionKit returned a nonfinite convex witness";
			}
		} catch (failure:Dynamic) {
			world.dispose();
			throw new SheetDiagnosis("press.collision-query", "CollisionKit convex query failed: " + Std.string(failure), [a.id, b.id]);
		}
		world.dispose();
		return sampled;
	}

	static function points(vertices:Array<Vector>, origin:Vector):Array<Float> {
		var result:Array<Float> = [];
		for (vertex in vertices) {
			var point = vertex.subtract(origin);
			result.push(point.x * NATIVE_SCALE); result.push(point.y * NATIVE_SCALE); result.push(point.z * NATIVE_SCALE);
		}
		return result;
	}
}
