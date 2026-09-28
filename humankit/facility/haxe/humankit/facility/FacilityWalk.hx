package humankit.facility;

import materia.automation.facility.FacilityRoute;
import robotkit.navigation.Path;

/**
 * Bridges AutomationKit's facility routing to HumanWalker. HumanKit itself
 * does not depend on RobotKit or AutomationKit; this small package is the
 * one place that knows about both, so a person can walk a FacilityRoute (or
 * any robotkit Path) planned above it.
 */
class FacilityWalk {
	/**
	 * Floor points ([x, y], metres) along path's waypoints in order, for
	 * HumanWalker.follow. Consecutive waypoints closer than a millimetre are
	 * collapsed, so a route never carries a zero-length final segment that
	 * would leave the walker's ending heading undefined.
	 */
	public static function routeFromPath(path:Path):Array<Array<Float>> {
		var points:Array<Array<Float>> = [];
		for (pose in path.poses()) {
			var point = [pose.x, pose.y];
			if (points.length == 0 || distance(points[points.length - 1], point) > 1e-3)
				points.push(point);
		}
		if (points.length < 2)
			throw "A facility path needs at least two distinct points to walk";
		return points;
	}

	/** Floor points along a facility route's path, from its origin to its destination station. */
	public static function routeFromFacilityRoute(route:FacilityRoute):Array<Array<Float>>
		return routeFromPath(route.path);

	static function distance(a:Array<Float>, b:Array<Float>):Float {
		var dx = a[0] - b[0], dy = a[1] - b[1];
		return Math.sqrt(dx * dx + dy * dy);
	}
}
