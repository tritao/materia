package humankit.action;

import humankit.HumanBody;

/** Walks to one floor point, or along an explicit sequence of floor points. */
class WalkTo extends HumanActionBase {
	public final speed:Float;
	final point:Null<Array<Float>>;
	final path:Null<Array<Array<Float>>>;
	final pointProvider:Null<Void->Array<Float>>;
	final preserveFacing:Bool;
	/** The route to start once the body has stood up from a crouch; null when walking or not yet planned. */
	var waiting:Null<Array<Array<Float>>> = null;

	/** Resolve a destination from the worker's pose when this action starts. */
	public static function deferred(pointProvider:Void->Array<Float>, speed:Float,
		preserveFacing:Bool = false):WalkTo
		return new WalkTo(null, speed, null, pointProvider, preserveFacing);

	public static function along(path:Array<Array<Float>>, speed:Float,
		?destination:Void->Array<Float>):WalkTo
		return new WalkTo(null, speed, path, destination);

	public function new(point:Null<Array<Float>>, speed:Float, ?path:Array<Array<Float>>,
		?pointProvider:Void->Array<Float>, preserveFacing:Bool = false) {
		super();
		this.point = point == null ? null : point.copy();
		this.path = path == null ? null : [for (p in path) p.copy()];
		this.speed = speed;
		this.pointProvider = pointProvider;
		this.preserveFacing = preserveFacing;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (!(speed > 0.0)) { fail("Walking speed must be positive"); return; }
		var root = worker.rootTransform();
		var start = [root[12], root[13]];
		var route:Array<Array<Float>>;
		var pathValue = path;
		if (pathValue != null) {
			if (pathValue.length == 0) { fail("Walking route is empty"); return; }
			route = [start];
			for (p in pathValue) {
				if (p == null || p.length < 2 || !Math.isFinite(p[0]) || !Math.isFinite(p[1])) {
					fail("Walking point needs finite x and y"); return;
				}
				if (Math.sqrt(Math.pow(p[0] - route[route.length - 1][0], 2)
					+ Math.pow(p[1] - route[route.length - 1][1], 2)) > 1e-5)
					route.push([p[0], p[1]]);
			}
			if (pointProvider != null) {
				var destination = pointProvider();
				if (destination == null || destination.length < 2 || !Math.isFinite(destination[0]) ||
					!Math.isFinite(destination[1])) { fail("Walking point needs finite x and y"); return; }
				if (Math.sqrt(Math.pow(destination[0] - route[route.length - 1][0], 2) +
					Math.pow(destination[1] - route[route.length - 1][1], 2)) > 1e-5)
					route.push([destination[0], destination[1]]);
			}
		} else {
			var goal = pointProvider == null ? point : pointProvider();
			if (goal == null || goal.length < 2 || !Math.isFinite(goal[0]) ||
				!Math.isFinite(goal[1])) { fail("Walking point needs finite x and y"); return; }
			route = [start, [goal[0], goal[1]]];
		}
		if (route.length < 2) {
			done = true;
			return;
		}
		// A crouched body does not walk: it stands up first, and the walk starts when it has.
		if (worker.downAmount() > 1e-3 || !worker.downReached()) {
			worker.setCrouch(0.0);
			worker.setKneel(0.0);
			waiting = route;
			return;
		}
		begin(route);
	}

	function begin(route:Array<Array<Float>>):Void {
		waiting = null;
		if (preserveFacing) worker.walker.retreatAlong(route, speed);
		else worker.walker.continueAlong(route, speed);
	}

	override public function advance(seconds:Float):Void {
		var route = waiting;
		if (route != null && worker.downAmount() <= 1e-3) begin(route);
	}

	override public function isDone():Bool
		return done || (worker != null && waiting == null && !worker.walker.isWalking());
}
