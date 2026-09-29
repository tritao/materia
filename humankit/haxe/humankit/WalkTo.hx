package humankit;

/** Walks to one floor point, or along an explicit sequence of floor points. */
class WalkTo extends HumanActionBase {
	public final speed:Float;
	final point:Null<Array<Float>>;
	final path:Null<Array<Array<Float>>>;

	public static function along(path:Array<Array<Float>>, speed:Float):WalkTo
		return new WalkTo(null, speed, path);

	public function new(point:Null<Array<Float>>, speed:Float, ?path:Array<Array<Float>>) {
		super();
		this.point = point == null ? null : point.copy();
		this.path = path == null ? null : [for (p in path) p.copy()];
		this.speed = speed;
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
			for (p in pathValue)
				if (p.length < 2 || Math.sqrt(Math.pow(p[0] - route[route.length - 1][0], 2)
					+ Math.pow(p[1] - route[route.length - 1][1], 2)) > 1e-5)
					route.push([p[0], p[1]]);
		} else {
			var goal = point;
			if (goal == null || goal.length < 2) { fail("Walking point needs x and y"); return; }
			route = [start, [goal[0], goal[1]]];
		}
		if (route.length < 2) {
			done = true;
			return;
		}
		worker.walker.follow(route, speed);
	}

	override public function isDone():Bool
		return done || (worker != null && !worker.walker.isWalking());
}
