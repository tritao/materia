package humankit;

/**
 * Walks a HumanCharacter along a route of floor points. The walk clip plays at
 * the rate that matches the body's speed to the clip's gait, so planted feet
 * stay on the floor. The body speeds up over the crossfade from idle, slows
 * down to stop at the end of the route, and turns towards the route at a
 * bounded rate. rootTransform() is where
 * to place the character; it faces +X like every AnimKit character.
 */
class HumanWalker {
	static inline var FADE_SECONDS:Float = 0.3;

	public final character:HumanCharacter;
	public final gait:HumanGait;
	final idleClip:Int;
	/** Fastest turn, in radians per second. */
	public var turnRate:Float = 4.0;

	var points:Array<Array<Float>> = [];
	/** Distance along the route at each point. */
	var distances:Array<Float> = [];
	var loop:Bool = false;
	var speed:Float = 0.0;
	var travelled:Float = 0.0;
	/** The body's current speed, ramping between rest and `speed`. */
	var velocity:Float = 0.0;
	var walking:Bool = false;
	var x:Float = 0.0;
	var y:Float = 0.0;
	var heading:Float = 0.0;
	var facing:Null<Float> = null;

	/** Measures the walk clip's gait and starts the character idling at the origin. */
	public function new(character:HumanCharacter, walkClip:String = "walk", idleClip:String = "idle") {
		this.character = character;
		var walk = character.asset.clipIndex(walkClip);
		var idle = character.asset.clipIndex(idleClip);
		if (walk < 0 || idle < 0)
			throw 'The character needs "$walkClip" and "$idleClip" clips';
		gait = HumanGait.measure(character.asset, character.rig, walk);
		this.idleClip = idle;
		character.player.play(idle, 0.0);
	}

	/**
	 * Walks along points ([x, y] on the floor, metres) at speed, starting at
	 * the first point facing the second. A looping route returns to its start.
	 */
	public function follow(route:Array<Array<Float>>, metresPerSecond:Float, loop:Bool = false):Void {
		if (route.length < 2 || !(metresPerSecond > 0.0))
			throw "A walk needs at least two points and a positive speed";
		points = [for (point in route) [point[0], point[1]]];
		if (loop)
			points.push([route[0][0], route[0][1]]);
		distances = [0.0];
		for (index in 1...points.length)
			distances.push(distances[index - 1] + segmentLength(index - 1));
		if (!(distances[distances.length - 1] > 0.0))
			throw "A walk route needs some length";
		this.loop = loop;
		speed = metresPerSecond;
		travelled = 0.0;
		velocity = 0.0;
		x = points[0][0];
		y = points[0][1];
		heading = tangentAt(0.0);
		walking = true;
		facing = null;
		character.player.play(gait.clip, FADE_SECONDS);
	}

	/**
	 * Walks along route from where the character stands (the route's first
	 * point), keeping its heading and turning onto the route at turnRate rather
	 * than snapping to it, so what it carries does not swing.
	 */
	public function continueAlong(route:Array<Array<Float>>, metresPerSecond:Float):Void {
		var current = heading;
		follow(route, metresPerSecond);
		heading = current;
	}

	/** Sets the starting floor pose before a job begins. */
	public function place(x:Float, y:Float, heading:Float):Void {
		stop();
		this.x = x;
		this.y = y;
		this.heading = wrap(heading);
		facing = null;
	}

	/** Turns in place at turnRate, including after a route has ended. */
	public function face(angle:Float):Void
		facing = wrap(angle);

	public function isTurning():Bool
		return facing != null;

	/** Stops where the character stands and idles. */
	public function stop():Void {
		facing = null;
		if (!walking)
			return;
		walking = false;
		velocity = 0.0;
		character.player.play(idleClip, FADE_SECONDS);
	}

	public function isWalking():Bool
		return walking;

	/** Distance walked along the current route, in metres. */
	public function distance():Float
		return travelled;

	/** Moves along the route and advances the animation by seconds. */
	public function advance(seconds:Float):Void {
		if (walking) {
			var length = distances[distances.length - 1];
			// Accelerate from rest over the fade into the walk; brake at the
			// same rate to stop exactly at the end of an open route.
			var acceleration = speed / FADE_SECONDS;
			velocity = Math.min(speed, velocity + acceleration * seconds);
			if (!loop)
				velocity = Math.min(velocity, Math.sqrt(2.0 * acceleration * Math.max(0.0, length - travelled)));
			travelled += velocity * seconds;
			if (loop)
				travelled %= length;
			else if (travelled >= length - 1e-6) {
				travelled = length;
				stop();
			}
			var point = pointAt(travelled);
			x = point[0];
			y = point[1];
			var target = tangentAt(travelled);
			var turn = wrap(target - heading);
			var limit = turnRate * seconds;
			heading = wrap(heading + Math.max(-limit, Math.min(limit, turn)));
		} else if (facing != null) {
			var target = facing;
			var turn = wrap(target - heading);
			var limit = turnRate * seconds;
			if (Math.abs(turn) <= limit) {
				heading = target;
				facing = null;
			} else
				heading = wrap(heading + (turn > 0.0 ? limit : -limit));
		}
		character.player.speed = walking ? velocity / gait.naturalSpeed : 1.0;
		character.advance(seconds);
	}

	/** The character root: standing at the walker's position, turned to its heading. */
	public function rootTransform():Array<Float> {
		var c = Math.cos(heading), s = Math.sin(heading);
		return [c, s, 0.0, 0.0, -s, c, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, x, y, 0.0, 1.0];
	}

	function segmentLength(index:Int):Float {
		var dx = points[index + 1][0] - points[index][0], dy = points[index + 1][1] - points[index][1];
		return Math.sqrt(dx * dx + dy * dy);
	}

	function segmentAt(along:Float):Int {
		for (index in 0...points.length - 1)
			if (along <= distances[index + 1])
				return index;
		return points.length - 2;
	}

	function pointAt(along:Float):Array<Float> {
		var index = segmentAt(along);
		var span = distances[index + 1] - distances[index];
		var t = span > 0.0 ? (along - distances[index]) / span : 0.0;
		return [
			points[index][0] + (points[index + 1][0] - points[index][0]) * t,
			points[index][1] + (points[index + 1][1] - points[index][1]) * t
		];
	}

	function tangentAt(along:Float):Float {
		var index = segmentAt(along);
		return Math.atan2(points[index + 1][1] - points[index][1], points[index + 1][0] - points[index][0]);
	}

	static function wrap(angle:Float):Float {
		var result = angle % (2.0 * Math.PI);
		if (result > Math.PI)
			result -= 2.0 * Math.PI;
		else if (result < -Math.PI)
			result += 2.0 * Math.PI;
		return result;
	}
}
