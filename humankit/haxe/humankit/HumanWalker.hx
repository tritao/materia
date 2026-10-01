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
	/** How long the body takes to get up to walking speed and to stop. */
	static inline var FADE_SECONDS:Float = 0.3;
	/**
	 * How long the idle and walk clips take to cross-fade. Shorter than the speed ramp: while the body speeds up
	 * the idle pose keeps its feet still, so the longer the idle shows through the further a planted foot is dragged.
	 */
	static inline var BLEND_SECONDS:Float = 0.3;

	public final character:HumanCharacter;
	public final gait:HumanGait;
	/** The gait of a walk that carries something, when the character has a clip for it; see setCarrying. */
	public final carryGait:Null<HumanGait>;
	/** The gait of walking backwards, when the character has a clip for it; a retreat uses it. */
	public final backGait:Null<HumanGait>;
	/** The turn-in-place clips, to the left and to the right, when the character has them; see face. */
	public final turnLeft:Null<HumanTurn>;
	public final turnRight:Null<HumanTurn>;
	/** Whole turns of a clip still to do, and which way (+1 left, -1 right), the one playing, and its clock. */
	var turnsLeft:Int = 0;
	var turnSign:Float = 1.0;
	var turnPlaying:Null<HumanTurn> = null;
	var turnClock:Float = 0.0;
	/** The gait the route being walked uses, so a walk does not change clip part way. */
	var routeGait:HumanGait;
	var carrying:Bool = false;
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
	var preserveHeading:Bool = false;
	var retreating:Bool = false;

	/** Measures the walk clip's gait and starts the character idling at the origin. */
	public function new(character:HumanCharacter, walkClip:String = "walk", idleClip:String = "idle") {
		this.character = character;
		var walk = character.asset.clipIndex(walkClip);
		var idle = character.asset.clipIndex(idleClip);
		if (walk < 0 || idle < 0)
			throw 'The character needs "$walkClip" and "$idleClip" clips';
		gait = HumanGait.measure(character.asset, character.rig, walk);
		routeGait = gait;
		var carry = character.asset.clipIndex("walk_carry");
		carryGait = carry < 0 ? null : HumanGait.measure(character.asset, character.rig, carry);
		var back = character.asset.clipIndex("walk_bwd");
		backGait = back < 0 ? null : HumanGait.measure(character.asset, character.rig, back, true);
		var left = HumanTurn.measure(character.asset, character.rig, character.asset.clipIndex("turn90_l"));
		var right = HumanTurn.measure(character.asset, character.rig, character.asset.clipIndex("turn90_r"));
		// Each is used for the direction it turns, whatever its name says.
		turnLeft = left != null && left.angle > 0.0 ? left : right != null && right.angle > 0.0 ? right : null;
		turnRight = right != null && right.angle < 0.0 ? right : left != null && left.angle < 0.0 ? left : null;
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
		preserveHeading = false;
		retreating = false;
		var carryWalk = carryGait;
		routeGait = carrying && carryWalk != null ? carryWalk : gait;
		character.player.play(routeGait.clip, BLEND_SECONDS);
	}

	/**
	 * Whether the next walk is one that carries something: a character with a carrying walk clip uses it, with
	 * the shoulders and stance of a worker holding a load. It applies from the next route, not the one under way.
	 */
	public function setCarrying(value:Bool):Void
		carrying = value;

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

	/** Moves along a retreat route without turning toward it. */
	public function retreatAlong(route:Array<Array<Float>>, metresPerSecond:Float):Void {
		continueAlong(route, metresPerSecond);
		preserveHeading = true;
		retreating = true;
		// Backwards on the character's own backward walk, or else sliding in the idle pose.
		var backWalk = backGait;
		if (backWalk != null) routeGait = backWalk;
		character.player.play(backWalk != null ? backWalk.clip : idleClip, BLEND_SECONDS);
	}

	/** Sets the starting floor pose before a job begins. */
	public function place(x:Float, y:Float, heading:Float):Void {
		stop();
		this.x = x;
		this.y = y;
		this.heading = wrap(heading);
		facing = null;
	}

	/**
	 * Returns to rest at a floor pose as if newly constructed: no route, no turn
	 * pending, and the idle clip playing from its start.
	 */
	public function restart(x:Float, y:Float, heading:Float):Void {
		points = [];
		distances = [];
		loop = false;
		speed = 0.0;
		travelled = 0.0;
		velocity = 0.0;
		walking = false;
		facing = null;
		turnsLeft = 0;
		turnPlaying = null;
		preserveHeading = false;
		retreating = false;
		this.x = x;
		this.y = y;
		this.heading = wrap(heading);
		character.player.restart(idleClip);
	}

	/** Turns in place at turnRate, including after a route has ended. */
	public function face(angle:Float):Void {
		var target = wrap(angle);
		if (facing != null && turnsLeft > 0 && Math.abs(wrap(target - facing)) < 0.01) return;
		facing = target;
		planTurnClips();
	}

	/**
	 * A turn of about a right angle or more, standing still, is taken in steps of the character's turn clip, which
	 * steps the feet round, and what is left over (under about 45 degrees) by turning the root as before.
	 */
	function planTurnClips():Void {
		turnsLeft = 0;
		turnPlaying = null;
		var target = facing;
		if (walking || target == null) return;
		var delta = wrap(target - heading);
		var clip = delta > 0.0 ? turnLeft : turnRight;
		if (clip == null) return;
		var steps = Std.int(Math.min(2.0, Math.floor(Math.abs(delta) / Math.abs(clip.angle) + 0.33)));
		if (steps < 1) return;
		turnsLeft = steps;
		turnSign = delta > 0.0 ? 1.0 : -1.0;
	}

	public function isTurning():Bool
		return facing != null || turnsLeft > 0;

	/**
	 * How much of the pose showing is the idle one, whose feet are on the floor: 1 standing, 0 in a steady walk, and
	 * in between while the walk fades in or out. A planted foot is held in the world while this is high.
	 */
	public function stanceShare():Float {
		var weight = character.player.fadeWeight();
		var playing = character.player.currentClip();
		return playing == idleClip ? weight : (character.player.fading() ? 1.0 - weight : 0.0);
	}

	/** Whether the character stands still with its walk faded out: not walking, not turning, not mid-crossfade. */
	public function settled():Bool
		return !walking && facing == null && turnsLeft == 0 && !character.player.fading();

	/** Stops where the character stands and idles. */
	public function stop():Void {
		facing = null;
		turnsLeft = 0;
		turnPlaying = null;
		if (!walking)
			return;
		walking = false;
		velocity = 0.0;
		character.player.play(idleClip, BLEND_SECONDS);
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
			if (!preserveHeading) {
				var target = tangentAt(travelled);
				var turn = wrap(target - heading);
				var limit = turnRate * seconds;
				heading = wrap(heading + Math.max(-limit, Math.min(limit, turn)));
			}
		} else if (turnsLeft > 0) {
			advanceTurnClip(seconds);
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
		character.player.speed = walking && (!retreating || backGait != null) ? velocity / routeGait.naturalSpeed : 1.0;
		character.advance(seconds);
	}

	/**
	 * Plays the turn clip for one step and, when its turn is done, turns the root by the same angle while
	 * the idle pose takes over, which is that pose turned the same way: nothing moves but the root.
	 */
	function advanceTurnClip(seconds:Float):Void {
		var clip = turnSign > 0.0 ? turnLeft : turnRight;
		if (clip == null) {
			turnsLeft = 0;
			return;
		}
		if (turnPlaying == null) {
			turnPlaying = clip;
			turnClock = 0.0;
			if (character.player.currentClip() == clip.clip) character.player.restart(clip.clip, false);
			else character.player.play(clip.clip, 0.15, false);
		}
		turnClock += seconds;
		if (turnClock < clip.seconds) return;
		heading = wrap(heading + clip.angle);
		turnsLeft--;
		turnPlaying = null;
		if (turnsLeft == 0) character.player.restart(idleClip);
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
