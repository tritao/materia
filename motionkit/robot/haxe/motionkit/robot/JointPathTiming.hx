package motionkit.robot;

import haxe.Int64;
import motionkit.robot.JointMotionPolicy.JointMotionRates;
import motionkit.trajectory.Trajectory;

/** Time certified straight joint edges with synchronized, jerk-limited stops. */
class JointPathTiming {
	final velocity:Array<Float>;
	final acceleration:Array<Float>;
	final jerk:Array<Float>;
	var edgeTimes:Map<String,Int64> = [];
	var lowerTimes:Map<String,Float> = [];
	var edgeTimeCount = 0;
	var lowerTimeCount = 0;
	public var durationCacheHits(default, null):Int = 0;
	public var lowerBoundCacheHits(default, null):Int = 0;

	public function new(rates:Array<JointMotionRates>) {
		if (rates == null || rates.length == 0) throw "Joint path timing needs joint rates";
		velocity = [for (rate in rates) rate.velocity];
		acceleration = [for (rate in rates) rate.acceleration];
		jerk = [for (rate in rates) rate.jerk];
	}

	public function trajectory(path:Array<Array<Float>>):Trajectory {
		if (path == null || path.length < 2) throw "Joint path timing needs its two endpoints";
		var zero = [for (_ in 0...velocity.length) 0.0];
		var segments:Array<{timeFromStartNs:Int64, durationNs:Int64, coefficients:Array<Array<Float>>}> = [];
		var offset = Int64.ofInt(0);
		for (index in 1...path.length) {
			var edge = Trajectory.generateStateToState(path[index - 1], zero, zero, path[index],
				velocity, acceleration, jerk, true);
			var last = Int64.ofInt(0);
			for (segment in edge.segments()) {
				segments.push({timeFromStartNs: segment.timeFromStartNs + offset, durationNs: segment.durationNs,
					coefficients: segment.coefficients});
				last = segment.timeFromStartNs + segment.durationNs;
			}
			offset = offset + last;
			edge.dispose();
		}
		return Trajectory.fromSegments(segments);
	}

	/**
	 * Time a certified joint path passing its interior waypoints without stopping where `space`
	 * proves the curved motion clear. Each waypoint gets a pass velocity: per joint, the slower
	 * of its two edges' average speeds when both edges move it the same way, else zero. A segment
	 * whose motion `space` cannot cover with clear boxes halves the pass speeds at its ends, down
	 * to the certified straight edge stopped at both ends, so the result is never less clear than
	 * `trajectory(path)`; it is used only when it is also faster.
	 */
	public function blended(path:Array<Array<Float>>, space:motionkit.planner.BoxClearanceSpace,
			depthLimit:Int = 14):Trajectory {
		if (path == null || path.length < 2) throw "Joint path timing needs its two endpoints";
		if (space.dimension() != velocity.length) throw "Blended joint timing needs a space of its joint dimension";
		var count = velocity.length, last = path.length - 1;
		if (last < 2) return trajectory(path);
		var zero = [for (_ in 0...count) 0.0];
		var restSeconds = [for (index in 1...path.length) {
			var edge = Trajectory.generateStateToState(path[index - 1], zero, zero, path[index], velocity, acceleration, jerk, true);
			var seconds = edge.durationSeconds();
			edge.dispose();
			seconds;
		}];
		var pass = [for (index in 0...path.length) {
			if (index == 0 || index == last || restSeconds[index - 1] <= 0 || restSeconds[index] <= 0) zero.copy();
			else [for (joint in 0...count) {
				var incoming = (path[index][joint] - path[index - 1][joint]) / restSeconds[index - 1];
				var outgoing = (path[index + 1][joint] - path[index][joint]) / restSeconds[index];
				incoming * outgoing <= 0 ? 0.0 : (incoming > 0 ? 1.0 : -1.0) *
					Math.min(Math.min(Math.abs(incoming), Math.abs(outgoing)), velocity[joint]);
			}];
		}];
		var scale = [for (_ in 0...path.length) 1.0];
		function moving(index:Int):Bool {
			if (scale[index] == 0.0) return false;
			for (value in pass[index]) if (value != 0.0) return true;
			return false;
		}
		function velocityAt(index:Int):Array<Float> return [for (value in pass[index]) value * scale[index]];
		var timed:Array<{timeFromStartNs:Int64, durationNs:Int64, coefficients:Array<Array<Float>>}> = [];
		var settled = false;
		while (!settled) {
			settled = true;
			timed = [];
			var offset = Int64.ofInt(0);
			// Each segment starts from the state the previous one actually ends in: a segment that
			// ends moving lands within a nanosecond's travel of its waypoint, and the joins must be exact.
			var position = path[0].copy(), current = zero.copy(), accelerationNow = zero.copy();
			for (index in 1...path.length) {
				var from = index - 1;
				var straight = !moving(from) && !moving(index);
				// Stopped at both ends, the segment is the certified straight edge itself.
				var motion = straight
					? Trajectory.generateStateToState(position, zero, zero, path[index], velocity, acceleration, jerk, true)
					: Trajectory.generateStateToState(position, current, accelerationNow, path[index],
						velocity, acceleration, jerk, false, velocityAt(index));
				if (!straight && !certified(space, motion, velocity, depthLimit)) {
					motion.dispose();
					settled = false;
					for (end in [from, index]) if (scale[end] > 0.0) scale[end] = scale[end] > 0.125 ? scale[end] / 2 : 0.0;
					break;
				}
				var end = Int64.ofInt(0);
				for (piece in motion.segments()) {
					timed.push({timeFromStartNs: piece.timeFromStartNs + offset, durationNs: piece.durationNs,
						coefficients: piece.coefficients});
					end = piece.timeFromStartNs + piece.durationNs;
				}
				offset = offset + end;
				var state = motion.evaluate(motion.durationSeconds());
				position = state.positions.copy(); current = state.velocities.copy(); accelerationNow = state.accelerations.copy();
				motion.dispose();
			}
		}
		var result = Trajectory.fromSegments(timed), stopped = trajectory(path);
		if (result.durationSeconds() < stopped.durationSeconds()) {
			stopped.dispose();
			return result;
		}
		result.dispose();
		return stopped;
	}

	/**
	 * Whether `space` proves every configuration `motion` passes clear: each time interval's box
	 * spans its midpoint ± each joint's speed limit times half the interval. Intervals that are
	 * not proven are halved to `depthLimit`; one whose midpoint itself collides fails at once.
	 */
	static function certified(space:motionkit.planner.BoxClearanceSpace, motion:Trajectory, speed:Array<Float>,
			depthLimit:Int):Bool {
		var count = speed.length, lower = space.lower(), upper = space.upper();
		var pending:Array<{start:Float, end:Float, depth:Int}> = [{start: 0.0, end: motion.durationSeconds(), depth: 0}];
		while (pending.length > 0) {
			var centres:Array<Float> = [], widths:Array<Float> = [], middles:Array<Array<Float>> = [];
			for (interval in pending) {
				var middle = motion.evaluate(0.5 * (interval.start + interval.end)).positions.copy();
				middles.push(middle);
				for (joint in 0...count) {
					centres.push(middle[joint]);
					widths.push(speed[joint] * 0.5 * (interval.end - interval.start));
				}
			}
			var clear = space.boxesClear(centres, widths, pending.length);
			// A box must also stay within the joint limits, which the curve can overshoot.
			for (index in 0...pending.length) for (joint in 0...count)
				if (centres[index * count + joint] - widths[index * count + joint] < lower[joint] ||
					centres[index * count + joint] + widths[index * count + joint] > upper[joint]) clear[index] = false;
			var next:Array<{start:Float, end:Float, depth:Int}> = [];
			for (index in 0...pending.length) if (!clear[index]) {
				var interval = pending[index], middle = middles[index];
				var outside = false;
				for (joint in 0...count) if (middle[joint] < lower[joint] || middle[joint] > upper[joint]) outside = true;
				if (interval.depth >= depthLimit || outside || space.invalid(middle) != null) return false;
				var half = 0.5 * (interval.start + interval.end);
				next.push({start: interval.start, end: half, depth: interval.depth + 1});
				next.push({start: half, end: interval.end, depth: interval.depth + 1});
			}
			pending = next;
		}
		return true;
	}

	/** The same timing used for execution; release its temporary native trajectory. */
	public function duration(path:Array<Array<Float>>):Float {
		if (path == null || path.length < 2) throw "Joint path timing needs its two endpoints";
		var moving = false;
		for (point in path) if (point.length != velocity.length) throw "Joint path timing received the wrong joint dimension";
		for (index in 1...path.length) for (joint in 0...velocity.length)
			if (path[index - 1][joint] != path[index][joint]) moving = true;
		if (!moving) return 0.0;
		var total = Int64.ofInt(0);
		for (index in 1...path.length) {
			var from = path[index - 1], to = path[index];
			var key = motionkit.planner.PlannerQueryKey.of(from) + motionkit.planner.PlannerQueryKey.of(to);
			var duration = edgeTimes.get(key);
			if (duration == null) {
				var zero = [for (_ in velocity) 0.0];
				var edge = Trajectory.generateStateToState(from, zero, zero, to, velocity, acceleration, jerk, true);
				duration = edge.durationNanoseconds(); edge.dispose();
				if (edgeTimeCount >= 8192) { edgeTimes = []; edgeTimeCount = 0; }
				edgeTimes.set(key, duration); edgeTimeCount++;
			} else durationCacheHits++;
			total = total + duration;
		}
		return Int64.toFloat(total) * 1e-9;
	}

	/** Improve bends using executed edge time, while certifying every replacement edge. */
	public function refine(space:motionkit.planner.PlannerSpace, path:Array<Array<Float>>, passes:Int = 4):Array<Array<Float>> {
		if (path == null || path.length < 2 || passes < 0 || space.dimension() != velocity.length)
			throw "Joint path refinement needs a path, matching space and nonnegative pass count";
		var result = [for (point in path) point.copy()], originalEdges:Array<Float> = [];
		for (point in result) {
			if (point.length != velocity.length || space.invalid(point) != null)
				throw "Joint path refinement received an invalid configuration";
		}
		for (index in 1...result.length) {
			for (value in result[index - 1]) originalEdges.push(value);
			for (value in result[index]) originalEdges.push(value);
		}
		for (clear in space.edgesClear(originalEdges, result.length - 1)) if (!clear)
			throw "Joint path refinement received an uncertified edge";
		for (pass in 0...passes) {
			var improved = false;
			for (offset in 1...result.length - 1) {
				var index = pass % 2 == 0 ? offset : result.length - 1 - offset;
				var before = result[index - 1], current = result[index], after = result[index + 1];
				var seconds = duration([before, current, after]);
				for (fraction in [1.0, 0.5, 0.25]) {
					var candidate = [for (joint in 0...current.length)
						current[joint] + fraction * ((before[joint] + after[joint]) / 2 - current[joint])];
					if (space.invalid(candidate) != null) continue;
					var proposed = duration([before, candidate, after]);
					if (proposed >= seconds - 1e-6) continue;
					var edges = before.concat(candidate).concat(candidate).concat(after);
					var clear = space.edgesClear(edges, 2);
					if (!clear[0] || !clear[1]) continue;
					result[index] = candidate; seconds = proposed; improved = true;
				}
			}
			// Moving a bend can make its neighbouring edge newly visible. Remove
			// the stop only when that complete edge is clear and actually faster.
			var index = 1;
			while (index < result.length - 1) {
				var before = result[index - 1], current = result[index], after = result[index + 1];
				if (duration([before, after]) < duration([before, current, after]) - 1e-6 &&
					space.edgesClear(before.concat(after), 1)[0]) {
					result.splice(index, 1); improved = true;
				} else index++;
			}
			if (!improved) break;
		}
		return result;
	}

	/** Independent joint minima bound any rest-to-rest path, including detours. */
	public function lowerBound(from:Array<Float>, to:Array<Float>):Float {
		if (from.length != velocity.length || to.length != velocity.length)
			throw "Joint timing endpoints must match the joint rates";
		var key = motionkit.planner.PlannerQueryKey.of(from) + motionkit.planner.PlannerQueryKey.of(to);
		if (lowerTimes.exists(key)) { lowerBoundCacheHits++; return lowerTimes.get(key); }
		var seconds = 0.0;
		for (joint in 0...velocity.length) {
			var motion = Trajectory.generateStateToState([from[joint]], [0.0], [0.0], [to[joint]],
				[velocity[joint]], [acceleration[joint]], [jerk[joint]], true);
			seconds = Math.max(seconds, motion.durationSeconds());
			motion.dispose();
		}
		if (lowerTimeCount >= 8192) { lowerTimes = []; lowerTimeCount = 0; }
		lowerTimes.set(key, seconds); lowerTimeCount++;
		return seconds;
	}
}
