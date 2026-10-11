package motionkit.robot;

import haxe.Int64;
import motionkit.robot.JointMotionPolicy.JointMotionRates;
import motionkit.trajectory.Trajectory;

/** Time certified straight joint edges with synchronized, jerk-limited stops. */
class JointPathTiming {
	final velocity:Array<Float>;
	final acceleration:Array<Float>;
	final jerk:Array<Float>;

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

	/** The same timing used for execution; release its temporary native trajectory. */
	public function duration(path:Array<Array<Float>>):Float {
		if (path == null || path.length < 2) throw "Joint path timing needs its two endpoints";
		var moving = false;
		for (point in path) if (point.length != velocity.length) throw "Joint path timing received the wrong joint dimension";
		for (index in 1...path.length) for (joint in 0...velocity.length)
			if (path[index - 1][joint] != path[index][joint]) moving = true;
		if (!moving) return 0.0;
		var motion = trajectory(path), seconds = motion.durationSeconds();
		motion.dispose();
		return seconds;
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
		var seconds = 0.0;
		for (joint in 0...velocity.length) {
			var motion = Trajectory.generateStateToState([from[joint]], [0.0], [0.0], [to[joint]],
				[velocity[joint]], [acceleration[joint]], [jerk[joint]], true);
			seconds = Math.max(seconds, motion.durationSeconds());
			motion.dispose();
		}
		return seconds;
	}
}
