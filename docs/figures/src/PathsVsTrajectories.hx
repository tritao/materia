import motionkit.planner.JointPathSamples;
import motionkit.planner.PathTimingLimits;
import motionkit.planner.ToppraPathTiming;
import motionkit.trajectory.Trajectory;

/**
 * One move of a two-axis gantry between the same two points, timed two ways: TOPP-RA along a fixed quarter-circle
 * path, and Ruckig as a free, jerk-limited move that picks its own (straight) route. Same axis limits for both.
 */
class PathsVsTrajectories {
	static inline var RADIUS = 0.2;
	static inline var PATH_SAMPLES = 121;
	static inline var SAMPLE_SECONDS = 0.005;
	static final MAX_VELOCITY = [0.5, 0.5];
	static final MAX_ACCELERATION = [2.0, 2.0];
	static final MAX_JERK = [40.0, 40.0];

	public static function compute():Dynamic {
		var path = new ToppraPathTiming().time(arc(), new PathTimingLimits(MAX_VELOCITY, MAX_ACCELERATION));
		var free = Trajectory.generateStateToState([RADIUS, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, RADIUS],
			MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK, true);
		var result = {
			figure: "paths-vs-trajectories",
			generator: "docs/figures/src/PathsVsTrajectories.hx",
			units: {position: "m", velocity: "m/s", acceleration: "m/s²", jerk: "m/s³", time: "s"},
			limits: {velocity: MAX_VELOCITY, acceleration: MAX_ACCELERATION, jerk: MAX_JERK},
			start: [RADIUS, 0.0],
			end: [0.0, RADIUS],
			methods: [
				method("toppra", "Path timing (TOPP-RA)", "Follows the fixed arc; fastest timing within velocity and acceleration limits.",
					path.trajectory),
				method("ruckig", "Free move (Ruckig)", "Chooses its own straight route; jerk-limited.", free)
			]
		};
		path.trajectory.dispose();
		path.releaseDistanceMap();
		free.dispose();
		return result;
	}

	/** Quarter circle about the origin from (r, 0) to (0, r), parameterised by arc length s = rθ. */
	static function arc():JointPathSamples {
		var s = [], q = [], qPrime = [], qDoublePrime = [];
		for (i in 0...PATH_SAMPLES) {
			var theta = Math.PI / 2 * i / (PATH_SAMPLES - 1);
			var cos = Math.cos(theta), sin = Math.sin(theta);
			s.push(RADIUS * theta);
			q.push([RADIUS * cos, RADIUS * sin]);
			qPrime.push([-sin, cos]);
			qDoublePrime.push([-cos / RADIUS, -sin / RADIUS]);
		}
		return new JointPathSamples(s, q, qPrime, qDoublePrime);
	}

	static function method(id:String, label:String, summary:String, trajectory:Trajectory):Dynamic {
		var duration = trajectory.durationSeconds();
		var samples = [];
		var steps = Math.ceil(duration / SAMPLE_SECONDS);
		for (step in 0...steps + 1) {
			var t = Math.min(step * SAMPLE_SECONDS, duration);
			var state = trajectory.evaluate(t);
			samples.push([for (value in [t].concat(state.positions).concat(state.velocities).concat(state.accelerations)
				.concat(state.jerks)) DocsFigures.round(value)]);
		}
		return {
			id: id,
			label: label,
			summary: summary,
			duration: DocsFigures.round(duration),
			columns: ["t", "x", "y", "vx", "vy", "ax", "ay", "jx", "jy"],
			samples: samples
		};
	}
}
