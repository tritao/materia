package motionkit.robot;

import haxe.Int64;
import motionkit.event.TimedEvent;
import motionkit.robot.ProgramCompiler.PendingMotion;
import motionkit.trajectory.ShaperImpulses;
import motionkit.trajectory.Trajectory;

/** What `TimedMotionTransforms` makes of a timed motion; it owns the trajectories it made (not the motion it was given). */
class TimedMotion {
	/**
	 * What is executed when the merged form is refused: the shaped motion when there is shaping (shaped from the pre-shape
	 * merge, so already within half the merge tolerances of the advanced one), else the coupled (and advanced) one, else the
	 * lowered one.
	 */
	public final executed:Trajectory;
	/** `executed` merged into longer pieces, or null when merging is off or was refused. */
	public var merged(default, null):Null<Trajectory>;
	/** Whether the executed motion is shaped, and so ends the shaper's tail later. */
	public final shaped:Bool;
	final owned:Array<Trajectory>;

	public function new(executed:Trajectory, merged:Null<Trajectory>, shaped:Bool, owned:Array<Trajectory>) {
		this.executed = executed; this.merged = merged; this.shaped = shaped; this.owned = owned;
	}

	/** The merged path broke a limit: it goes as `executed`. */
	public function dropMerged():Void {
		if (merged != null) merged.dispose();
		merged = null;
	}

	public function dispose():Void {
		for (trajectory in owned) trajectory.dispose();
		owned.splice(0, owned.length);
		dropMerged();
	}
}

/** The settings of the transforms, from the compiler. */
typedef MotionFilterSettings = {
	/** The extruder's lead on its velocity averaged over the window, seconds; zero for none. */
	var pressureAdvance:Float;
	var pressureAdvanceWindow:Float;
	/** Each shaped joint's impulses, or null for no shaping, and the mean delay the shaped motion lags the lowered one by, seconds. */
	var shaping:Null<Array<ShaperImpulses>>;
	var shapingDelay:Float;
}

typedef TimedMotionSettings = {
	var filters:MotionFilterSettings;
	/** The longest merged piece, seconds; zero for no merging. */
	var mergeSeconds:Float;
	/** Merges runs of segments of a trajectory (the followers not yet projected) within the given share of the merge tolerances. */
	var mergeRuns:Trajectory->Float->Trajectory;
}

/**
 * The transforms a lowered path's timed motion goes through on its way to a plan: the pressure advance (the driven joint
 * runs ahead of itself), the coupled joints projected again after each, the input shaper (cut after the advance's own cuts
 * are merged away, so it works on the plan's segments), and the merge of runs of segments for the link. It returns the
 * motion that is executed, its merged form, and owns the intermediate trajectories' disposal.
 */
class TimedMotionTransforms {
	/**
	 * `pending` is the lowered motion; shaping moves its events and times by the shaper's delay. `mark` names the stage just
	 * finished, for the compile profile.
	 */
	public static function apply(settings:TimedMotionSettings, projection:Null<CouplingProjection>, pending:PendingMotion,
			mark:String->Void):TimedMotion {
		if (pending.filtered) {
			var merged:Null<Trajectory> = null;
			if (settings.mergeSeconds > 0.0) {
				var runs = settings.mergeRuns(pending.trajectory,1.0);
				if (projection == null) merged = runs;
				else try { merged = projection.trajectory(runs); runs.dispose(); }
				catch (error:Dynamic) { runs.dispose(); throw error; }
			}
			mark("segmentMerge");
			return new TimedMotion(pending.trajectory,merged,pending.filteredShaped,[]);
		}
		var owned:Array<Trajectory> = [];
		function project(source:Trajectory):Trajectory return projection.trajectory(source);
		var projected:Null<Trajectory> = null;
		var shaped:Null<Trajectory> = null;
		var merged:Null<Trajectory> = null;
		var premerged = false;
		try {
			// Pressure advance runs the driven joint ahead of itself, and what follows it follows.
			var advanced:Null<Trajectory> = null;
			var drive = pending.path == null ? null : pending.path.drive;
			if (settings.filters.pressureAdvance > 0.0 && drive != null)
				advanced = pending.trajectory.advanceJoint(drive.joint, settings.filters.pressureAdvance, settings.filters.pressureAdvanceWindow);
			mark("pressureAdvance");
			var timed = advanced == null ? pending.trajectory : advanced;
			if (projection != null) projected = project(timed);
			else if (advanced != null) { projected = advanced; advanced = null; }
			if (advanced != null) advanced.dispose();
			if (projected != null) owned.push(projected);
			mark("advanceCouplings");
			// Input shaping: each shaped joint's motion becomes the weighted sum of delayed copies of itself, rest to rest,
			// and the plan ends the shaper's tail later. What the plan does at a given instant is the lowered motion's some
			// `shapingDelay` earlier, so the events and the times that locate the plan on its path move with it.
			var impulses = settings.filters.shaping;
			if (impulses != null && pending.path != null) {
				// The pressure advance cuts every plan into three times the segments, and the shaper cuts each at every delay:
				// the cuts of the advance are merged away first, so the shaper (and the projection after it) works on the
				// plan's own segments. The shaper is a weighted sum with positive weights adding to 1, so it passes the
				// pre-merge's error on unchanged, and the merge after it adds its own: each takes half the merge tolerances.
				var coarse:Null<Trajectory> = null;
				if (projected != null && settings.mergeSeconds > 0.0) {
					var runs = settings.mergeRuns(projected, 0.5);
					premerged = true;
					if (projection != null) { coarse = project(runs); runs.dispose(); } else coarse = runs;
				}
				mark("preShapeMerge");
				var cut:Trajectory;
				try {
					cut = (coarse != null ? coarse : (projected == null ? pending.trajectory : projected)).shape(
						[for (joint in impulses) joint.delays], [for (joint in impulses) joint.amplitudes]);
				} catch (error:Dynamic) {
					if (coarse != null) coarse.dispose();
					throw error;
				}
				if (coarse != null) coarse.dispose();
				mark("shapeConvolution");
				if (projection != null) { shaped = project(cut); cut.dispose(); } else shaped = cut;
				owned.push(shaped);
				var lag = Trajectory.nanoseconds(settings.filters.shapingDelay);
				var later = [for (event in pending.events) new TimedEvent(Int64.add(event.timeNs, lag), event.channel, event.value, event.holdPolicy)];
				pending.events.splice(0, pending.events.length);
				for (event in later) pending.events.push(event);
				for (index in 0...pending.times.length) pending.times[index] += settings.filters.shapingDelay;
				for (index in 0...pending.checkTimes.length) pending.checkTimes[index] += settings.filters.shapingDelay;
			}
			mark("inputShaping");
			var executed = shaped != null ? shaped : (projected == null ? pending.trajectory : projected);
			if (settings.mergeSeconds > 0.0 && pending.path != null) {
				var runs = settings.mergeRuns(executed, premerged ? 0.5 : 1.0);
				// The followers are again exactly the sum of their terms, in the merged coefficients.
				if (projection != null) { merged = project(runs); runs.dispose(); } else merged = runs;
			}
			mark("segmentMerge");
			return new TimedMotion(executed, merged, shaped != null, owned);
		} catch (error:Dynamic) {
			for (trajectory in owned) trajectory.dispose();
			if (merged != null) merged.dispose();
			throw error;
		}
	}
}
