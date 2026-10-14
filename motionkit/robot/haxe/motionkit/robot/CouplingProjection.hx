package motionkit.robot;

import motionkit.trajectory.Trajectory;

/**
 * The couplings of a machine as one projection: a follower joint is the sum of its terms, `ratio * leader + offset`, taken
 * from joints that are complete before it. Built once from the model's terms: the followers come in dependency order (each
 * after the joints it follows), their terms grouped, and a cycle is an error (a joint cannot follow itself). Everything
 * that sets followers from their leaders (positions, rates, a trajectory's coefficients, the limits a follower inherits)
 * walks this one order.
 */
class CouplingProjection {
	/** The coupled joints, each after the coupled joints among its leaders. */
	public final followers:Array<Int> = [];
	/** The terms, grouped by follower in `followers` order. */
	public final termFollowers:Array<Int> = [];
	public final termLeaders:Array<Int> = [];
	public final termRatios:Array<Float> = [];
	public final termOffsets:Array<Float> = [];

	function new() {}

	/**
	 * The projection of `terms` (joint indices below `jointCount`). Followers are ordered by dependency, and the terms of a
	 * follower keep their authored order (the sum is taken in that order). Throws on a cycle.
	 */
	public static function of(terms:Array<{leader:Int, follower:Int, ratio:Float, offset:Float}>, jointCount:Int):CouplingProjection {
		var result = new CouplingProjection();
		var pending = terms.copy();
		var ordered:Array<{leader:Int, follower:Int, ratio:Float, offset:Float}> = [];
		while (pending.length > 0) {
			var next = -1;
			for (index in 0...pending.length) {
				var depends = false;
				for (other in pending) if (other.follower == pending[index].leader) depends = true;
				if (!depends) { next = index; break; }
			}
			if (next < 0) throw "Joint couplings contain a cycle";
			ordered.push(pending.splice(next, 1)[0]);
		}
		// A follower is complete once its last term is placed, which is after every leader is.
		var remaining = [for (_ in 0...jointCount) 0];
		for (term in ordered) remaining[term.follower]++;
		for (term in ordered) if (--remaining[term.follower] == 0) result.followers.push(term.follower);
		for (follower in result.followers) for (term in ordered) if (term.follower == follower) {
			result.termFollowers.push(follower); result.termLeaders.push(term.leader);
			result.termRatios.push(term.ratio); result.termOffsets.push(term.offset);
		}
		return result;
	}

	public function isEmpty():Bool return followers.length == 0;

	/** The terms of `follower` as (leader, ratio, offset), in the order they are summed. */
	public function termsOf(follower:Int):Array<{leader:Int, ratio:Float, offset:Float}> {
		var result = [];
		for (at in 0...termFollowers.length) if (termFollowers[at] == follower)
			result.push({leader: termLeaders[at], ratio: termRatios[at], offset: termOffsets[at]});
		return result;
	}

	/** Sets every coupled joint of `q` to the sum of its terms: positions take the offsets, rates (`positions` false) do not. */
	public function complete(q:Array<Float>, positions:Bool = true):Void {
		var at = 0;
		while (at < termFollowers.length) {
			var follower = termFollowers[at], sum = 0.0;
			while (at < termFollowers.length && termFollowers[at] == follower) {
				sum += termRatios[at] * q[termLeaders[at]] + (positions ? termOffsets[at] : 0.0);
				at++;
			}
			q[follower] = sum;
		}
	}

	/** Sets every coupled joint of `limits` to what its terms add up to when each leader is at its limit: the sum of |ratio| * leader. */
	public function addLimits(limits:Array<Float>):Void {
		var at = 0;
		while (at < termFollowers.length) {
			var follower = termFollowers[at], sum = 0.0;
			while (at < termFollowers.length && termFollowers[at] == follower) {
				sum += Math.abs(termRatios[at]) * limits[termLeaders[at]];
				at++;
			}
			limits[follower] = sum;
		}
	}

	/** A copy of `source` in which every follower is again exactly the sum of its terms (after a lowering that treated joints independently). */
	public function trajectory(source:Trajectory):Trajectory
		return source.projectCouplings(termFollowers, termLeaders, termRatios, termOffsets);
}
