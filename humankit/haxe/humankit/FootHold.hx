package humankit;

import humankit.rig.HumanBone;

/**
 * While the idle pose shows (standing, and the fade into and out of a walk), the feet are held where they stand:
 * the idle pose keeps its feet still relative to the body, so a body that starts to move drags them, and one that
 * lowers or rises slides them. Each foot is reached for at the spot it had when the hold began, with a hold that
 * comes in and goes out smoothly; in a steady walk the gait already keeps a planted foot still. Only a character whose
 * legs are IK chains can hold its feet.
 */
class FootHold {
	final body:HumanBody;
	final enabled:Bool;
	final anchors:Array<Null<Array<Float>>> = [null, null];
	var lock:Float = 0.0;

	public function new(body:HumanBody) {
		this.body = body;
		enabled = body.character.legsAreChains();
	}

	/** How firmly the feet are held where they stand, 0 (free) to 1 (held). */
	public function amount():Float
		return lock;

	/** Lets go of both feet at once. */
	public function reset():Void {
		lock = 0.0;
		anchors[0] = null;
		anchors[1] = null;
	}

	/** Eases the hold toward what the pose asks for and reaches each held foot to its anchor. `kneeling` turns it off. */
	public function advance(seconds:Float, kneeling:Bool):Void {
		var posture = body.posture;
		if (!enabled || !posture.lockFeet) return;
		// A kneel puts a foot behind the body and a knee on the floor, which a foot held where it stood would fight.
		var share = kneeling ? 0.0 : body.walker.stanceShare();
		var wanted = Math.max(0.0, Math.min(1.0, (share - posture.lockFrom) / posture.lockSpan));
		wanted = wanted * wanted * (3.0 - 2.0 * wanted);
		var step = seconds / posture.lockSeconds;
		lock = Math.abs(wanted - lock) <= step ? wanted : lock + (wanted > lock ? step : -step);
		var feet = [HumanBone.FootL, HumanBone.FootR], legs = [LegL, LegR];
		if (lock <= 1e-4) {
			for (side in 0...2) if (anchors[side] != null) {
				anchors[side] = null;
				body.clearReach(legs[side]);
			}
			return;
		}
		for (side in 0...2) {
			var foot = body.character.pose.bonePosition(feet[side]);
			if (foot == null) continue;
			var here = body.toWorld(foot);
			var anchor = anchors[side];
			// Taken where the foot is now, as the hold begins; let go and taken again if the body has left it too far behind.
			if (anchor == null || distance(anchor, here) > posture.lockReach + 0.5 * (1.0 - lock)) {
				anchor = here;
				anchors[side] = anchor;
			}
			body.setReachWorld(legs[side], anchor, lock);
		}
	}

	static function distance(a:Array<Float>, b:Array<Float>):Float
		return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2) + Math.pow(a[2] - b[2], 2));
}
