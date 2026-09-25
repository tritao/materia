package machinekit.transmission;

import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord.AssemblyFrame;

/** Two spur gears meshed at the standard (no profile shift) centre distance, both rotating about
 * parallel +Z axes. `a` sits at the origin; `pose` places `b` along `a`'s local +X.
 */
class GearPair {
	public final a:SpurGear;
	public final b:SpurGear;
	public final centerDistance:Float;

	public static function mesh(a:SpurGear, b:SpurGear):GearPair
		return new GearPair(a, b);

	function new(a:SpurGear, b:SpurGear) {
		this.a = a;
		this.b = b;
		centerDistance = a.centerDistance(b);
	}

	/** Gear ratio, driven teeth over driving teeth: `b.teeth / a.teeth`. */
	public function ratio():Float
		return b.teeth / a.teeth;

	/** World pose for `b` when `a` sits at the identity pose. */
	public function pose():AssemblyFrame
		return AssemblyFrames.translation(centerDistance, 0, 0);
}
