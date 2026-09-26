package machinekit.transmission;

import materia.project.AssemblyRecord.AssemblyFrame;

/** Two spur gears meshed at the standard (no profile shift) centre distance, both rotating about
 * parallel +Z axes. `a` sits at the origin; `pose` places `b` along `a`'s local +X, turned about
 * its own axis so the teeth interleave.
 */
class GearPair {
	public final a:SpurGear;
	public final b:SpurGear;
	public final centerDistance:Float;
	/** Rotation of `b` about its +Z axis, radians, in `0 <= angle < 2*pi/b.teeth`. */
	public final bRotation:Float;

	public static function mesh(a:SpurGear, b:SpurGear):GearPair
		return new GearPair(a, b);

	function new(a:SpurGear, b:SpurGear) {
		this.a = a;
		this.b = b;
		centerDistance = a.centerDistance(b);
		// `SpurGear` centres a tooth on local angle 0, so `a` has a tooth pointing at the mesh
		// point (+X). `b` sees the mesh point at its local angle pi and must present a tooth
		// space there; spaces sit half a pitch from teeth, so the turn t satisfies
		// t + (k + 1/2) * 2pi/z_b = pi, i.e. t = pi - pi/z_b (mod 2pi/z_b): zero for odd z_b and
		// half a tooth pitch (pi/z_b) for even z_b.
		var step = 2 * Math.PI / b.teeth;
		var turn = Math.PI - Math.PI / b.teeth;
		turn -= step * Math.ffloor(turn / step + 1e-9);
		bRotation = turn < 1e-9 ? 0.0 : turn;
	}

	/** Gear ratio, driven teeth over driving teeth: `b.teeth / a.teeth`. */
	public function ratio():Float
		return b.teeth / a.teeth;

	/** World pose for `b` when `a` sits at the identity pose: translated to the centre distance
	 * and turned by `bRotation` about +Z.
	 */
	public function pose():AssemblyFrame
		return {x: centerDistance, y: 0, z: 0, qx: 0, qy: 0, qz: Math.sin(bRotation / 2), qw: Math.cos(bRotation / 2)};
}
