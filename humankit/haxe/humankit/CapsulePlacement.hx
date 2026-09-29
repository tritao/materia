package humankit;

/** Where one proxy capsule is: its centre and a rotation taking +Z to its axis (x, y, z, w). */
class CapsulePlacement {
	public final center:Array<Float>;
	public final rotation:Array<Float>;

	public function new(center:Array<Float>, rotation:Array<Float>) {
		this.center = center;
		this.rotation = rotation;
	}
}
