package humankit;

/**
 * One capsule of a body proxy. It runs from its `from` bone towards its `to`
 * bone, continued past `to` by `extension` times that span (a head beyond its
 * joint, fingers beyond the knuckles) or stopped `trim` metres short of it (a
 * shin that leaves the ankle to the foot). The capsule's hemisphere centres sit
 * at the two ends, so it covers `radius` beyond each.
 */
class HumanCapsule {
	public final name:String;
	public final from:HumanBone;
	public final to:HumanBone;
	public final extension:Float;
	public final trim:Float;
	public final radius:Float;
	/** Distance between the hemisphere centres, fixed at the rest pose. */
	public final length:Float;

	public function new(name:String, from:HumanBone, to:HumanBone, extension:Float, trim:Float, radius:Float,
			length:Float) {
		this.name = name;
		this.from = from;
		this.to = to;
		this.extension = extension;
		this.trim = trim;
		this.radius = radius;
		this.length = length;
	}
}
