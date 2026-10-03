package machinekit.transmission;

/** Resolved motion and engineering allowances, in the coupling's units. */
class TransmissionRelation {
	public var ratio:Float;
	public var efficiency:Float;
	public var stiffness:Null<Float>;
	public var backlash:Null<Float>;
	public var drag:Null<Float>;
	public var followerSpeedCap:Null<Float>;

	public function new(ratio:Float, efficiency:Float, ?stiffness:Float, ?backlash:Float,
			?drag:Float, ?followerSpeedCap:Float) {
		this.ratio = ratio;
		this.efficiency = efficiency;
		this.stiffness = stiffness;
		this.backlash = backlash;
		this.drag = drag;
		this.followerSpeedCap = followerSpeedCap;
	}
}
