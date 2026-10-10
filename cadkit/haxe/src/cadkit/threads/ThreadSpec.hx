package cadkit.threads;

/** Hand of a thread, independent of the face's orientation. */
@:wire
enum ThreadHand {
	@:id(1) RightHand;
	@:id(2) LeftHand;
}

/** Nominal thread profile; ball races have no cosmetic flank profile. */
@:wire
enum ThreadProfile {
	@:id(1) Metric;
	@:id(2) Trapezoidal;
	@:id(3) Acme;
	@:id(4) Ball;
}

/** Immutable nominal thread dimensions in millimetres; no helical B-rep is made. */
class ThreadSpec {
	public final designation:String;
	public final majorDiameter:Float;
	public final pitch:Float;
	public final starts:Int;
	public final hand:ThreadHand;
	public final internal:Bool;
	public final toleranceClass:String;
	public final profile:ThreadProfile;
	public var lead(get, never):Float;

	public function new(designation:String, majorDiameter:Float, pitch:Float, starts:Int = 1,
			hand:ThreadHand = RightHand, internal:Bool = false, toleranceClass:Null<String> = null,
			profile:ThreadProfile = Metric) {
		if (designation == null || StringTools.trim(designation) == "") throw "Thread needs a designation";
		if (!Math.isFinite(majorDiameter) || majorDiameter <= 0) throw "Thread needs a positive major diameter";
		if (!Math.isFinite(pitch) || pitch <= 0) throw "Thread needs a positive pitch";
		if (starts < 1 || !Math.isFinite(pitch * starts)) throw "Thread needs a finite lead and at least one start";
		var tolerance = toleranceClass == null ? (profile == Metric ? (internal ? "6H" : "6g") : "nominal") : toleranceClass;
		if (hand == null || profile == null || StringTools.trim(tolerance) == "")
			throw "Thread needs a hand, profile and tolerance class";
		this.designation = designation;
		this.majorDiameter = majorDiameter;
		this.pitch = pitch;
		this.starts = starts;
		this.hand = hand;
		this.internal = internal;
		this.toleranceClass = tolerance;
		this.profile = profile;
	}

	public function data():ThreadSpecData return {
		designation: designation, majorDiameter: majorDiameter, pitch: pitch, starts: starts,
		hand: hand, internal: internal, toleranceClass: toleranceClass, profile: profile
	};

	public static function fromData(data:ThreadSpecData):ThreadSpec
		return new ThreadSpec(data.designation, data.majorDiameter, data.pitch, data.starts,
			data.hand, data.internal, data.toleranceClass, data.profile);

	function get_lead():Float return pitch * starts;

	/** The same nominal thread on its mating face. */
	public function mating():ThreadSpec
		return new ThreadSpec(designation, majorDiameter, pitch, starts, hand, !internal,
			profile == Metric ? (internal ? "6g" : "6H") : toleranceClass, profile);

	/** Matching dimensions do not imply that two faces are the same face. */
	public function matches(other:ThreadSpec):Bool
		return other != null && profile == other.profile && Math.abs(majorDiameter - other.majorDiameter) < 1e-6
			&& Math.abs(pitch - other.pitch) < 1e-6 && starts == other.starts && hand == other.hand;
}
