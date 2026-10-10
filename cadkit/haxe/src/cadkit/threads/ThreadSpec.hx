package cadkit.threads;

import materia.project.ThreadSpecData;
import materia.project.ThreadSpecData.ThreadSpecDataValues;
import materia.project.ThreadSpecData.ThreadHand;
import materia.project.ThreadSpecData.ThreadProfile;

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
		var tolerance = toleranceClass == null ? (profile == Metric ? (internal ? "6H" : "6g") : "nominal") : toleranceClass;
		ThreadSpecDataValues.validate({designation: designation, majorDiameter: majorDiameter, pitch: pitch,
			starts: starts, hand: hand, internal: internal, toleranceClass: tolerance, profile: profile});
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
