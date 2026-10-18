package materia.project;

/** Hand of a thread, independent of the face's orientation. */
@:wire
enum ThreadHand {
	@:id(1) RightHand;
	@:id(2) LeftHand;
}

/** Nominal thread profile; ball races use rounded cosmetic grooves. */
@:wire
enum ThreadProfile {
	@:id(1) Metric;
	@:id(2) Trapezoidal;
	@:id(3) Acme;
	@:id(4) Ball;
}

/** Current saved nominal dimensions; face intervals are rebuilt from their feature. */
@:wire
typedef ThreadSpecData = {
	@:id(1) var designation:String;
	@:id(2) var majorDiameter:Float;
	@:id(3) var pitch:Float;
	@:id(4) var starts:Int;
	@:id(5) var hand:ThreadHand;
	@:id(6) var internal:Bool;
	@:id(7) var toleranceClass:String;
	@:id(8) var profile:ThreadProfile;
}

/** One validation policy for CAD values and portable saved data. */
class ThreadSpecDataValues {
	public static function validate(data:ThreadSpecData):Void {
		if (data.designation == null || StringTools.trim(data.designation) == "") throw "Thread needs a designation";
		if (!Math.isFinite(data.majorDiameter) || data.majorDiameter <= 0) throw "Thread needs a positive major diameter";
		if (!Math.isFinite(data.pitch) || data.pitch <= 0) throw "Thread needs a positive pitch";
		if (data.starts < 1 || !Math.isFinite(data.pitch * data.starts)) throw "Thread needs a finite lead and at least one start";
		if (data.hand == null || data.profile == null || data.toleranceClass == null || StringTools.trim(data.toleranceClass) == "")
			throw "Thread needs a hand, profile and tolerance class";
	}
}
