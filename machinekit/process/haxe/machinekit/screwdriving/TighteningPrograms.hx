package machinekit.screwdriving;

import machinekit.screwdriving.Fastening;
import processkit.screwdriving.TighteningProgram;
/** Explicit process settings; target torque and angle qualification belong to the CAD fastening. */
typedef TighteningPolicy = {
	var runDownSpeed:Float;
	var finalSpeed:Float;
	var runDownTorqueLimit:Float;
	var finalTorqueLimit:Float;
	var maximumRunDownAngle:Float;
	var minimumStroke:Float;
	var maximumStroke:Float;
};
/** Builds a fresh process program from the current fastening, refusing absent qualification. */
class TighteningPrograms {
	public static function fromFastening(fastening:Fastening, policy:TighteningPolicy):TighteningProgram {
		var window = fastening.angleWindow;
		if (window == null) throw '${fastening.name()}: tightening needs a qualified angle window from snug';
		return new TighteningProgram({runDownSpeed:policy.runDownSpeed, finalSpeed:policy.finalSpeed,
			targetTorque:fastening.tightening.torqueNm, minimumAngle:window.minimum, maximumAngle:window.maximum,
			runDownTorqueLimit:policy.runDownTorqueLimit, finalTorqueLimit:policy.finalTorqueLimit,
			maximumRunDownAngle:policy.maximumRunDownAngle, minimumStroke:policy.minimumStroke, maximumStroke:policy.maximumStroke});
	}
}
