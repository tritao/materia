package machinekit.motion;

import machinekit.component.Dimension;

/** Semantic lead-screw thread. Dimensions are nominal millimetres; this does not validate
 * that a diameter/pitch combination is listed in a particular edition of a thread standard.
 */
enum LeadScrewThreadFamily {
	MetricTrapezoidal;
	Acme;
}

enum LeadScrewHand {
	RightHand;
	LeftHand;
}

class LeadScrewThread {
	public final family:LeadScrewThreadFamily;
	public final screwDiameter:Float;
	public final pitch:Float;
	public final starts:Int;
	public final hand:LeadScrewHand;
	/** Positive axial distance per revolution, independent of handedness. */
	public final lead:Float;
	public final designation:String;

	public function new(family:LeadScrewThreadFamily, screwDiameter:Float, pitch:Float,
			starts:Int = 1, hand:LeadScrewHand = RightHand) {
		if (family == null) throw "Lead screw thread needs a family";
		if (!(screwDiameter > 0) || !Math.isFinite(screwDiameter))
			throw "Lead screw thread needs a positive screw diameter";
		if (!(pitch > 0) || !Math.isFinite(pitch)) throw "Lead screw thread needs a positive pitch";
		if (starts < 1) throw "Lead screw thread needs at least one start";
		if (hand == null) throw "Lead screw thread needs a hand";
		var calculatedLead = pitch * starts;
		if (!Math.isFinite(calculatedLead)) throw "Lead screw thread lead is too large";
		this.family = family;
		this.screwDiameter = screwDiameter;
		this.pitch = pitch;
		this.starts = starts;
		this.hand = hand;
		lead = calculatedLead;
		var familyCode = family == MetricTrapezoidal ? "TR" : "ACME";
		var handCode = hand == RightHand ? "RH" : "LH";
		designation = '${familyCode}-D${Dimension.format(screwDiameter)}-P${Dimension.format(pitch)}-S$starts-$handCode';
	}

	/** Signed nut travel for one positive screw revolution about the screw's +Z axis.
	 * A right-hand thread moves the nut toward -Z; a left-hand thread moves it toward +Z.
	 */
	public function signedLead():Float
		return hand == RightHand ? -lead : lead;

	/**
	 * Share of the motor's work that reaches the nut when the screw drives it: tan λ / tan(λ + φ'),
	 * with λ the lead angle at the pitch diameter and φ' the friction angle on the 15° flanks of a
	 * trapezoidal or Acme thread. `friction` is the nut's sliding friction coefficient; 0.1 is
	 * typical of a greased bronze or plastic nut on steel.
	 */
	public function efficiency(friction:Float = 0.1):Float {
		if (!(friction >= 0) || !Math.isFinite(friction)) throw "Lead screw friction must be non-negative";
		var pitchDiameter = screwDiameter - pitch / 2;
		var leadAngle = Math.atan(lead / (Math.PI * pitchDiameter));
		var frictionAngle = Math.atan(friction / Math.cos(15 * Math.PI / 180));
		return Math.tan(leadAngle) / Math.tan(leadAngle + frictionAngle);
	}
}
