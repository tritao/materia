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

	/** Signed travel for one positive screw revolution. */
	public function signedLead():Float
		return hand == RightHand ? lead : -lead;
}
