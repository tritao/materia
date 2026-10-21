package machinekit.milling;

import machinekit.motion.*;

/** An X/Y drive selection: ball nut, screw supports, servo and coupling. `MillXYStageLayout` places the set. */
class MillXYDrive {
	public final nut:BallNut;
	public final fixed:ScrewSupportUnit;
	public final floating:ScrewSupportUnit;
	public final motor:LeadshineServo;
	public final coupling:ShaftCoupling;

	public function new(nut:BallNut, fixed:ScrewSupportUnit, floating:ScrewSupportUnit, motor:LeadshineServo, coupling:ShaftCoupling) {
		this.nut = nut;
		this.fixed = fixed;
		this.floating = floating;
		this.motor = motor;
		this.coupling = coupling;
	}

	/** BK12/BF12 supports, preloaded 1605 nut and the ELM2H-0400 servo with a 14/12 coupling. */
	public static function bk12():MillXYDrive
		return new MillXYDrive(new BallNut(), ScrewSupportUnit.bk12(), ScrewSupportUnit.bf12(), LeadshineServo.model("ELM2H-0400LA60F"),
			new ShaftCoupling(14, 12, null, 28));
}
