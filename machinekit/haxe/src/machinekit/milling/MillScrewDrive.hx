package machinekit.milling;

import machinekit.motion.*;
import machinekit.standard.DeepGrooveBearing;

/** A supported ball-screw drive: a motor mount with its fixed bearings, the floating support and its retaining ring, servo and coupling.
 * Every mounting datum is read from the parts' catalogue and STEP review data; only the coupling
 * (outside diameter, length, clamps and where it seats) is stated here. Nothing is sized for a load.
 */
class MillScrewDrive {
	/** Integrated motor mount with the fixed bearing pair. */
	public final fixed:MotorMount;
	public final floating:ScrewSupportUnit;
	/** The DIN 471 ring that holds the floating bearing on its journal, and where along the support it sits. */
	public final ring:machinekit.standard.RetainingRing;
	public final ringSeatZ:Float;
	public final motor:LeadshineServo;
	public final coupling:ShaftCoupling;
	/** Motor face to the coupling origin, along the screw. */
	public final couplingSeat:Float;
	/** HZMotion screw end machined for the fixed mount. */
	public final endProfile:String;
	/** Motor face to fixed-bearing centre. */
	public final bearingDrop:Float;
	/** Screw's fixed end, shoulder to thread: the screw starts `fixed.spec.shoulderZ - fixedEnd` from the motor face. */
	public final fixedEnd:Float;
	/** Screw end to the floating bearing centre: the journal less half the bearing, which seats on its shoulder. */
	public final floatingEnd:Float;
	/** Floating support's bolt line and bolt offset. */
	public final floatingBoltZ:Float;
	public final floatingBoltX:Float;
	/** Bolt counterbore seats above each part's mounting base. */
	public final fixedBoltSeat:Float;
	public final floatingBoltSeat:Float;

	public function new(fixed:MotorMount, floating:ScrewSupportUnit, motor:LeadshineServo, couplingLength:Float, couplingSeat:Float,
			?couplingOutside:Float, ?couplingClamps:Array<ShaftCoupling.ShaftCouplingSetScrew>) {
		this.fixed = fixed;
		this.floating = floating;
		this.motor = motor;
		this.couplingSeat = couplingSeat;
		if (!Math.isFinite(couplingSeat) || couplingSeat <= 0) throw "Screw-drive coupling seat must be finite and positive";
		var profile = fixed.spec.endProfile;
		if (profile == null || !StringTools.startsWith(profile, "C-")) throw "Screw drive needs a C-profile fixed mount";
		endProfile = profile.substr(2);
		if (fixed.spec.oppositeSupport != floating.spec.model) throw "Floating support must be the fixed mount's opposite support";
		if (fixed.spec.shoulderZ == null) throw "Screw drive mount needs a screw shoulder datum";
		bearingDrop = fixed.bearingCentreZ();
		fixedEnd = HzMotionDrawingScrew.fixedEndFor(endProfile);
		coupling = new ShaftCoupling(motor.rating.shaftDiameter, HzMotionDrawingScrew.couplingDiameterFor(endProfile), couplingOutside,
			couplingLength, couplingClamps);
		var bearing = floating.bearing;
		if (!StringTools.endsWith(bearing, "ZZ")) throw "Floating support needs a shielded deep-groove bearing";
		floatingEnd = HzMotionDrawingScrew.floatingEndFor(endProfile)
			- DeepGrooveBearing.catalog().get(bearing.substr(0, bearing.length - 2)).width / 2;
		var holes = floating.footHoles();
		if (holes.length != 2 || holes[0].z != holes[1].z || holes[0].x != -holes[1].x) throw "Floating support needs one symmetric bolt pair";
		floatingBoltZ = holes[0].z;
		floatingBoltX = Math.abs(holes[0].x);
		fixedBoltSeat = fixed.boltSeatY();
		floatingBoltSeat = floating.boltSeatY();
		var seat = floating.ringSeatZ();
		if (seat == null) throw 'Floating support ${floating.spec.model} has no reviewed retaining-ring seat';
		ringSeatZ = seat;
		ring = machinekit.standard.RetainingRing.forShaft(floating.boreDiameter);
	}

	/** Mill Z: M80E-BK15-C / BF15, braked ELM2H-0750 servo, 48 mm coupling. */
	public static function m80eBk15():MillScrewDrive
		return new MillScrewDrive(MotorMount.of("M80E-BK15-C", null, "hzmotion"), ScrewSupportUnit.model("BF15", "hzmotion"),
			LeadshineServo.model("ELM2H-0750LA80E"),
			48, 17);

	/** Small mill Y: M60D-BK12-C / BF12, ELM2H-0400 servo, 25 × 32 clamp coupling. */
	public static function m60dBk12():MillScrewDrive
		return new MillScrewDrive(MotorMount.of("M60D-BK12-C", null, "hzmotion"), ScrewSupportUnit.model("BF12", "hzmotion"),
			LeadshineServo.model("ELM2H-0400LA60F"),
			32, 16, 25, [{z: 8, angle: Math.PI / 2}, {z: 24, angle: Math.PI / 2}]);
}
