package machinekit.robotics;

/** Assumed reference envelopes for two-jaw pneumatic grippers. Force is the
 * matched cylinder's extension force at 0.6 MPa; dimensions and speed are
 * generic catalogue values, not a vendor's verified row. */
class ParallelGripperSpec {
	/** Assumed side clearance between the housing and the open-jaw envelope. */
	public static inline var HOUSING_SIDE_CLEARANCE:Float = 1.0;
	/** Assumed carriage tongue running on the housing's relieved guide floor. */
	public static inline var GUIDE_TONGUE_REACH:Float = 3.0;
	public static inline var GUIDE_TONGUE_HEIGHT:Float = 2.0;
	/** Assumed side bracket plate thickness and attachment overlap on the housing. */
	public static inline var CYLINDER_BRACKET_THICKNESS:Float = 3.0;
	public static inline var CYLINDER_BRACKET_LAP:Float = 5.0;
	/** Nominal endpoint switch window and clearance between it and width feedback, in mm. */
	public static inline var SWITCH_EDGE_CLEARANCE:Float = 0.5;
	/** Assumed static/kinetic rubber-to-workpiece contact coefficient for the replaceable pads. */
	public static inline var FINGER_PAD_FRICTION_COEFFICIENT:Float = 1.5;
	/** Inward reach of each optional support step beneath the blank. */
	public static inline var FINGER_SUPPORT_STEP_PROJECTION:Float = 6.0;
	/** Small overlap joins the step to the finger without leaving a collision seam. */
	public static inline var FINGER_SUPPORT_STEP_ROOT_OVERLAP:Float = 0.1;
	public final size:Int;
	public final bodyWidth:Float;
	public final bodyDepth:Float;
	public final bodyLength:Float;
	public final jawWidth:Float;
	public final jawHeight:Float;
	public final totalJawTravel:Float;
	public final openingTravel:Float;
	public final expectedPartWidth:Float;
	public final fingerWidth:Float;
	public final fingerDepth:Float;
	public final fingerLength:Float;
	public final fingerPadFrictionCoefficient:Float;
	/** Whether replaceable fingers have inward toes beneath the fingertip plane. */
	public final supportStep:Bool;
	public final ratedGripForce:Float;
	public final assumptions:Array<String>;
	public final cylinderFamily:String;
	public final cylinderBore:Float;
	public final cylinderSpeed:Float;

	public function new(size:Int, expectedPartWidth:Float = 60.0, fingerLength:Float = 60.0,
			openingTravel:Float = 16.0, supportStep:Bool = false, ?fingerDepth:Float, ?fingerWidth:Float) {
		if (size != 25 && size != 32 && size != 40)
			throw 'Unsupported assumed gripper size $size; use 25, 32 or 40 mm';
		if (!(expectedPartWidth > openingTravel) || !(fingerLength > 0) || !(openingTravel > 2.0) ||
			!Math.isFinite(expectedPartWidth + fingerLength + openingTravel))
			throw "Gripper width and finger length must be finite and positive, with more than 2 mm of opening travel";
		this.size = size;
		this.bodyDepth = size == 25 ? 48 : size == 32 ? 50 : 58;
		var actualFingerDepth = fingerDepth == null ? bodyDepth : fingerDepth;
		if (!(actualFingerDepth > 0 && actualFingerDepth <= bodyDepth) || !Math.isFinite(actualFingerDepth))
			throw 'Gripper contact finger depth must be positive and fit its ${bodyDepth} mm housing';
		this.bodyLength = 80;
		this.jawWidth = 10;
		this.jawHeight = 20;
		var actualFingerWidth = fingerWidth == null ? 8.0 : fingerWidth;
		if (!(actualFingerWidth > 0 && actualFingerWidth <= jawWidth) || !Math.isFinite(actualFingerWidth))
			throw 'Gripper contact finger width must be positive and fit its ${jawWidth} mm jaw';
		this.totalJawTravel = openingTravel * 2;
		this.openingTravel = openingTravel;
		this.expectedPartWidth = expectedPartWidth;
		this.fingerWidth = actualFingerWidth;
		// Keep the replaceable pad within the housing's depth envelope. Extending
		// it across the nominal blank width would run into the vise jaws at load.
		this.fingerDepth = actualFingerDepth;
		this.fingerLength = fingerLength;
		this.fingerPadFrictionCoefficient = FINGER_PAD_FRICTION_COEFFICIENT;
		this.supportStep = supportStep;
		this.cylinderFamily = size == 25 ? "ISO6432" : "ISO15552";
		this.cylinderBore = size;
		this.cylinderSpeed = 300;
		var catalogueWidth = size == 25 ? 72.0 : size == 32 ? 80.0 : 92.0;
		// Keep the housing inside the jaw's swept face envelope. The carriage and
		// piston rod then run beside the housing instead of through its collision hull.
		this.bodyWidth = Math.min(catalogueWidth,
			expectedPartWidth + openingTravel - 2 * HOUSING_SIDE_CLEARANCE);
		var cylinder = this.cylinder();
		this.ratedGripForce = cylinder.extendForce(600000);
		this.assumptions = ["generic gripper envelope", "cylinder bore matched to gripper size",
			"housing has at least 1 mm clearance from each open-jaw face",
			'contact finger ${this.fingerWidth} x ${this.fingerDepth} mm (assumed, replaceable)',
			"600 kPa supply", "300 mm/s flow-limited cylinder speed",
			'finger-pad friction coefficient ${FINGER_PAD_FRICTION_COEFFICIENT} (assumed)',
			supportStep
				? 'fingertip support step ${FINGER_SUPPORT_STEP_PROJECTION} mm reach (assumed)'
				: "flat replaceable fingers, no support step"];
	}

	/** The jaw's mechanical travel is half the total change in the opening. */
	public function jawTravel():Float return totalJawTravel / 2.0;

	/** The only physical cylinder specification used by the gripper assembly and housing frames. */
	public function cylinder():machinekit.pneumatic.PneumaticCylinder
		return new machinekit.pneumatic.PneumaticCylinder(cylinderFamily, cylinderBore, jawTravel(), cylinderSpeed);

	/** Part-width limits derived from the open jaw gap and the cylinder's mirrored travel. */
	public function minimumPartWidth():Float return expectedPartWidth - openingTravel;
	public function maximumPartWidth():Float return expectedPartWidth + openingTravel;

	/** Jaw positions that correspond to a supported part width, excluding the open and empty end stops. */
	public function grippedTravelWindow():{lower:Float, upper:Float} {
		var openGap = maximumPartWidth();
		var endTravel = (openGap - minimumPartWidth()) / 2.0;
		var lower = 2.0 * SWITCH_EDGE_CLEARANCE;
		var upper = endTravel - 2.0 * SWITCH_EDGE_CLEARANCE;
		return {lower: lower, upper: upper};
	}

	/** Symmetric clearance envelope around the flange axis, including the inline cylinder and open jaws. */
	public function overallHalfWidth():Float {
		var cylinder = this.cylinder();
		return Math.max(bodyWidth / 2, Math.max(openJawCentre() + jawWidth / 2,
			openJawCentre() + jawWidth / 2 + cylinder.bodyLength + cylinder.stemLength));
	}

	/** Jaw centre distance at maximum opening, including the jaw carriage thickness. */
	public function openJawCentre():Float
		return expectedPartWidth / 2.0 + openingTravel / 2.0 + jawWidth / 2.0;

	public function supportSideReach():Float
		return Math.max(bodyDepth / 2, cylinder().bodyWidth / 2) + CYLINDER_BRACKET_THICKNESS;

	/** Wide commissioned openings need longer guide tongues to reach the housing. */
	public function guideTongueReach():Float
		return Math.max(GUIDE_TONGUE_REACH,
			openJawCentre() - jawWidth / 2 - bodyWidth / 2 + HOUSING_SIDE_CLEARANCE);

	/** Rod clears the guide floor throughout closure, and meets the carriage's outside face. */
	public function cylinderAxisZ():Float
		return bodyLength - jawHeight + cylinder().rod / 2 + HOUSING_SIDE_CLEARANCE;

	public function cylinderStartX():Float {
		var drive = cylinder();
		return -openJawCentre() - jawWidth / 2 - drive.bodyLength - drive.stemLength;
	}

	/** Valve mounting station in the housing's jaw-travel axis. */
	public function valveSeatX():Float return bodyWidth / 2.0 - 30.0;
}
