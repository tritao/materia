package machinekit.standard;

/** Fit intent for a bearing's shaft journal. Allowances are diametral. */
enum BearingShaftFit {
	Slip;
	Transition;
	Interference;
}

/** Fit intent for a bearing's housing seat. Allowances are diametral. */
enum BearingHousingFit {
	Slip;
	Transition;
	Interference;
}

/**
 * Generic layout allowances for bearing fits.
 *
 * These values make fit intent explicit in a generated layout. They are not
 * ISO 286 tolerance limits; production drawings should replace them with the
 * selected bearing, shaft, housing material, temperature and load analysis.
 */
class BearingFit {
	/** Base diametral allowance grows with the bearing nominal diameter band. */
	static function baseAllowance(nominal:Float):Float {
		if (!(nominal > 0) || !Math.isFinite(nominal)) throw "Bearing fit nominal must be positive";
		return nominal <= 10 ? 0.01 : nominal <= 18 ? 0.015 : nominal <= 30 ? 0.02 : 0.03;
	}

	public static function shaftAllowance(fit:BearingShaftFit, nominal:Float):Float {
		var allowance = baseAllowance(nominal);
		return switch (fit) {
			case Slip: -allowance;
			case Transition: 0;
			case Interference: allowance;
		}
	}

	public static function housingAllowance(fit:BearingHousingFit, nominal:Float):Float {
		var allowance = baseAllowance(nominal) * 2.5;
		return switch (fit) {
			case Slip: allowance;
			case Transition: allowance / 2;
			case Interference: -baseAllowance(nominal);
		}
	}
}
