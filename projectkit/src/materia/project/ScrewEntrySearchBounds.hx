package materia.project;

/** Shared bounded fallback sizing for force-guided screw entry. */
class ScrewEntrySearchBounds {
	static inline final MAXIMUM_REFINEMENT_TURNS:Float = 10.0;
	static inline final REFINEMENT_TURN_MULTIPLE:Float = 2.5;
	static inline final MAXIMUM_FALLBACK_TURNS:Float = 14.0;
	static inline final FALLBACK_TURN_MULTIPLE:Float = 4.0;

	static function boundedTurns(initialTurns:Float, multiple:Float, maximum:Float):Float {
		if (!Math.isFinite(initialTurns) || !(initialTurns > 0))
			throw "Screw-entry refinement needs positive finite initial spiral turns";
		return Math.min(maximum, initialTurns * multiple);
	}

	static function durationSeconds(radius:Float, turns:Float, maximumLateralSpeed:Float):Float {
		if (!Math.isFinite(radius) || !(radius > 0) || !Math.isFinite(maximumLateralSpeed) || !(maximumLateralSpeed > 0))
			throw "Screw-entry refinement needs a positive finite radius and lateral speed";
		return radius * (1.0 + 2.0 * Math.PI * turns) / maximumLateralSpeed;
	}

	public static function minimumSpiralSeconds(approachMeters:Float, reliefMeters:Float, axialSpeed:Float):Float {
		if (!Math.isFinite(approachMeters) || approachMeters < 0 || !Math.isFinite(reliefMeters) || reliefMeters < 0 ||
			!Math.isFinite(axialSpeed) || !(axialSpeed > 0))
			throw "Screw-entry spiral needs a non-negative approach and relief and a positive axial speed";
		return (approachMeters + reliefMeters) / axialSpeed;
	}

	/** One catch-up window plus a bounded retract and return to the search depth. */
	public static function catchupRecoverySeconds(reliefMeters:Float, axialSpeed:Float,
		candidateProgressTimeoutSeconds:Float):Float {
		if (!Math.isFinite(reliefMeters) || reliefMeters < 0 || !Math.isFinite(axialSpeed) || !(axialSpeed > 0) ||
			!Math.isFinite(candidateProgressTimeoutSeconds) || !(candidateProgressTimeoutSeconds > 0))
			throw "Screw-entry catch-up recovery needs finite non-negative relief and positive speed/time bounds";
		return candidateProgressTimeoutSeconds + 2.0 * reliefMeters / axialSpeed;
	}

	/** Retract by at most the relief distance without passing the commissioned approach clearance. */
	public static function catchupReliefDepthMeters(currentDepthMeters:Float, reliefMeters:Float):Float {
		if (!Math.isFinite(currentDepthMeters) || !Math.isFinite(reliefMeters) || !(reliefMeters > 0))
			throw "Screw-entry catch-up relief needs a finite measured depth and positive clearance";
		return Math.max(-reliefMeters, currentDepthMeters - reliefMeters);
	}

	/** Maximum TCP displacement from the approach origin, including the registered lateral region and one servo step. */
	public static function requiredTravelMeters(approachMeters:Float, reliefMeters:Float,
		halfUMeters:Float, halfVMeters:Float, contactSeedOffsetUMeters:Float,
		spiralRadiusMeters:Float, freshnessDriftMeters:Float, maximumLinearStepMeters:Float):Float {
		if (!Math.isFinite(approachMeters) || approachMeters < 0 || !Math.isFinite(reliefMeters) || reliefMeters < 0 ||
			!Math.isFinite(halfUMeters) || halfUMeters < 0 || !Math.isFinite(halfVMeters) || halfVMeters < 0 ||
			!Math.isFinite(contactSeedOffsetUMeters) || !Math.isFinite(spiralRadiusMeters) || spiralRadiusMeters < 0 ||
			!Math.isFinite(freshnessDriftMeters) || freshnessDriftMeters < 0 ||
			!Math.isFinite(maximumLinearStepMeters) || maximumLinearStepMeters < 0)
			throw "Screw-entry travel needs finite non-negative CAD, relief, search and servo-step bounds";
		var lateralRegion = Math.sqrt(halfUMeters * halfUMeters + halfVMeters * halfVMeters);
		var seededSearch = Math.abs(contactSeedOffsetUMeters) + spiralRadiusMeters + freshnessDriftMeters;
		var lateralTravel = Math.max(lateralRegion, seededSearch);
		var axialTravel = Math.max(approachMeters, reliefMeters);
		return Math.sqrt(axialTravel * axialTravel + lateralTravel * lateralTravel) + maximumLinearStepMeters;
	}

	/** Conservative end-to-end bound, including lateral search time beyond the axial approach. */
	public static function minimumRunSeconds(approachMeters:Float, reliefMeters:Float, axialSpeed:Float,
		spiralSeconds:Float, radius:Float, initialTurns:Float, maximumLateralSpeed:Float,
		candidateProgressTimeoutSeconds:Float):Float {
		if (!Math.isFinite(candidateProgressTimeoutSeconds) || !(candidateProgressTimeoutSeconds > 0))
			throw "Screw-entry run needs a positive finite candidate-progress timeout";
		var refinementSeconds = refinementDurationSeconds(radius / 2.0, initialTurns, maximumLateralSpeed);
		var fallbackSeconds = fallbackDurationSeconds(radius, initialTurns, maximumLateralSpeed);
		var axialSearchSeconds = (3.0 * approachMeters + 2.0 * reliefMeters) / axialSpeed;
		var catchupRecovery = catchupRecoverySeconds(reliefMeters, axialSpeed, candidateProgressTimeoutSeconds);
		// The initial contact-plane spiral runs with axial force held, so its full
		// duration adds to the axial approach, candidate checks and registered sweeps.
		// The three approach traversals already cover the recovery's return to depth;
		// reserve its catch-up timeout and one complete retract-and-reposition relief.
		return axialSearchSeconds + 6.0 * candidateProgressTimeoutSeconds + refinementSeconds + fallbackSeconds +
			spiralSeconds + catchupRecovery;
	}

	public static function refinementTurns(initialTurns:Float):Float
		return boundedTurns(initialTurns, REFINEMENT_TURN_MULTIPLE, MAXIMUM_REFINEMENT_TURNS);

	public static function refinementDurationSeconds(radius:Float, initialTurns:Float, maximumLateralSpeed:Float):Float
		return durationSeconds(radius, refinementTurns(initialTurns), maximumLateralSpeed);

	public static function fallbackTurns(initialTurns:Float):Float
		return boundedTurns(initialTurns, FALLBACK_TURN_MULTIPLE, MAXIMUM_FALLBACK_TURNS);

	public static function fallbackDurationSeconds(radius:Float, initialTurns:Float, maximumLateralSpeed:Float):Float
		return durationSeconds(radius, fallbackTurns(initialTurns), maximumLateralSpeed);

	/** Worst-case time not overlapped by the axial approach and the lateral spiral. */
	public static function spiralOverrunSeconds(spiralSeconds:Float, axialApproachSeconds:Float):Float {
		if (!Math.isFinite(spiralSeconds) || spiralSeconds < 0 || !Math.isFinite(axialApproachSeconds) || axialApproachSeconds < 0)
			throw "Screw-entry spiral and axial approach times must be finite and non-negative";
		return Math.max(0.0, spiralSeconds - axialApproachSeconds);
	}
}
