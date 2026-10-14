package materia.project;

/** What a force-guided screw entry's travel and run bounds must cover. Lengths are metres, speeds m/s. */
class ScrewEntrySearchBounds {
	/** Largest TCP displacement from the approach pose: the CAD approach plus the entry advance, the registered region, one servo step. */
	public static function requiredTravelMeters(approachMeters:Float, advanceMeters:Float, halfUMeters:Float,
		halfVMeters:Float, maximumLinearStepMeters:Float):Float {
		for (value in [approachMeters, advanceMeters, halfUMeters, halfVMeters, maximumLinearStepMeters])
			if (!Math.isFinite(value) || value < 0)
				throw "Screw-entry travel needs finite non-negative approach, entry advance, region and servo-step lengths";
		var axial = approachMeters + advanceMeters;
		return Math.sqrt(axial * axial + halfUMeters * halfUMeters + halfVMeters * halfVMeters) + maximumLinearStepMeters;
	}

	/** Path length of an Archimedean spiral from its centre out to `radius`, turns `pitch` apart. */
	public static function spiralLengthMeters(radius:Float, pitch:Float):Float {
		if (!Math.isFinite(radius) || !(radius > 0) || !Math.isFinite(pitch) || !(pitch > 0))
			throw "Screw-entry spiral needs a positive radius and pitch";
		var scale = pitch / (2.0 * Math.PI), angle = radius / scale;
		var root = Math.sqrt(1.0 + angle * angle);
		return 0.5 * scale * (angle * root + Math.log(angle + root));
	}

	/** The guarded descent over the approach and the entry advance, plus one complete spiral at its path speed. */
	public static function minimumRunSeconds(approachMeters:Float, advanceMeters:Float, approachSpeed:Float,
		spiralRadius:Float, spiralPitch:Float, spiralSpeed:Float):Float {
		if (!Math.isFinite(approachSpeed) || !(approachSpeed > 0) || !Math.isFinite(spiralSpeed) || !(spiralSpeed > 0) ||
			!Math.isFinite(approachMeters) || approachMeters < 0 || !Math.isFinite(advanceMeters) || advanceMeters < 0)
			throw "Screw-entry run needs a non-negative approach and positive speeds";
		return (approachMeters + advanceMeters) / approachSpeed + spiralLengthMeters(spiralRadius, spiralPitch) / spiralSpeed;
	}
}
