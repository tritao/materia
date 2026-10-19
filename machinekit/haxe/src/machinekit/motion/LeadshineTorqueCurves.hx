package machinekit.motion;

typedef LeadshineTorquePoint = { var rpm:Float; var torqueNm:Float; }

/** Raster evidence is distinct from a verified motor capability. Speeds are rpm. */
typedef LeadshineTorqueCurve = {
	var id:String;
	var baseDesignation:String;
	var matchedDesignation:Null<String>;
	var pageVariant:Null<String>;
	var source:String;
	var imageSource:String;
	var imageSha256:String;
	var status:String;
	var operatingConditionsVerified:Bool;
	var intermittentDurationSeconds:Null<Float>;
	var review:String;
	var torqueReadingUncertaintyNm:Float;
	var speedReadingUncertaintyRpm:Float;
	var continuous:Array<LeadshineTorquePoint>;
	var intermittent:Array<LeadshineTorquePoint>;
}

class LeadshineTorqueCurves {
	/** Fresh records so callers cannot mutate shared qualification state. */
	public static function all():Array<LeadshineTorqueCurve> return LeadshineTorqueCurveData.rows();
	public static function evidenceFor(designation:String):Array<LeadshineTorqueCurve>
		return [for (curve in all()) if (curve.baseDesignation == designation || curve.matchedDesignation == designation) curve];

	/** Only identified variants are eligible for an explicitly conditional study. */
	public static function studyCurve(designation:String):Null<LeadshineTorqueCurve> {
		for (curve in all()) if (curve.matchedDesignation == designation) return curve;
		return null;
	}

	/** Returns null outside the graph; never extrapolate torque beyond captured speeds.
	 * Reading allowance is subtracted for a conservative ordinate, but does not bound
	 * unknown drive/cooling conditions or uncertainty in a schematic graph's axes.
	 */
	public static function studyTorque(curve:LeadshineTorqueCurve, rpm:Float, intermittent:Bool = false):Null<Float> {
		if (!Math.isFinite(rpm) || rpm < 0) return null;
		var points = intermittent ? curve.intermittent : curve.continuous;
		if (points.length < 2 || rpm < points[0].rpm || rpm > points[points.length-1].rpm) return null;
		var slope = 0.0;
		for (i in 1...points.length) slope = Math.max(slope,Math.abs((points[i].torqueNm-points[i-1].torqueNm)/(points[i].rpm-points[i-1].rpm)));
		for (i in 1...points.length) if (rpm <= points[i].rpm) {
			var a = points[i-1], b = points[i];
			return Math.max(0, a.torqueNm + (b.torqueNm-a.torqueNm)*(rpm-a.rpm)/(b.rpm-a.rpm)
				-curve.torqueReadingUncertaintyNm-slope*curve.speedReadingUncertaintyRpm);
		}
		return null;
	}

	/** Qualification requires both exact variant match and verified operating conditions. */
	public static function qualifiedContinuousTorque(designation:String, rpm:Float):Null<Float> {
		var curve = studyCurve(designation);
		return curve == null || !curve.operatingConditionsVerified ? null : studyTorque(curve,rpm);
	}
}
