package machinekit.design;

import cadkit.threads.ThreadSpec;

typedef InsertMaterial = materia.casting.InsertMaterial;

/** Nominal thread paired with its declared machining and clearance sizes. */
typedef ThreadedHoleChoice = {
	var thread:ThreadSpec;
	var tapDrill:Float;
	var clearanceDiameter:Float;
}

typedef HoleLimitOverrides = {
	@:optional var minEngagement:Float;
	@:optional var minTipGap:Float;
	@:optional var minBack:Float;
	@:optional var minWall:Float;
	@:optional var minClearance:Float;
}

typedef ResolvedHoleLimits = {
	var policyId:String;
	var minEngagement:Float;
	var minTipGap:Float;
	var minBack:Float;
	var minWall:Float;
	var minClearance:Float;
	var overrides:HoleLimitOverrides;
}

/** Design minima, independent of anchorage or coupon evidence. Distances are mm. */
class ThreadedHolePolicy {
	public static inline var CLEARANCE_POLICY_ID = "c45-clearance-hole-v2";
	public static inline var STEEL_POLICY_ID = "c45-iso4762-8.8-vdi2230-v2";
	/** VDI 2230 Part 1 theoretical 1.3D for Rm >500 MPa / class 8.8,
	 * conservatively using that strength band for C45 bright Rm >=580 MPa.
	 * Bossard, Length of engaged thread, 01-2025, bracketed VDI values:
	 * https://assets.eu.ctfassets.net/0vp0u5uh75zd/3wPlDC2XQOYT3XOdwCk6lz/9a9282ffd29746a7a7f6b0149509f1cf/052_Length_engaged_thread_Fastening_EN_01_2025.pdf
	 * Effective full-thread overlap excludes runout and chamfer.
	 */
	public static inline var STEEL_ENGAGEMENT_DIAMETERS = 1.3;
	/** In-house machining allowance: two pitches for a bottoming tap runout.
	 * Usable full thread ends before this zone; drill depth includes it. */
	public static inline var TAP_RUNOUT_PITCHES = 2.0;
	/** In-house: one pitch of free usable thread beyond the screw tip. */
	public static inline var TIP_GAP_PITCHES = 1.0;
	/** In-house: a quarter diameter of intact steel closes the resin path. */
	public static inline var BACK_WALL_DIAMETERS = 0.25;
	/** In-house: one diameter of radial steel limits edge breakout and dilation;
	 * this geometric rule does not establish anchorage capacity in the casting. */
	public static inline var RADIAL_WALL_DIAMETERS = 1.0;
	/** In-house minimum separation between machined features; not bolt clearance. */
	public static inline var CLEARANCE_DIAMETERS = 0.1;

	public static function neutral(limits:ResolvedHoleLimits):materia.casting.InsertHole.InsertHoleLimits {
		var overrides:Array<{name:String, value:Float}> = [];
		for (name in Reflect.fields(limits.overrides)) {
			var value:Null<Float> = Reflect.field(limits.overrides, name);
			if (value != null)
				overrides.push({name: name, value: value});
		}
		overrides.sort((a, b) -> Reflect.compare(a.name, b.name));
		return {
			policyId: limits.policyId, threaded: {minEngagement: limits.minEngagement, minTipGap: limits.minTipGap},
			minBack: limits.minBack, minWall: limits.minWall, minClearance: limits.minClearance, overrides: overrides
		};
	}

	/** Unthreaded bores carry only applicable geometric rules. */
	public static function clearance(diameter:Float):materia.casting.InsertHole.InsertHoleLimits {
		if (!Math.isFinite(diameter) || diameter <= 0)
			throw "Clearance hole policy requires a positive diameter";
		return {
			policyId: CLEARANCE_POLICY_ID, threaded: null, minBack: diameter * BACK_WALL_DIAMETERS,
			minWall: diameter * RADIAL_WALL_DIAMETERS, minClearance: diameter * CLEARANCE_DIAMETERS, overrides: []
		};
	}

	public static function resolve(spec:ThreadSpec, material:InsertMaterial, ?overrides:HoleLimitOverrides):ResolvedHoleLimits {
		if (material != materia.casting.InsertMaterial.C45Bright && material != materia.casting.InsertMaterial.C45Ground)
			throw "C45 threaded hole policy requires C45 stock; purchased sockets need their own declared policy";
		if (spec == null || !spec.internal || !Math.isFinite(spec.majorDiameter)
			|| !Math.isFinite(spec.pitch) || spec.majorDiameter <= 0 || spec.pitch <= 0)
			throw "Threaded hole policy requires a valid internal thread";
		if (materia.casting.InsertMaterial.InsertMaterials.specification(material).minimumTensileStrengthMPa <= 500)
			throw "Threaded hole policy requires steel with Rm >500 MPa";
		var recorded:HoleLimitOverrides = {
			minEngagement: null, minTipGap: null, minBack: null, minWall: null, minClearance: null
		};
		var supplied:HoleLimitOverrides = overrides == null ? recorded : overrides;
		var result:ResolvedHoleLimits = {
			policyId: STEEL_POLICY_ID,
			minEngagement: spec.majorDiameter * STEEL_ENGAGEMENT_DIAMETERS,
			minTipGap: spec.pitch * TIP_GAP_PITCHES,
			minBack: spec.majorDiameter * BACK_WALL_DIAMETERS,
			minWall: spec.majorDiameter * RADIAL_WALL_DIAMETERS,
			minClearance: spec.majorDiameter * CLEARANCE_DIAMETERS,
			overrides: recorded
		};
		for (field in Reflect.fields(supplied)) {
			if (["minEngagement", "minTipGap", "minBack", "minWall", "minClearance"].indexOf(field) < 0)
				throw "Unknown threaded hole limit override: " + field;
			if (Reflect.field(supplied, field) == null)
				continue;
			var value:Float = Reflect.field(supplied, field);
			if (!Math.isFinite(value) || value <= 0)
				throw "Threaded hole limit override must be finite and positive: " + field;
			Reflect.setField(recorded, field, value);
			Reflect.setField(result, field, value);
		}
		return result;
	}
}
