package machinekit.design;

import cadkit.threads.ThreadSpec;

enum abstract InsertMaterial(String) to String {
	var Steel = "steel";
}

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
	public static inline var STEEL_POLICY_ID = "steel-threaded-hole-v1";
	public static inline var STEEL_ENGAGEMENT_DIAMETERS = 1.0;
	public static inline var TIP_GAP_PITCHES = 1.0;
	public static inline var BACK_WALL_DIAMETERS = 0.25;
	public static inline var RADIAL_WALL_DIAMETERS = 0.5;
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
			policyId: limits.policyId, minEngagement: limits.minEngagement, minTipGap: limits.minTipGap,
			minBack: limits.minBack, minWall: limits.minWall, minClearance: limits.minClearance, overrides: overrides
		};
	}

	/** Unthreaded bores retain numeric design limits; engagement is inapplicable to their checks. */
	public static function clearance(diameter:Float):materia.casting.InsertHole.InsertHoleLimits {
		if (!Math.isFinite(diameter) || diameter <= 0)
			throw "Clearance hole policy requires a positive diameter";
		return {
			policyId: "steel-clearance-hole-v1", minEngagement: diameter,
			minTipGap: diameter * CLEARANCE_DIAMETERS, minBack: diameter * BACK_WALL_DIAMETERS,
			minWall: diameter * RADIAL_WALL_DIAMETERS, minClearance: diameter * CLEARANCE_DIAMETERS, overrides: []
		};
	}

	public static function resolve(spec:ThreadSpec, material:InsertMaterial, ?overrides:HoleLimitOverrides):ResolvedHoleLimits {
		if (spec == null || !spec.internal || !Math.isFinite(spec.majorDiameter)
			|| !Math.isFinite(spec.pitch) || spec.majorDiameter <= 0 || spec.pitch <= 0)
			throw "Threaded hole policy requires a valid internal thread";
		if (material != Steel)
			throw "Unsupported threaded hole material: " + material;
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
