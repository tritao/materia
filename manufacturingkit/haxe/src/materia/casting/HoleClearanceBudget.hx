package materia.casting;

import materia.casting.InsertManufacturing.InsertHoleRoute;

/** Manufacturing error allocations, separate from the numeric hole design limits.
 * In-plane magnitudes refer to the insert XY datum origin; the angular term
 * acts over each hole's actual radial distance from that origin.
 * Null denotes an unresolved accuracy input, never an unknown thread policy.
 */
typedef HoleClearanceBudget = {
	var boltDiameterMm:Float;
	var clearanceDiameterMm:Null<Float>;
	var distanceFromDatumMm:Float;
	var placementOffsetMm:Null<Float>;
	var placementAngleRad:Null<Float>;
	var assemblyAdjustmentMm:Null<Float>;
	var cureBowMm:Null<Float>;
	var machiningErrorMm:Null<Float>;
	var pitchAccumulationMm:Null<Float>;
	var requiredMarginMm:Null<Float>;
}

typedef HoleClearanceAssessment = {
	var beforeCastingMarginMm:Null<Float>;
	var transferMarginMm:Null<Float>;
	var missingInputs:Array<String>;
	var missingTransferInputs:Array<String>;
}

enum HoleRouteDecision {
	UsePreCast(marginMm:Float);
	UseTransfer(marginMm:Float);
	ProposeTransfer(missingInputs:Array<String>);
	AwaitInputs(missingInputs:Array<String>);
	ChangeDesign(reason:String);
}

class HoleClearanceBudgets {
	public static function snapshot(raw:Dynamic):HoleClearanceBudget {
		CastingFeatureCodec.fields(raw, ["boltDiameterMm", "clearanceDiameterMm", "distanceFromDatumMm",
			"placementOffsetMm", "placementAngleRad", "assemblyAdjustmentMm", "cureBowMm",
			"machiningErrorMm", "pitchAccumulationMm", "requiredMarginMm"]);
		function nonnegative(name:String):Float {
			var value = CastingRecordCodec.number(raw, name);
			if (value < 0) throw "Negative hole clearance budget: " + name;
			return value;
		}
		function optional(name:String):Null<Float>
			return Reflect.field(raw, name) == null ? null : nonnegative(name);
		var result:HoleClearanceBudget = {
			boltDiameterMm: nonnegative("boltDiameterMm"),
			clearanceDiameterMm: optional("clearanceDiameterMm"),
			distanceFromDatumMm: nonnegative("distanceFromDatumMm"),
			placementOffsetMm: optional("placementOffsetMm"),
			placementAngleRad: optional("placementAngleRad"),
			assemblyAdjustmentMm: optional("assemblyAdjustmentMm"),
			cureBowMm: optional("cureBowMm"),
			machiningErrorMm: optional("machiningErrorMm"),
			pitchAccumulationMm: optional("pitchAccumulationMm"),
			requiredMarginMm: optional("requiredMarginMm")
		};
		if (result.boltDiameterMm <= 0 || (result.clearanceDiameterMm != null && result.clearanceDiameterMm <= 0))
			throw "Hole clearance budget diameters must be positive";
		if (result.placementAngleRad != null && result.placementAngleRad > Math.PI)
			throw "Hole placement angle exceeds pi radians";
		return result;
	}

	/** Worst-case vector magnitudes add; no statistical independence is assumed.
	 * Rotation contributes the chord displacement from the actual datum lever arm.
	 * Transfer drilling removes casting placement, adjustment, bow and pitch error;
	 * its drill/guide error and the required clearance margin still apply.
	 */
	public static function assess(raw:HoleClearanceBudget):HoleClearanceAssessment {
		var budget = snapshot(raw), missing:Array<String> = [];
		for (name in ["clearanceDiameterMm", "placementOffsetMm", "placementAngleRad", "assemblyAdjustmentMm",
			"cureBowMm", "machiningErrorMm", "pitchAccumulationMm", "requiredMarginMm"])
			if (Reflect.field(budget, name) == null) missing.push(name);
		var clearance = budget.clearanceDiameterMm, machining = budget.machiningErrorMm, margin = budget.requiredMarginMm;
		var transfer:Null<Float> = clearance == null || machining == null || margin == null
			? null : (clearance - budget.boltDiameterMm) / 2 - machining - margin;
		var before:Null<Float> = null;
		if (missing.length == 0) {
			var offset:Float = cast budget.placementOffsetMm, angle:Float = cast budget.placementAngleRad;
			var adjustment:Float = cast budget.assemblyAdjustmentMm, bow:Float = cast budget.cureBowMm;
			var pitch:Float = cast budget.pitchAccumulationMm;
			var available:Float = cast transfer;
			before = available - offset - 2 * budget.distanceFromDatumMm * Math.sin(angle / 2)
				- adjustment - bow - pitch;
		}
		return {beforeCastingMarginMm: before, transferMarginMm: transfer, missingInputs: missing,
			missingTransferInputs: [for (name in ["clearanceDiameterMm", "machiningErrorMm", "requiredMarginMm"])
				if (Reflect.field(budget, name) == null) name]};
	}

	public static function choose(raw:HoleClearanceBudget, permitted:Array<InsertHoleRoute>):HoleRouteDecision {
		if (permitted == null) throw "Missing permitted hole routes";
		var seen:Array<InsertHoleRoute> = [];
		for (route in permitted) {
			if (route != InsertHoleRoute.BeforeCasting && route != InsertHoleRoute.TransferAfterCasting)
				throw "Unknown permitted hole route";
			if (seen.indexOf(route) >= 0) throw "Duplicate permitted hole route";
			seen.push(route);
		}
		var result = assess(raw), before = result.beforeCastingMarginMm, transfer = result.transferMarginMm;
		var allowBefore = permitted.indexOf(InsertHoleRoute.BeforeCasting) >= 0;
		var allowTransfer = permitted.indexOf(InsertHoleRoute.TransferAfterCasting) >= 0;
		if (allowBefore && before != null && before >= 0) return UsePreCast(before);
		if (allowTransfer) {
			if (transfer == null) return ProposeTransfer(result.missingTransferInputs);
			if (transfer >= 0) return UseTransfer(transfer);
		}
		// Every omitted pre-cast term is nonnegative. A negative transfer margin
		// therefore also proves the pre-cast route cannot close.
		if (allowBefore && before == null && (transfer == null || transfer >= 0))
			return AwaitInputs(result.missingInputs);
		return ChangeDesign("No permitted hole route closes the bolt-clearance budget");
	}
}
