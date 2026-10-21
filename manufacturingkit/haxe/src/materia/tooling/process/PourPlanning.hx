package materia.tooling.process;

import materia.casting.CastingFeatureCodec;
import materia.casting.CastingRecordCodec;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;
import materia.tooling.process.FillAnalysis.FillReport;

enum abstract MixIngredientKind(String) to String {
	var Resin = "resin";
	var Hardener = "hardener";
	var Aggregate = "aggregate";
	var Additive = "additive";
}

typedef MixIngredient = {
	var id:String;
	var kind:MixIngredientKind;
	var materialReference:String;
	var massFraction:Float;
}

typedef MixComposition = {
	var reference:String;
	var cureReference:String;
	/** The heat-of-reaction denominator may be base resin or mixed binder.
	 * The source names the weighed ingredients belonging to that mass basis.
	 */
	var cureMassIngredientIds:Array<String>;
	var ingredients:Array<MixIngredient>;
}

/** Usable wet-mix mass, with sequential mixing, transfer and pouring.
 * Working time runs from the start of mixing. Parallel mixers need another model.
 */
typedef PourMixerSpec = {
	var reference:String;
	var capacityKg:Float;
	var mixingSeconds:Float;
	var transferSeconds:Float;
	var workingTimeSeconds:Float;
	var workingTimeReference:String;
}

typedef PourQuantity = {var ingredientId:String; var materialReference:String; var massKg:Float;}

typedef PourBatch = {
	var index:Int;
	var massKg:Float;
	var quantities:Array<PourQuantity>;
	var lowerHeightMm:Float;
	var upperHeightMm:Float;
	var mixingStartSeconds:Float;
	var pourStartSeconds:Float;
	var pourEndSeconds:Float;
}

typedef PourSchedule = {
	var batches:Array<PourBatch>;
	var totalElapsedSeconds:Float;
	/** Includes the age at which each following batch first contacts the prior one. */
	var minimumWorkingTimeMarginSeconds:Float;
}

typedef PourAssessment = {
	var sourceFingerprint:String;
	var fillProcessFingerprint:String;
	var mixingFingerprint:String;
	var totalMixMassKg:Float;
	var cureBasisMassKg:Null<Float>;
	var quantities:Array<PourQuantity>;
	var schedule:Null<PourSchedule>;
	var validation:Array<ToolingValidation>;
}

/** Uses the existing corrected fill curve, not another geometry/fill pipeline.
 * These are planning checks; the process pipeline retains fill, cure and physical
 * qualification checks. No supplier mix ratio, mixer or working time is defaulted.
 */
class PourPlanning {
	public static inline final MAX_BATCHES = 10000;
	static inline final RELATIVE_TOLERANCE = 1e-10;

	public static function analyze(sourceId:String, fill:FillReport, composition:Null<MixComposition>,
			mixer:Null<PourMixerSpec>):PourAssessment {
		materia.tooling.ToolingIds.validate(sourceId);
		var mass = validateCurve(fill);
		var recipe = composition == null ? null : copyComposition(composition);
		var equipment = mixer == null ? null : copyMixer(mixer);
		var result:PourAssessment = {
			sourceFingerprint: fill.sourceFingerprint, fillProcessFingerprint: fill.processFingerprint,
			mixingFingerprint: haxe.crypto.Sha256.encode(materia.tooling.ToolingCodec.canonical({composition: recipe, mixer: equipment})),
			totalMixMassKg: mass, cureBasisMassKg: recipe == null ? null : mass * cureMassFraction(recipe),
			quantities: quantities(recipe, mass), schedule: null, validation: []
		};
		result.validation.push(check(sourceId, "mix-quantities", recipe == null ? ToolingCheckStatus.Pending : ToolingCheckStatus.Pass,
			recipe == null ? "Declare the mix ingredients and fractions by weight." : "Material quantities use the declared fractions by weight."));
		if (equipment == null) {
			result.validation.push(check(sourceId, "batch-schedule", ToolingCheckStatus.Pending, "Declare usable mixer capacity, mixing/transfer times and working time."));
			return result;
		}
		var declaredMixer:PourMixerSpec = equipment;
		if (fill.model != "exact-orthogonal-flat-free-surface") {
			result.validation.push(check(sourceId, "batch-schedule", ToolingCheckStatus.Unsupported,
				"Batch level inversion requires an exact volume evaluator for this fill model; no linear approximation is used."));
			return result;
		}
		var end = fill.levels[fill.levels.length - 1];
		var duration = end.timeSeconds;
		if (duration == null) {
			result.validation.push(check(sourceId, "batch-schedule", ToolingCheckStatus.Pending, "Declare a pour rate to budget batch working time."));
			return result;
		}
		if (duration <= 0 || !Math.isFinite(duration)) throw "Invalid fill duration for batch planning";
		var durationSeconds:Float = duration;
		var flow = mass / durationSeconds;
		if (!Math.isFinite(flow) || flow <= 0) throw "Invalid fill mass flow";
		for (level in fill.levels) {
			var time = level.timeSeconds;
			if (time == null || !near(time * flow, level.mixMassKg, mass)) throw "Fill times disagree with the declared mass flow";
		}
		var dead = declaredMixer.mixingSeconds + declaredMixer.transferSeconds;
		var finalCapacity = Math.min(declaredMixer.capacityKg, flow * (declaredMixer.workingTimeSeconds - dead));
		if (finalCapacity <= 0) {
			result.validation.push(check(sourceId, "batch-schedule", ToolingCheckStatus.Fail, "Mixing and transfer consume the declared working time."));
			return result;
		}
		var regularCapacity = finalCapacity;
		if (mass > finalCapacity + RELATIVE_TOLERANCE * mass) {
			// The next fresh batch must arrive before the previous batch's deadline.
			regularCapacity = Math.min(declaredMixer.capacityKg, flow * (declaredMixer.workingTimeSeconds - 2 * dead));
			if (regularCapacity <= 0) {
				result.validation.push(check(sourceId, "batch-schedule", ToolingCheckStatus.Fail,
					"A sequential mixer cannot bring the next batch to the interface within the declared working time."));
				return result;
			}
		}
		var nonfinalRatio = mass <= finalCapacity + RELATIVE_TOLERANCE * mass ? 0.0
			: (mass - finalCapacity) / regularCapacity;
		// Math.ceil returns Int in Haxe: bound the ratio before that conversion.
		if (!Math.isFinite(nonfinalRatio) || nonfinalRatio > MAX_BATCHES - 1 + RELATIVE_TOLERANCE) {
			result.validation.push(check(sourceId, "batch-schedule", ToolingCheckStatus.Unsupported, "Batch count exceeds the supported planning limit."));
			return result;
		}
		var count = 1 + Math.ceil(Math.max(0, nonfinalRatio - RELATIVE_TOLERANCE));
		var batches:Array<PourBatch> = [], cumulative = 0.0, elapsed = 0.0, margin = declaredMixer.workingTimeSeconds;
		for (i in 0...count) {
			var amount = i == count - 1 ? mass - cumulative : Math.min(regularCapacity, mass - cumulative);
			var start = elapsed + dead, endTime = start + amount / flow;
			if (amount <= 0 || amount > declaredMixer.capacityKg + RELATIVE_TOLERANCE * mass) throw "Invalid computed batch mass";
			margin = Math.min(margin, declaredMixer.workingTimeSeconds - (endTime - elapsed));
			if (i > 0) margin = Math.min(margin, declaredMixer.workingTimeSeconds - (start - batches[i - 1].mixingStartSeconds));
			batches.push({index: i + 1, massKg: amount, quantities: quantities(recipe, amount),
				lowerHeightMm: heightAtMass(fill, cumulative), upperHeightMm: heightAtMass(fill, cumulative + amount),
				mixingStartSeconds: elapsed, pourStartSeconds: start, pourEndSeconds: endTime});
			cumulative += amount;
			elapsed = endTime;
		}
		if (!near(cumulative, mass, mass) || margin < -RELATIVE_TOLERANCE * declaredMixer.workingTimeSeconds)
			throw "Computed batch schedule exceeds its mass or working-time budget";
		result.schedule = {batches: batches, totalElapsedSeconds: elapsed, minimumWorkingTimeMarginSeconds: Math.max(0, margin)};
		result.validation.push(check(sourceId, "batch-schedule", ToolingCheckStatus.Pass,
			"Batch masses and sequential arrival times fit the declared capacity and working time; thermal/gel and physical qualification remain separate."));
		return result;
	}

	static function validateCurve(fill:FillReport):Float {
		if (fill == null || fill.levels == null || fill.levels.length < 2 || fill.totalMixVolumeMm3 <= 0
			|| !Math.isFinite(fill.totalMixVolumeMm3)) throw "Invalid fill curve for batch planning";
		var end = fill.levels[fill.levels.length - 1], mass = end.mixMassKg;
		if (!Math.isFinite(mass) || mass <= 0 || !near(end.mixVolumeMm3, fill.totalMixVolumeMm3, fill.totalMixVolumeMm3))
			throw "Fill curve does not reach its declared total";
		var previousHeight = fill.levels[0].heightMm - 1, previousMass = 0.0;
		for (level in fill.levels) {
			if (!Math.isFinite(level.heightMm) || level.heightMm <= previousHeight || !Math.isFinite(level.mixMassKg)
				|| level.mixMassKg < previousMass || !Math.isFinite(level.mixVolumeMm3) || level.mixVolumeMm3 < 0
				|| !near(level.mixMassKg, mass * level.mixVolumeMm3 / fill.totalMixVolumeMm3, mass))
				throw "Invalid monotone fill mass curve";
			previousHeight = level.heightMm;
			previousMass = level.mixMassKg;
		}
		if (!near(fill.levels[0].mixMassKg, 0, mass)) throw "Batch fill curve must start empty";
		return mass;
	}

	static function heightAtMass(fill:FillReport, mass:Float):Float {
		if (mass <= 0) return fill.levels[0].heightMm;
		for (i in 1...fill.levels.length) {
			var a = fill.levels[i - 1], b = fill.levels[i];
			if (mass <= b.mixMassKg) {
				if (b.mixMassKg == a.mixMassKg) return a.heightMm;
				return a.heightMm + (b.heightMm - a.heightMm) * (mass - a.mixMassKg) / (b.mixMassKg - a.mixMassKg);
			}
		}
		var end = fill.levels[fill.levels.length - 1];
		if (near(mass, end.mixMassKg, end.mixMassKg)) return end.heightMm;
		throw "Batch mass exceeds the corrected fill curve";
	}

	static function quantities(composition:Null<MixComposition>, mass:Float):Array<PourQuantity>
		return composition == null ? [] : [for (i in composition.ingredients)
			{ingredientId: i.id, materialReference: i.materialReference, massKg: mass * i.massFraction}];

	public static function copyComposition(raw:Dynamic):MixComposition {
		CastingFeatureCodec.fields(raw, ["reference", "cureReference", "cureMassIngredientIds", "ingredients"]);
		var ingredients:Array<MixIngredient> = [], ids:Map<String,Bool> = [], total = 0.0;
		var rows = CastingRecordCodec.array(raw, "ingredients");
		if (rows.length == 0 || rows.length > 128) throw "Invalid mix ingredient count";
		for (row in rows) {
			CastingFeatureCodec.fields(row, ["id", "kind", "materialReference", "massFraction"]);
			var id = CastingRecordCodec.string(row, "id"), fraction = CastingRecordCodec.number(row, "massFraction");
			CastingFeatureCodec.identifier(id);
			if (ids.exists(id) || fraction <= 0 || fraction > 1) throw "Invalid or duplicate mix ingredient: " + id;
			ids.set(id, true);
			var kind:MixIngredientKind = switch CastingRecordCodec.string(row, "kind") {
				case "resin": MixIngredientKind.Resin;
				case "hardener": MixIngredientKind.Hardener;
				case "aggregate": MixIngredientKind.Aggregate;
				case "additive": MixIngredientKind.Additive;
				default: throw "Unknown mix ingredient kind";
			};
			ingredients.push({id: id, kind: kind, materialReference: text(row, "materialReference"), massFraction: fraction});
			total += fraction;
		}
		if (Math.abs(total - 1) > RELATIVE_TOLERANCE) throw "Mix mass fractions must sum to one";
		ingredients.sort((a, b) -> Reflect.compare(a.id, b.id));
		var basis = CastingRecordCodec.identifiers(raw, "cureMassIngredientIds", 128);
		if (basis.length == 0) throw "Mix must declare its cure mass basis";
		for (id in basis) if (!ids.exists(id)) throw "Unknown ingredient in cure mass basis: " + id;
		return {reference: text(raw, "reference"), cureReference: text(raw, "cureReference"),
			cureMassIngredientIds: basis, ingredients: ingredients};
	}

	public static function cureMassFraction(composition:MixComposition):Float {
		var fraction = 0.0;
		for (ingredient in composition.ingredients)
			if (composition.cureMassIngredientIds.indexOf(ingredient.id) >= 0) fraction += ingredient.massFraction;
		return fraction;
	}

	/** Explicit denominator/reference agreement, separate from kinetics or calibration. */
	public static function cureConsistency(sourceId:String, composition:Null<MixComposition>,
			cureReference:String, resinMassFraction:Null<Float>):ToolingValidation {
		materia.tooling.ToolingIds.validate(sourceId);
		if (composition == null) return check(sourceId, "cure-mass-basis", ToolingCheckStatus.Pending,
			"Declare the weighed ingredients used by the cure model's heat-of-reaction mass basis.");
		var recipe = copyComposition(composition);
		if (recipe.cureReference != cureReference) return check(sourceId, "cure-mass-basis", ToolingCheckStatus.Fail,
			"The mix and cure model name different resin systems.");
		if (resinMassFraction == null) return check(sourceId, "cure-mass-basis", ToolingCheckStatus.Pending,
			"Declare the cure model's reactive mass fraction and heat-of-reaction denominator.");
		var fraction:Float = resinMassFraction;
		if (!Math.isFinite(fraction) || fraction < 0 || fraction > 1) throw "Invalid cure mass fraction";
		var agrees = near(fraction, cureMassFraction(recipe), 1);
		return check(sourceId, "cure-mass-basis", agrees ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,
			agrees ? "The cure mass fraction matches the explicitly selected weighed ingredients."
			: "The cure mass fraction disagrees with its declared ingredient mass basis.");
	}

	public static function copyMixer(raw:Dynamic):PourMixerSpec {
		CastingFeatureCodec.fields(raw, ["reference", "capacityKg", "mixingSeconds", "transferSeconds", "workingTimeSeconds", "workingTimeReference"]);
		var result:PourMixerSpec = {reference: text(raw, "reference"), capacityKg: CastingRecordCodec.number(raw, "capacityKg"),
			mixingSeconds: CastingRecordCodec.number(raw, "mixingSeconds"), transferSeconds: CastingRecordCodec.number(raw, "transferSeconds"),
			workingTimeSeconds: CastingRecordCodec.number(raw, "workingTimeSeconds"), workingTimeReference: text(raw, "workingTimeReference")};
		if (result.capacityKg <= 0 || result.mixingSeconds <= 0 || result.transferSeconds < 0 || result.workingTimeSeconds <= 0)
			throw "Invalid mixer capacity or times";
		return result;
	}
	static function text(raw:Dynamic, field:String):String {
		var value = CastingRecordCodec.string(raw, field);
		if (StringTools.trim(value).length == 0 || materia.assembly.AssemblyCodec.containsNul(value)) throw "Invalid pour specification reference";
		return value;
	}
	static function near(a:Float, b:Float, scale:Float):Bool
		return Math.abs(a - b) <= RELATIVE_TOLERANCE * Math.max(1, scale);
	static function check(id:String, suffix:String, status:ToolingCheckStatus, message:String):ToolingValidation
		return {rule: "process.pour." + suffix, affectedIds: [id], status: status,
			severity: status == ToolingCheckStatus.Pass ? ToolingSeverity.Info : status == ToolingCheckStatus.Pending ? ToolingSeverity.Warning : ToolingSeverity.Error, message: message, evidence: []};
}
