package materia.tooling.process;

import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingValidation;
import materia.tooling.process.CastingProcessSpec;

typedef CureSectionResult = {
	var id:String;
	var sourceIds:Array<String>;
	var thicknessMm:Float;
	var peakTemperatureC:Float;
	var timeToPeakSeconds:Float;
	var finalTemperaturesC:Array<Float>;
	var finalConversions:Array<Float>;

	/** Estimate only; conduction can preheat unreacted material. Completion is checked separately. */
	var futurePeakEstimateC:Float;

	var steps:Int;
	var remainingHeatC:Float;
	var reactionComplete:Bool;
	var simulatedSeconds:Float;
	var extensionCapReached:Bool;
	var reactionHeatJPerM2:Float;
	var boundaryHeatOutJPerM2:Float;
	var storedHeatJPerM2:Float;
	var energyResidualJPerM2:Float;
}

typedef CureLimitResult = {
	var id:String;
	var maximumTemperatureC:Null<Float>;
	var marginC:Null<Float>;
}

typedef CureReport = {
	var sourceFingerprint:String;
	var resinReference:String;
	var hasEPS:Bool;
	var processFingerprint:String;
	var adiabaticRiseC:Null<Float>;
	var lumpedPeakEstimateC:Null<Float>;
	var sections:Array<CureSectionResult>;
	var limits:Array<CureLimitResult>;
	var validation:Array<ToolingValidation>;
}

/** Cell-centred finite-volume slab, backward Euler conduction and backward Euler
 * declared Arrhenius/autocatalytic conversion, iterated together at every step. SI internally.
 * Only declared planar slabs are represented; joints and steel sinks are not invented.
 */
class CureAnalysis {
	// Universal gas constant, not a recipe-dependent material/process threshold.
	static inline var GAS_CONSTANT = 8.31446261815324;

	static function required(v:Null<Float>):Float {
		if (v == null)
			throw "Missing cure numeric input";
		return v;
	}

	public static function adiabaticRise(c:ResinCureSpec):Null<Float> {
		if (c.resinMassFraction == null || c.heatOfReactionJPerKgResin == null || c.mixSpecificHeatJPerKgK == null)
			return null;
		return c.resinMassFraction * c.heatOfReactionJPerKgResin / c.mixSpecificHeatJPerKgK;
	}

	static function coefficient(b:ThermalBoundary, conductivity:Float, dx:Float):Float {
		if (b.kind == Insulated)
			return 0;
		if (b.kind == FixedTemperature)
			return 2 * conductivity / dx;
		return 1 / (dx / (2 * conductivity) + 1 / required(b.heatTransferWPerM2K));
	}

	static function solve(lower:Array<Float>, diagonal:Array<Float>, upper:Array<Float>, rhs:Array<Float>):Array<Float> {
		var n = diagonal.length, d = diagonal.copy(), r = rhs.copy();
		for (i in 1...n) {
			var f = lower[i] / d[i - 1];
			d[i] -= f * upper[i - 1];
			r[i] -= f * r[i - 1];
		}
		var out = [for (_ in 0...n) 0.0];
		out[n - 1] = r[n - 1] / d[n - 1];
		var i = n - 2;
		while (i >= 0) {
			out[i] = (r[i] - upper[i] * out[i + 1]) / d[i];
			i--;
		}
		return out;
	}

	/** Callers use ProcessSpecCodec first. This entry also validates its direct inputs. */
	public static function section(section:CureSection, spec:CastingProcessSpec, densityKgM3:Float):CureSectionResult {
		spec = ProcessSpecCodec.snapshot(spec);
		var found = spec.sections.filter(s -> s.id == section.id);
		if (found.length != 1 || materia.tooling.ToolingCodec.canonical(found[0]) != materia.tooling.ToolingCodec.canonical(section))
			throw "Cure section does not match process spec";
		if (!Math.isFinite(densityKgM3) || densityKgM3 <= 0)
			throw "Invalid cure mix density";
		var c = spec.cure, rise = adiabaticRise(c), initial = spec.initialTemperatureC;
		if (rise == null || initial == null || c.mixConductivityWPerMK == null || c.mixSpecificHeatJPerKgK == null)
			throw "Missing cure thermal properties";
		if (rise > 0 && c.kinetics == null)
			throw "Missing cure kinetics";
		var settings = spec.solver,
			n = settings.cells,
			dx = section.thicknessMm * 0.001 / n;
		var conductivity:Float = c.mixConductivityWPerMK,
			capacity:Float = c.mixSpecificHeatJPerKgK * densityKgM3;
		var temperature = [for (_ in 0...n) (initial : Float)],
			alpha = [for (_ in 0...n) c.kinetics == null ? 0.0 : c.kinetics.initialConversion];
		var peak:Float = initial, peakTime = 0.0, time = 0.0, steps = 0;
		var reactionHeat = 0.0, boundaryHeat = 0.0;
		function remaining():Float {
			var minimum = alpha[0];
			for (value in alpha)
				minimum = Math.min(minimum, value);
			return rise * (1 - minimum);
		}
		var cap = settings.durationSeconds + settings.maximumExtensionSeconds;
		while (time < cap - 1e-10 && (time < settings.durationSeconds - 1e-10 || remaining() > settings.remainingHeatToleranceC)) {
			var boundaryTime = time < settings.durationSeconds - 1e-10 ? settings.durationSeconds : cap;
			var dt = Math.min(settings.timeStepSeconds, boundaryTime - time),
				guess = temperature.copy();
			var nextAlpha = alpha.copy(),
				accepted:Array<Float> = [],
				converged = false;
			var r = conductivity * dt / (capacity * dx * dx);
			var l = coefficient(section.left, conductivity, dx) * dt / (capacity * dx);
			var q = coefficient(section.right, conductivity, dx) * dt / (capacity * dx);
			for (_ in 0...settings.maximumIterations) {
				var alphaError = 0.0;
				for (i in 0...n) {
					if (c.kinetics != null && rise > 0) {
						var kelvin = guess[i] + 273.15;
						if (kelvin <= 0)
							throw "Invalid cure temperature";
						var k = c.kinetics;
						var rate = k.preExponentialPerSecond;
						if (k.kind == AutocatalyticArrhenius)
							rate = (rate
								+ k.autocatalyticPreExponentialPerSecond * Math.pow(nextAlpha[i],
									k.autocatalyticExponentM)) * Math.pow(Math.max(0, 1 - nextAlpha[i]), k.reactionExponentN - 1);
						rate *= Math.exp(-k.activationEnergyJPerMol / (GAS_CONSTANT * kelvin));
						var updated = (alpha[i] + dt * rate) / (1 + dt * rate);
						alphaError = Math.max(alphaError, Math.abs(updated - nextAlpha[i]) * rise);
						nextAlpha[i] = updated;
					} else
						nextAlpha[i] = alpha[i];
				}
				var lower = [for (_ in 0...n) -r],
					upper = lower.copy(),
					diagonal = [for (_ in 0...n) 1 + 2 * r];
				var rhs = [for (i in 0...n) temperature[i] + rise * (nextAlpha[i] - alpha[i])];
				lower[0] = 0;
				upper[n - 1] = 0;
				diagonal[0] = 1 + r + l;
				diagonal[n - 1] = 1 + r + q;
				if (section.left.kind != Insulated)
					rhs[0] += l * required(section.left.temperatureC);
				if (section.right.kind != Insulated)
					rhs[n - 1] += q * required(section.right.temperatureC);
				accepted = solve(lower, diagonal, upper, rhs);
				var error = 0.0;
				for (i in 0...n) {
					if (!Math.isFinite(accepted[i]))
						throw "Nonfinite cure solution";
					error = Math.max(error, Math.abs(accepted[i] - guess[i]));
				}
				guess = accepted;
				if (Math.max(error, alphaError) <= settings.convergenceToleranceC) {
					converged = true;
					break;
				}
			}
			if (!converged)
				throw "Cure solver did not converge; refine timestep";
			for (i in 0...n)
				reactionHeat += capacity * dx * rise * (nextAlpha[i] - alpha[i]);
			for (side in [0, 1]) {
				var boundary = side == 0 ? section.left : section.right;
				if (boundary.kind != Insulated)
					boundaryHeat += coefficient(boundary, conductivity, dx) * (accepted[side == 0 ? 0 : n - 1] - required(boundary.temperatureC)) * dt;
			}
			time += dt;
			steps++;
			temperature = accepted;
			alpha = nextAlpha.copy();
			for (t in temperature)
				if (t > peak) {
					peak = t;
					peakTime = time;
				}
		}
		var hottest = temperature[0], leastConverted = alpha[0];
		for (i in 0...n) {
			hottest = Math.max(hottest, temperature[i]);
			leastConverted = Math.min(leastConverted, alpha[i]);
		}
		for (b in [section.left, section.right])
			if (b.kind != Insulated)
				hottest = Math.max(hottest, required(b.temperatureC));
		var storedHeat = 0.0;
		for (value in temperature)
			storedHeat += capacity * dx * (value - initial);
		var remainingHeat = rise * (1 - leastConverted);
		return {
			id: section.id,
			sourceIds: section.sourceIds.copy(),
			thicknessMm: section.thicknessMm,
			peakTemperatureC: peak,
			timeToPeakSeconds: peakTime,
			finalTemperaturesC: temperature,
			finalConversions: alpha,
			futurePeakEstimateC: Math.max(peak, hottest + rise * (1 - leastConverted)),
			steps: steps,
			remainingHeatC: remainingHeat,
			reactionComplete: remainingHeat <= settings.remainingHeatToleranceC,
			simulatedSeconds: time,
			extensionCapReached: remainingHeat > settings.remainingHeatToleranceC && time >= cap - 1e-8,
			reactionHeatJPerM2: reactionHeat,
			boundaryHeatOutJPerM2: boundaryHeat,
			storedHeatJPerM2: storedHeat,
			energyResidualJPerM2: storedHeat - reactionHeat + boundaryHeat
		};
	}

	public static function analyze(sourceId:String, sourceFingerprint:String, processFingerprint:String, raw:CastingProcessSpec, densityKgM3:Float,
			epsLimitC:Null<Float>, hasEPS:Bool):CureReport {
		materia.tooling.ToolingIds.validate(sourceId);
		if (!Math.isFinite(densityKgM3)
			|| densityKgM3 <= 0
			|| epsLimitC != null
			&& (!Math.isFinite(epsLimitC) || epsLimitC <= -273.15))
			throw "Invalid cure analysis inputs";
		var spec = ProcessSpecCodec.snapshot(raw),
			rise = adiabaticRise(spec.cure),
			peak:Null<Float> = null;
		var checks:Array<ToolingValidation> = [],
			results:Array<CureSectionResult> = [];
		if (rise != null && spec.initialTemperatureC != null && spec.ambientTemperatureC != null) {
			var remaining = spec.cure.kinetics == null ? 1.0 : 1 - spec.cure.kinetics.initialConversion;
			var environment = Math.max(spec.initialTemperatureC, spec.ambientTemperatureC);
			for (section in spec.sections)
				for (boundary in [section.left, section.right])
					if (boundary.temperatureC != null)
						environment = Math.max(environment, boundary.temperatureC);
			peak = environment + rise * remaining;
		}
		var limits:Array<CureLimitResult> = [
			{id: sourceId, maximumTemperatureC: spec.cure.resinMaximumTemperatureC, marginC: null},
			{id: "resin-gel", maximumTemperatureC: spec.cure.gelMaximumTemperatureC, marginC: null}
		];
		if (hasEPS)
			limits.push({id: "eps-core-stock", maximumTemperatureC: epsLimitC, marginC: null});
		var boundSafe = peak != null && spec.temperatureMarginC != null && spec.temperatureMarginC > 0;
		for (limit in limits) {
			if (limit.maximumTemperatureC == null || peak == null || spec.temperatureMarginC == null)
				boundSafe = false;
			else {
				limit.marginC = limit.maximumTemperatureC - peak;
				if (limit.marginC <= spec.temperatureMarginC)
					boundSafe = false;
			}
		}
		checks.push(FillAnalysis.check("process.cure.lumped-estimate", [sourceId], boundSafe ? ToolingCheckStatus.Pass : ToolingCheckStatus.Pending,
			boundSafe ? "Lumped adiabatic estimate is below declared limits with the required margin. This is not a spatial upper bound or whole-casting qualification." : "Lumped estimate lacks inputs, limits or required margin, or exceeds a screening limit. Spatial preheating is not bounded by this estimate.",
			[processFingerprint]));
		checks.push(FillAnalysis.check("process.cure.material-evidence", [sourceId],
			spec.cure.evidence.length > 0 ? ToolingCheckStatus.Pass : ToolingCheckStatus.Pending,
			"Mix heat capacity, conductivity, resin fraction, reaction heat and kinetic model need supplier/trial provenance; references do not qualify physical calibration.",
			spec.cure.evidence));
		{
			if (spec.sections.length == 0)
				checks.push(FillAnalysis.check("process.cure.section-coverage", [sourceId], ToolingCheckStatus.Pending,
					"Opposing-face slab sections and junction coverage are not qualified. No whole-casting thermal pass from a nominal section.", []));
			for (s in spec.sections) {
				var rule = "process.cure.section." + s.id;
				if (peak == null || rise == null || spec.cure.mixConductivityWPerMK == null || rise > 0 && spec.cure.kinetics == null) {
					checks.push(FillAnalysis.check(rule, s.sourceIds, ToolingCheckStatus.Pending,
						"Missing thermal properties, initial temperature or cure kinetics.", []));
					continue;
				}
				var result:CureSectionResult;
				try
					result = section(s, spec, densityKgM3)
				catch (error:Dynamic) {
					var message = Std.string(error);
					if (message.indexOf("Cure solver did not converge") < 0 && message.indexOf("Nonfinite cure solution") < 0)
						throw error;
					checks.push(FillAnalysis.check(rule, s.sourceIds, ToolingCheckStatus.Unsupported, message, [processFingerprint]));
					continue;
				}
				results.push(result);
				checks.push(FillAnalysis.check("process.cure.convergence." + s.id, s.sourceIds, ToolingCheckStatus.Pending,
					"Analytic solver fixtures pass; the actual section still requires time/space convergence with its declared material and boundary data.",
					[processFingerprint]));
				var exceeded = false,
					safe = spec.temperatureMarginC != null && spec.temperatureMarginC > 0 && result.reactionComplete;
				for (limit in limits) {
					if (limit.maximumTemperatureC == null)
						safe = false;
					else {
						if (result.peakTemperatureC >= limit.maximumTemperatureC)
							exceeded = true;
						if (spec.temperatureMarginC == null
							|| result.peakTemperatureC + result.remainingHeatC >= limit.maximumTemperatureC - spec.temperatureMarginC)
							safe = false;
					}
				}
				if (s.evidence.length == 0 || s.left.evidence.length == 0 || s.right.evidence.length == 0 || spec.cure.evidence.length == 0)
					safe = false;
				checks.push(FillAnalysis.check(rule, s.sourceIds,
					exceeded ? ToolingCheckStatus.Fail : safe ? ToolingCheckStatus.Pass : ToolingCheckStatus.Pending,
					"Declared 1D slab peak="
					+ result.peakTemperatureC
					+ " C; residual-heat estimate="
					+ result.futurePeakEstimateC
					+ " C; remaining reaction heat="
					+ result.remainingHeatC
					+ " C; completion="
					+ result.reactionComplete
					+ ". This is not an upper bound and does not establish junction/whole-casting coverage.",
					s.evidence.concat(spec.cure.evidence)));
			}
			// Declared examples do not prove automatic opposing-face/junction coverage.
			if (spec.sections.length > 0)
				checks.push(FillAnalysis.check("process.cure.section-coverage", [sourceId], ToolingCheckStatus.Pending,
					"Declared slab results require geometric section/junction coverage before whole-casting release.", []));
		}
		checks.push(FillAnalysis.check("process.cure.kinetics-model",
			[sourceId], rise == 0 || spec.cure.kinetics != null && spec.cure.kinetics.kind == AutocatalyticArrhenius ? ToolingCheckStatus.Pass : ToolingCheckStatus.Pending,
			"Reactive fabrication requires declared autocatalytic kinetics; first-order kinetics are retained for analytic fixtures only.",
			spec.cure.evidence));
		checks.push(FillAnalysis.check("process.cure.physical-calibration", [sourceId], ToolingCheckStatus.Pending,
			"Thermocouple traces for the actual mix, core surface and thickest section are required for qualification.", []));
		return {
			sourceFingerprint: sourceFingerprint,
			resinReference: spec.cure.reference,
			hasEPS: hasEPS,
			processFingerprint: processFingerprint,
			adiabaticRiseC: rise,
			lumpedPeakEstimateC: peak,
			sections: results,
			limits: limits,
			validation: checks
		};
	}
}
