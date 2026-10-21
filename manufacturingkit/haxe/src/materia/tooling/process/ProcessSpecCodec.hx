package materia.tooling.process;

import materia.tooling.ToolingCodec;
import materia.tooling.ToolingIds;
import materia.tooling.process.CastingProcessSpec;

/** Strict process format; old encodings are rejected, never filled with new defaults. */
class ProcessSpecCodec {
	public static inline final VERSION = 6;
	static function object(v:Dynamic, keys:Array<String>):Void {
		if (v == null || !Reflect.isObject(v) || Std.isOfType(v, Array) || Std.isOfType(v, String))
			throw "Invalid casting process object";
		for (key in Reflect.fields(v))
			if (keys.indexOf(key) < 0)
				throw "Unknown casting process field: " + key;
		for (key in keys)
			if (!Reflect.hasField(v, key))
				throw "Missing casting process field: " + key;
	}

	static function number(v:Dynamic, minimum:Float, maximum:Float):Float {
		if ((!Std.isOfType(v, Int) && !Std.isOfType(v, Float)) || !Math.isFinite(v) || v < minimum || v > maximum)
			throw "Invalid casting process number";
		return v;
	}

	static function optional(v:Dynamic, minimum:Float, maximum:Float):Null<Float>
		return v == null ? null : number(v, minimum, maximum);

	static function integer(v:Dynamic, minimum:Int, maximum:Int):Int {
		if (!Std.isOfType(v, Int))
			throw "Invalid casting process integer";
		number(v, minimum, maximum);
		return v;
	}

	static function text(v:Dynamic):String {
		if (!Std.isOfType(v, String))
			throw "Invalid casting process text";
		var s:String = v;
		if (StringTools.trim(s).length == 0 || s.length > 1000)
			throw "Invalid casting process text";
		for (i in 0...s.length)
			if (s.charCodeAt(i) == 0)
				throw "Invalid casting process text";
		return s;
	}

	static function list(v:Dynamic, max:Int = 512):Array<Dynamic> {
		if (!Std.isOfType(v, Array))
			throw "Invalid casting process array";
		var a:Array<Dynamic> = cast v;
		if (a.length > max)
			throw "Casting process array exceeds limit";
		return a;
	}

	static function texts(v:Dynamic, ids:Bool = false):Array<String> {
		var out:Array<String> = [];
		for (s in list(v)) {
			var value = text(s);
			if (ids)
				ToolingIds.validate(value);
			if (out.indexOf(value) >= 0)
				throw "Duplicate casting process text";
			out.push(value);
		}
		out.sort(Reflect.compare);
		return out;
	}

	static function boundary(v:Dynamic):ThermalBoundary {
		object(v, ["kind", "temperatureC", "heatTransferWPerM2K", "evidence"]);
		if (v.kind != "insulated" && v.kind != "convection" && v.kind != "fixed-temperature")
			throw "Invalid thermal boundary kind";
		var t = optional(v.temperatureC, -273.14, 2000),
			h = optional(v.heatTransferWPerM2K, 0, 1e8);
		if (v.kind == "insulated" && (t != null || h != null) || v.kind == "fixed-temperature" && (t == null || h != null) || v.kind == "convection"
			&& (t == null || h == null || h <= 0))
			throw "Inconsistent thermal boundary";
		return {
			kind: cast v.kind,
			temperatureC: t,
			heatTransferWPerM2K: h,
			evidence: texts(v.evidence)
		};
	}

	public static function snapshot(v:Dynamic):CastingProcessSpec {
		object(v, [
			"id",
			"revision",
			"recipeFingerprint",
			"initialTemperatureC",
			"ambientTemperatureC",
			"temperatureMarginC",
			"pourRateMm3PerSecond",
			"maximumAirEscapeWidthMm",
			"mixComposition",
			"mixer",
			"castability",
			"cure",
			"sections",
			"solver",
			"faceContacts",
			"openFaceContact"
		]);
		var conditions:Array<{faceId:String, kind:ThermalBoundaryKind, evidence:Array<String>}> = [],
			faceIds:Array<String> = [];
		for (condition in list(v.faceContacts, 4096)) {
			object(condition, ["faceId", "kind", "evidence"]);
			if (["insulated", "convection", "fixed-temperature"].indexOf(condition.kind) < 0)
				throw "Invalid thermal contact kind";
			var faceId = text(condition.faceId);
			ToolingIds.validate(faceId);
			if (faceIds.indexOf(faceId) >= 0)
				throw "Duplicate thermal face condition";
			faceIds.push(faceId);
			conditions.push({faceId: faceId, kind: cast condition.kind, evidence: texts(condition.evidence)});
		}
		conditions.sort((a, b) -> Reflect.compare(a.faceId, b.faceId));
		var open:Null<{kind:ThermalBoundaryKind, evidence:Array<String>}> = null;
		if (v.openFaceContact != null) {
			object(v.openFaceContact, ["kind", "evidence"]);
			if (["insulated", "convection", "fixed-temperature"].indexOf(v.openFaceContact.kind) < 0)
				throw "Invalid thermal contact kind";
			open = {kind: cast v.openFaceContact.kind, evidence: texts(v.openFaceContact.evidence)};
		}
		var id = text(v.id);
		ToolingIds.validate(id);
		var fingerprint = text(v.recipeFingerprint);
		if (!~/^[a-f0-9]{64}$/.match(fingerprint))
			throw "Invalid process recipe fingerprint";
		var limits = v.castability;
		object(limits, [
			"maximumAggregateSizeMm",
			"minimumPassageAggregateRatio",
			"maximumSectionThicknessMm",
			"evidence"
		]);
		var castability:CastabilityLimits = {
			maximumAggregateSizeMm: optional(limits.maximumAggregateSizeMm, 1e-9, 1e6),
			minimumPassageAggregateRatio: optional(limits.minimumPassageAggregateRatio, 1, 1e6),
			maximumSectionThicknessMm: optional(limits.maximumSectionThicknessMm, 1e-9, 1e6),
			evidence: texts(limits.evidence)
		};
		var c = v.cure;
		object(c, [
			"reference",
			"resinMassFraction",
			"heatOfReactionJPerKgResin",
			"mixSpecificHeatJPerKgK",
			"mixConductivityWPerMK",
			"resinMaximumTemperatureC",
			"gelMaximumTemperatureC",
			"kinetics",
			"evidence"
		]);
		var kinetics:Null<CureKinetics> = null;
		if (c.kinetics != null) {
			var k = c.kinetics;
			object(k, [
				"kind",
				"preExponentialPerSecond",
				"activationEnergyJPerMol",
				"initialConversion",
				"autocatalyticPreExponentialPerSecond",
				"autocatalyticExponentM",
				"reactionExponentN"
			]);
			if (k.kind != "first-order-arrhenius" && k.kind != "autocatalytic-arrhenius")
				throw "Unsupported cure kinetics";
			kinetics = {
				kind: cast k.kind,
				preExponentialPerSecond: number(k.preExponentialPerSecond, 1e-30, 1e30),
				activationEnergyJPerMol: number(k.activationEnergyJPerMol, 0, 1e7),
				initialConversion: number(k.initialConversion, 0, 1),
				autocatalyticPreExponentialPerSecond: number(k.autocatalyticPreExponentialPerSecond, 0, 1e30),
				autocatalyticExponentM: number(k.autocatalyticExponentM, 0.01, 10),
				reactionExponentN: number(k.reactionExponentN, 1, 10)
			};
		}
		if (kinetics != null
			&& (kinetics.kind == AutocatalyticArrhenius
				&& kinetics.autocatalyticPreExponentialPerSecond <= 0
				|| kinetics.kind == FirstOrderArrhenius
				&& (kinetics.autocatalyticPreExponentialPerSecond != 0 || kinetics.reactionExponentN != 1)))
			throw "Inconsistent cure kinetics parameters";
		var sections:Array<CureSection> = [], ids:Array<String> = [];
		for (s in list(v.sections, 128)) {
			object(s, [
				"id",
				"sourceIds",
				"spanIds",
				"geometryFingerprint",
				"thicknessMm",
				"left",
				"right",
				"evidence"
			]);
			var sid = text(s.id);
			ToolingIds.validate(sid);
			if (ids.indexOf(sid) >= 0)
				throw "Duplicate cure section";
			ids.push(sid);
			var sources = texts(s.sourceIds, true);
			if (sources.length == 0)
				throw "Cure section needs source IDs";
			var sectionFingerprint:Null<String> = s.geometryFingerprint == null ? null : text(s.geometryFingerprint);
			if (sectionFingerprint != null && !~/^[a-f0-9]{64}$/.match(sectionFingerprint))
				throw "Invalid cure section geometry fingerprint";
			var spanIds = texts(s.spanIds, true);
			if (spanIds.length > 0 && sectionFingerprint == null)
				throw "Bound cure section requires geometry fingerprint";
			sections.push({
				id: sid,
				sourceIds: sources,
				spanIds: spanIds,
				geometryFingerprint: sectionFingerprint,
				thicknessMm: number(s.thicknessMm, 1e-6, 1e6),
				left: boundary(s.left),
				right: boundary(s.right),
				evidence: texts(s.evidence)
			});
		}
		sections.sort((a, b) -> Reflect.compare(a.id, b.id));
		var s = v.solver;
		object(s, [
			"cells",
			"timeStepSeconds",
			"durationSeconds",
			"maximumExtensionSeconds",
			"convergenceToleranceC",
			"maximumIterations",
			"remainingHeatToleranceC"
		]);
		var solver:CureSolverSettings = {
			cells: integer(s.cells, 3, 201),
			timeStepSeconds: number(s.timeStepSeconds, 1e-6, 1e7),
			durationSeconds: number(s.durationSeconds, 1e-6, 1e9),
			maximumExtensionSeconds: number(s.maximumExtensionSeconds, 0, 1e9),
			convergenceToleranceC: number(s.convergenceToleranceC, 1e-10, 0.1),
			maximumIterations: integer(s.maximumIterations, 2, 1000),
			remainingHeatToleranceC: number(s.remainingHeatToleranceC, 1e-9, 1)
		};
		if ((solver.durationSeconds + solver.maximumExtensionSeconds) / solver.timeStepSeconds > 2000000.0 / solver.cells)
			throw "Cure solver work limit exceeded";
		return {
			id: id,
			revision: integer(v.revision, 1, 100000000),
			recipeFingerprint: fingerprint,
			initialTemperatureC: optional(v.initialTemperatureC, -273.14, 2000),
			ambientTemperatureC: optional(v.ambientTemperatureC, -273.14, 2000),
			temperatureMarginC: optional(v.temperatureMarginC, 0, 2000),
			pourRateMm3PerSecond: optional(v.pourRateMm3PerSecond, 1e-12, 1e15),
			maximumAirEscapeWidthMm: optional(v.maximumAirEscapeWidthMm, 1e-9, 1e6),
			mixComposition: v.mixComposition == null ? null : PourPlanning.copyComposition(v.mixComposition),
			mixer: v.mixer == null ? null : PourPlanning.copyMixer(v.mixer),
			castability: castability,
			cure: {
				reference: text(c.reference),
				resinMassFraction: optional(c.resinMassFraction, 0, 1),
				heatOfReactionJPerKgResin: optional(c.heatOfReactionJPerKgResin, 0, 1e9),
				mixSpecificHeatJPerKgK: optional(c.mixSpecificHeatJPerKgK, 1e-9, 1e9),
				mixConductivityWPerMK: optional(c.mixConductivityWPerMK, 1e-12, 1e6),
				resinMaximumTemperatureC: optional(c.resinMaximumTemperatureC, -273.14, 2000),
				gelMaximumTemperatureC: optional(c.gelMaximumTemperatureC, -273.14, 2000),
				kinetics: kinetics,
				evidence: texts(c.evidence)
			},
			sections: sections,
			solver: solver,
			faceContacts: conditions,
			openFaceContact: open
		};
	}

	public static function encode(v:CastingProcessSpec):String {
		var text = ToolingCodec.canonical({format: "materia.casting-process", version: VERSION, spec: snapshot(v)}) + "\n";
		if (haxe.io.Bytes.ofString(text).length > 2097152)
			throw "Casting process size limit";
		return text;
	}

	public static function decode(s:String):CastingProcessSpec {
		if (s == null || haxe.io.Bytes.ofString(s).length > 2097152)
			throw "Casting process size limit";
		var v:Dynamic;
		try
			v = haxe.Json.parse(s)
		catch (error:Dynamic) {
			if (Std.string(error).indexOf("NUL") >= 0)
				throw "Invalid casting process text";
			throw error;
		}
		object(v, ["format", "version", "spec"]);
		if (v.format != "materia.casting-process" || v.version != VERSION)
			throw "Unsupported casting process format/version";
		return snapshot(v.spec);
	}
}
