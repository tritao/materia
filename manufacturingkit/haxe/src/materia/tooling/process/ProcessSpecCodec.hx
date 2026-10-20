package materia.tooling.process;

import materia.tooling.ToolingCodec;
import materia.tooling.ToolingIds;
import materia.tooling.process.CastingProcessSpec;

/** Strict independent v2 format. Reconstructs typed records after validation. */
class ProcessSpecCodec {
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
			"castability",
			"cure",
			"sections",
			"solver"
		]);
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
				"initialConversion"
			]);
			if (k.kind != "first-order-arrhenius")
				throw "Unsupported cure kinetics";
			kinetics = {
				kind: FirstOrderArrhenius,
				preExponentialPerSecond: number(k.preExponentialPerSecond, 1e-30, 1e30),
				activationEnergyJPerMol: number(k.activationEnergyJPerMol, 0, 1e7),
				initialConversion: number(k.initialConversion, 0, 1)
			};
		}
		var sections:Array<CureSection> = [], ids:Array<String> = [];
		for (s in list(v.sections, 128)) {
			object(s, ["id", "sourceIds", "thicknessMm", "left", "right", "evidence"]);
			var sid = text(s.id);
			ToolingIds.validate(sid);
			if (ids.indexOf(sid) >= 0)
				throw "Duplicate cure section";
			ids.push(sid);
			var sources = texts(s.sourceIds, true);
			if (sources.length == 0)
				throw "Cure section needs source IDs";
			sections.push({
				id: sid,
				sourceIds: sources,
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
			"convergenceToleranceC",
			"maximumIterations"
		]);
		var solver:CureSolverSettings = {
			cells: integer(s.cells, 3, 201),
			timeStepSeconds: number(s.timeStepSeconds, 1e-6, 1e7),
			durationSeconds: number(s.durationSeconds, 1e-6, 1e9),
			convergenceToleranceC: number(s.convergenceToleranceC, 1e-10, 0.1),
			maximumIterations: integer(s.maximumIterations, 2, 1000)
		};
		if (solver.durationSeconds / solver.timeStepSeconds > 2000000.0 / solver.cells)
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
			castability: castability,
			cure: {
				reference: text(c.reference),
				resinMassFraction: optional(c.resinMassFraction, 0, 1),
				heatOfReactionJPerKgResin: optional(c.heatOfReactionJPerKgResin, 0, 1e9),
				mixSpecificHeatJPerKgK: optional(c.mixSpecificHeatJPerKgK, 1e-9, 1e9),
				mixConductivityWPerMK: optional(c.mixConductivityWPerMK, 1e-12, 1e6),
				resinMaximumTemperatureC: optional(c.resinMaximumTemperatureC, -273.14, 2000),
				kinetics: kinetics,
				evidence: texts(c.evidence)
			},
			sections: sections,
			solver: solver
		};
	}

	public static function encode(v:CastingProcessSpec):String {
		var text = ToolingCodec.canonical({format: "materia.casting-process", version: 2, spec: snapshot(v)}) + "\n";
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
		if (v.format != "materia.casting-process" || v.version != 2)
			throw "Unsupported casting process format/version";
		return snapshot(v.spec);
	}
}
