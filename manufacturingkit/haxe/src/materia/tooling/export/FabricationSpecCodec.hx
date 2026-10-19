package materia.tooling.export;

import materia.tooling.ToolingCodec;
import materia.tooling.ToolingIds;
import materia.tooling.export.FabricationSpec;

/** A new format, independent of the casting source contract. Unknown fields fail. */
class FabricationSpecCodec {
	static function hasNul(s:String):Bool {
		for (i in 0...s.length)
			if (s.charCodeAt(i) == 0)
				return true;
		return false;
	}

	static function object(v:Dynamic, names:Array<String>):Void {
		if (v == null || !Reflect.isObject(v) || Std.isOfType(v, Array) || Std.isOfType(v, String))
			throw "Invalid fabrication object";
		for (k in Reflect.fields(v))
			if (names.indexOf(k) < 0)
				throw "Unknown fabrication field: " + k;
		for (k in names)
			if (!Reflect.hasField(v, k))
				throw "Missing fabrication field: " + k;
	}

	static function number(v:Dynamic):Void {
		if ((!Std.isOfType(v, Int) && !Std.isOfType(v, Float)) || !Math.isFinite(v) || Math.abs(v) > 1e12)
			throw "Invalid fabrication number";
	}

	static function list(v:Dynamic):Array<Dynamic> {
		if (!Std.isOfType(v, Array))
			throw "Invalid fabrication array";
		var a:Array<Dynamic> = cast v;
		if (a.length > 1000)
			throw "Fabrication array exceeds limit";
		return a;
	}

	static function strings(v:Dynamic, ids:Bool = false):Void {
		var seen = new Map<String, Bool>();
		for (x in list(v)) {
			if (!Std.isOfType(x, String))
				throw "Invalid fabrication text";
			var s:String = cast x;
			if (StringTools.trim(s).length == 0 || s.length > 1000 || hasNul(s) || seen.exists(s))
				throw "Invalid/duplicate fabrication text";
			seen.set(s, true);
			if (ids)
				ToolingIds.validate(s);
		}
	}

	static function rows(v:Dynamic, ids:Array<String>, nums:Array<String>, references:Array<String>):Void {
		for (row in list(v)) {
			object(row, ids.concat(nums).concat(references).concat(["evidence"]));
			for (k in ids) {
				var x:Dynamic = Reflect.field(row, k);
				if (!Std.isOfType(x, String))
					throw "Invalid fabrication identifier";
				ToolingIds.validate(cast x);
			}
			for (k in nums)
				number(Reflect.field(row, k));
			for (k in references)
				strings(Reflect.field(row, k), true);
			strings(Reflect.field(row, "evidence"));
		}
	}

	static function texts(v:Dynamic):Array<String> {
		var result:Array<String> = [for (x in list(v)) cast(x, String)];
		result.sort(Reflect.compare);
		return result;
	}

	public static function snapshot(input:Dynamic):FabricationSpec {
		var v:Dynamic = input;
		object(v, [
			"revision",
			"sheetMaterialId",
			"solvent",
			"adhesiveContainsEPS",
			"curePeakC",
			"cureEvidence",
			"compatibilityEvidence",
			"gravityMps2",
			"loadFactor",
			"compactionPressurePa",
			"compactionEvidence",
			"plates",
			"braces",
			"fasteners",
			"coreRestraints",
			"blankMarginMm",
			"blankKerfMm",
			"couponClearancesMm"
		]);
		if (!Std.isOfType(v.revision, Int) || v.revision < 1)
			throw "Invalid fabrication revision";
		if (v.solvent != "unknown" && v.solvent != "styrene-bearing" && v.solvent != "styrene-free")
			throw "Invalid resin solvent family";
		for (k in ["gravityMps2", "loadFactor", "blankMarginMm", "blankKerfMm"])
			number(Reflect.field(v, k));
		for (k in ["curePeakC", "compactionPressurePa"])
			if (Reflect.field(v, k) != null)
				number(Reflect.field(v, k));
		for (k in ["cureEvidence", "compatibilityEvidence", "compactionEvidence"])
			strings(Reflect.field(v, k));
		for (x in list(v.couponClearancesMm))
			number(x);
		if (v.blankMarginMm < 0
			|| v.blankKerfMm < 0
			|| list(v.couponClearancesMm).length == 0
			|| list(v.couponClearancesMm).length > 8)
			throw "Invalid fabrication blank/coupon settings";
		rows(v.plates, ["panelId"], [
			"spanXmm",
			"spanYmm",
			"youngPa",
			"poisson",
			"allowableStressPa",
			"maximumDeflectionMm"
		], []);
		rows(v.braces, ["id"], [
			"spanMm",
			"tributaryWidthMm",
			"widthMm",
			"depthMm",
			"youngPa",
			"allowableStressPa",
			"maximumDeflectionMm"
		], ["panelIds"]);
		rows(v.fasteners, ["id"], ["allowableN"], ["loadIds", "hardwareIds"]);
		rows(v.coreRestraints, ["coreId"], ["allowableN", "bearingAreaMm2", "allowableBearingPa"], []);
		if (v.adhesiveContainsEPS != null && !Std.isOfType(v.adhesiveContainsEPS, Bool))
			throw "Invalid adhesive EPS declaration";
		if (!Std.isOfType(v.sheetMaterialId, String))
			throw "Invalid sheet material reference";
		materia.tooling.ToolingIds.validate(cast v.sheetMaterialId);
		if (materia.project.MaterialLibrary.get(cast v.sheetMaterialId) == null)
			throw "Unknown sheet material reference";
		var result:FabricationSpec = {
			revision: v.revision,
			sheetMaterialId: v.sheetMaterialId,
			solvent: cast v.solvent,
			adhesiveContainsEPS: v.adhesiveContainsEPS,
			curePeakC: v.curePeakC,
			cureEvidence: texts(v.cureEvidence),
			compatibilityEvidence: texts(v.compatibilityEvidence),
			gravityMps2: v.gravityMps2,
			loadFactor: v.loadFactor,
			compactionPressurePa: v.compactionPressurePa,
			compactionEvidence: texts(v.compactionEvidence),
			blankMarginMm: v.blankMarginMm,
			blankKerfMm: v.blankKerfMm,
			couponClearancesMm: [for (x in list(v.couponClearancesMm)) cast(x, Float)],
			plates: [
				for (row in list(v.plates))
					{
						panelId: cast(row.panelId, String),
						spanXmm: cast(row.spanXmm, Float),
						spanYmm: cast(row.spanYmm, Float),
						youngPa: cast(row.youngPa, Float),
						poisson: cast(row.poisson, Float),
						allowableStressPa: cast(row.allowableStressPa, Float),
						maximumDeflectionMm: cast(row.maximumDeflectionMm, Float),
						evidence: texts(row.evidence)
					}
			],
			braces: [
				for (row in list(v.braces))
					{
						id: cast(row.id, String),
						panelIds: texts(row.panelIds),
						spanMm: cast(row.spanMm, Float),
						tributaryWidthMm: cast(row.tributaryWidthMm, Float),
						widthMm: cast(row.widthMm, Float),
						depthMm: cast(row.depthMm, Float),
						youngPa: cast(row.youngPa, Float),
						allowableStressPa: cast(row.allowableStressPa, Float),
						maximumDeflectionMm: cast(row.maximumDeflectionMm, Float),
						evidence: texts(row.evidence)
					}
			],
			fasteners: [
				for (row in list(v.fasteners))
					{
						id: cast(row.id, String),
						loadIds: texts(row.loadIds),
						hardwareIds: texts(row.hardwareIds),
						allowableN: cast(row.allowableN, Float),
						evidence: texts(row.evidence)
					}
			],
			coreRestraints: [
				for (row in list(v.coreRestraints))
					{
						coreId: cast(row.coreId, String),
						allowableN: cast(row.allowableN, Float),
						bearingAreaMm2: cast(row.bearingAreaMm2, Float),
						allowableBearingPa: cast(row.allowableBearingPa, Float),
						evidence: texts(row.evidence)
					}
			]
		};
		result.plates.sort((a, b) -> Reflect.compare(a.panelId, b.panelId));
		result.braces.sort((a, b) -> Reflect.compare(a.id, b.id));
		result.fasteners.sort((a, b) -> Reflect.compare(a.id, b.id));
		result.coreRestraints.sort((a, b) -> Reflect.compare(a.coreId, b.coreId));
		result.couponClearancesMm.sort((a, b) -> Reflect.compare(a, b));
		return result;
	}

	public static function encode(v:FabricationSpec):String
		return ToolingCodec.canonical({format: "materia.fabrication-spec", version: 1, spec: snapshot(v)});

	public static function decode(s:String):FabricationSpec {
		if (s == null || haxe.io.Bytes.ofString(s).length > 1048576)
			throw "Fabrication spec exceeds limit";
		var v:Dynamic = haxe.Json.parse(s);
		object(v, ["format", "version", "spec"]);
		if (v.format != "materia.fabrication-spec" || v.version != 1)
			throw "Unsupported fabrication spec version";
		return snapshot(v.spec);
	}
}
