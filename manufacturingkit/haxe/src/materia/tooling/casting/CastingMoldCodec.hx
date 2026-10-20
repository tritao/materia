package materia.tooling.casting;

import materia.tooling.ToolingCodec;

/** Persist source, process options and resolved plan; load verifies exact regeneration. */
class CastingMoldCodec {
	public static function encode(p:CastingMoldPlan):String {
		CastingMoldPlanner.validate(p);
		if (ToolingCodec.canonical(CastingMoldPlanner.plan(p.document, p.options)) != ToolingCodec.canonical(p))
			throw "Mold differs from source regeneration";
		var s = record(p);
		if (haxe.io.Bytes.ofString(s).length > 4194304)
			throw "Casting mold exceeds size limit";
		return s;
	}

	static function record(p:CastingMoldPlan):String {
		var resolved:Dynamic = {};
		for (key in Reflect.fields(p))
			if (key != "document")
				Reflect.setField(resolved, key, Reflect.field(p, key));
		return ToolingCodec.canonical({
			format: "materia.casting-mold",
			version: 7,
			source: haxe.Json.parse(ToolingCodec.encode(p.document)),
			options: p.options,
			plan: resolved
		}) + "\n";
	}

	public static function decode(s:String):CastingMoldPlan {
		if (s == null || haxe.io.Bytes.ofString(s).length > 4194304)
			throw "Casting mold exceeds size limit";
		var r:Dynamic = haxe.Json.parse(s);
		if (r == null || Reflect.field(r, "format") != "materia.casting-mold" || Reflect.field(r, "version") != 7)
			throw "Unsupported casting mold format/version";
		var d = ToolingCodec.decode(haxe.Json.stringify(Reflect.field(r, "source"))),
			o = CastingMoldOptionsCodec.read(Reflect.field(r, "options")),
			p = CastingMoldPlanner.plan(d, o);
		if (ToolingCodec.canonical(r) != ToolingCodec.canonical(haxe.Json.parse(record(p))))
			throw "Casting mold records differ from regeneration";
		return p;
	}

	public static function fingerprint(p:CastingMoldPlan):String
		return haxe.crypto.Sha256.encode(encode(p));
}
