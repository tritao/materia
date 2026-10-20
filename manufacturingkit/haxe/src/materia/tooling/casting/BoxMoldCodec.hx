package materia.tooling.casting;

import haxe.Json;
import haxe.crypto.Sha256;
import materia.tooling.ToolingCodec;

/** Version 7 persists this explicit template and its resolved records. On load all records
 * must equal regeneration from the strict source/setup/recipe document. No native resources.
 * Hand-edited generated plans are deliberately rejected; edit the source document instead.
 */
class BoxMoldCodec {
	public static function encode(p:MoldPlan):String {
		BoxMoldPlanner.validate(p);
		var regenerated = BoxMoldPlanner.plan(p.document, p.supportOptions);
		if (ToolingCodec.canonical(p) != ToolingCodec.canonical(regenerated))
			throw "Mold records differ from template regeneration";
		var text = record(p);
		if (haxe.io.Bytes.ofString(text).length > 2097152)
			throw "Mold record exceeds size limit";
		return text;
	}

	static function record(p:MoldPlan):String {
		var resolved:Dynamic = {};
		for (key in Reflect.fields(p))
			if (key != "document")
				Reflect.setField(resolved, key, Reflect.field(p, key));
		return ToolingCodec.canonical({
			format: "materia.box-mold",
			version: 8,
			supportOptions: p.supportOptions,
			source: Json.parse(ToolingCodec.encode(p.document)),
			resolved: resolved
		}) + "\n";
	}

	public static function decode(text:String):MoldPlan {
		if (text == null || haxe.io.Bytes.ofString(text).length > 2097152)
			throw "Mold record exceeds size limit";
		var raw:Dynamic = Json.parse(text);
		if (raw == null || Reflect.field(raw, "format") != "materia.box-mold" || Reflect.field(raw, "version") != 8)
			throw "Unsupported mold format/version";
		var document = ToolingCodec.decode(Json.stringify(Reflect.field(raw, "source"))),
			p = BoxMoldPlanner.plan(document, CoreSupports.options(Reflect.field(raw, "supportOptions")));
		if (ToolingCodec.canonical(raw) != ToolingCodec.canonical(Json.parse(record(p))))
			throw "Mold records differ from template regeneration";
		return p;
	}

	public static function fingerprint(p:MoldPlan):String
		return Sha256.encode(encode(p));
}
