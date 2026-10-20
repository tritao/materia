package materia.casting;

import materia.casting.CastingFeature;
import materia.assembly.AssemblyCodec;

/** Strict source schema; no old box-list encoding is accepted. */
class CastingFeatureCodec {
	public static inline var VERSION = 1;

	public static function encode(features:Array<CastingFeature>):String {
		var copy = snapshot(features);
		return haxe.Json.stringify({version: VERSION, features: copy});
	}

	public static function decode(text:String):Array<CastingFeature> {
		var raw:Dynamic = haxe.Json.parse(text);
		fields(raw, ["version", "features"]);
		if (raw.version != VERSION)
			throw "Unsupported casting feature schema";
		return snapshot(Reflect.field(raw, "features"));
	}

	public static function snapshot(raw:Dynamic):Array<CastingFeature> {
		if (!Std.isOfType(raw, Array))
			throw "Invalid casting feature list";
		var rows:Array<Dynamic> = cast raw;
		if (rows.length > 512)
			throw "Invalid casting feature list";
		var copy:Array<CastingFeature> = [];
		for (row in rows) {
			fields(row, [
				"id",
				"kind",
				"geometry",
				"openings",
				"draft",
				"cornerRadius",
				"owner",
				"adjustable",
				"derivedFrom"
			]);
			var g:Dynamic = Reflect.field(row, "geometry"),
				d:Dynamic = Reflect.field(row, "draft"),
				axis:Dynamic = Reflect.field(d, "axis");
			fields(g, ["kind", "width", "depth", "height", "pose"]);
			fields(d, ["degrees", "axis"]);
			fields(axis, ["x", "y", "z"]);
			var openings:Array<CastingOpening> = [];
			for (o in CastingRecordCodec.array(row, "openings")) {
				fields(o, ["kind", "target"]);
				openings.push({kind: cast CastingRecordCodec.string(o, "kind"), target: CastingRecordCodec.string(o, "target")});
			}
			var adjustable:Array<String> = [];
			for (value in CastingRecordCodec.array(row, "adjustable")) {
				if (!Std.isOfType(value, String))
					throw "Invalid adjustable dimension";
				adjustable.push(cast value);
			}
			copy.push({
				id: CastingRecordCodec.string(row, "id"),
				kind: cast CastingRecordCodec.string(row, "kind"),
				geometry: {
					kind: cast CastingRecordCodec.string(g, "kind"),
					width: CastingRecordCodec.number(g, "width"),
					depth: CastingRecordCodec.number(g, "depth"),
					height: CastingRecordCodec.number(g, "height"),
					pose: CastingRecordCodec.frame(Reflect.field(g, "pose"))
				},
				openings: openings,
				draft: {degrees: CastingRecordCodec.number(d,
					"degrees"), axis: {x: CastingRecordCodec.number(axis,
						"x"), y: CastingRecordCodec.number(axis, "y"), z: CastingRecordCodec.number(axis, "z")}},
				cornerRadius: CastingRecordCodec.number(row, "cornerRadius"),
				owner: CastingRecordCodec.optionalString(row, "owner"),
				adjustable: adjustable,
				derivedFrom: CastingRecordCodec.optionalString(row, "derivedFrom")
			});
		}
		var ids = new Map<String, Bool>();
		for (f in copy) {
			fields(f, [
				"id",
				"kind",
				"geometry",
				"openings",
				"draft",
				"cornerRadius",
				"owner",
				"adjustable",
				"derivedFrom"
			]);
			identifier(f.id);
			if (ids.exists(f.id))
				throw "Duplicate casting feature: " + f.id;
			ids.set(f.id, true);
			if ([Lightening, ServiceOpening, InterfaceRecess, InsertRecess, Clearance].indexOf(f.kind) < 0)
				throw "Unknown casting feature kind: " + f.id;
			var g = f.geometry;
			fields(g, ["kind", "width", "depth", "height", "pose"]);
			if (g.kind != Box)
				throw "Unsupported casting feature geometry: " + f.id;
			for (v in [g.width, g.depth, g.height])
				if (!Math.isFinite(v) || v <= 0)
					throw "Invalid casting feature dimensions: " + f.id;
			AssemblyCodec.validateFrame(g.pose);
			fields(f.draft, ["degrees", "axis"]);
			fields(f.draft.axis, ["x", "y", "z"]);
			var a = f.draft.axis;
			if (!Math.isFinite(f.draft.degrees)
				|| f.draft.degrees < 0
				|| f.draft.degrees >= 45
				|| Math.abs(a.x * a.x + a.y * a.y + a.z * a.z - 1) > 1e-8)
				throw "Invalid casting feature draft: " + f.id;
			if (!Math.isFinite(f.cornerRadius) || f.cornerRadius < 0 || f.cornerRadius > Math.min(g.width, g.depth) / 2)
				throw "Invalid casting feature corner radius: " + f.id;
			if (f.owner != null)
				identifier(f.owner);
			if (f.derivedFrom != null)
				identifier(f.derivedFrom);
			if (f.openings == null || f.adjustable == null)
				throw "Missing casting feature declarations: " + f.id;
			var openings = new Map<String, Bool>();
			for (o in f.openings) {
				fields(o, ["kind", "target"]);
				if (o.kind != BodyFace && o.kind != Feature)
					throw "Unknown casting opening kind: " + f.id;
				identifier(o.target);
				var key = o.kind + ":" + o.target;
				if (openings.exists(key))
					throw "Duplicate casting opening: " + f.id;
				openings.set(key, true);
			}
			var dimensions = new Map<String, Bool>();
			for (d in f.adjustable) {
				if (["width", "depth", "height", "draft", "cornerRadius"].indexOf(d) < 0 || dimensions.exists(d))
					throw "Invalid adjustable dimension: " + f.id;
				dimensions.set(d, true);
			}
		}
		for (f in copy)
			for (o in f.openings)
				if (o.kind == Feature && (!ids.exists(o.target) || o.target == f.id))
					throw "Unknown feature opening: " + f.id;
		copy.sort((a, b) -> Reflect.compare(a.id, b.id));
		return copy;
	}

	public static function fields(raw:Dynamic, names:Array<String>):Void {
		if (raw == null || Std.isOfType(raw, Array) || Std.isOfType(raw, String))
			throw "Invalid casting record";
		var actual = Reflect.fields(raw);
		if (actual.length != names.length)
			throw "Unexpected casting record fields";
		for (name in names)
			if (!Reflect.hasField(raw, name))
				throw "Missing casting field: " + name;
	}

	public static function identifier(id:String):Void {
		if (id == null || !~/^[A-Za-z0-9][A-Za-z0-9_.-]*$/.match(id) || AssemblyCodec.containsNul(id))
			throw "Invalid casting ID";
	}
}
