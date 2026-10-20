package materia.casting;

import materia.casting.CastingFeature;
import materia.assembly.AssemblyCodec;

/** Strict source schema; no old box-list encoding is accepted. */
class CastingFeatureCodec {
	public static inline var VERSION = 3;

	public static function encode(features:Array<CastingFeature>):String {
		var records:Array<Dynamic> = [];
		for (feature in snapshot(features)) {
			var record:Dynamic = {};
			for (key in Reflect.fields(feature))
				Reflect.setField(record, key, Reflect.field(feature, key));
			Reflect.setField(record, "geometry", {solid: CastingFeature.CastingGeometrySolids.record(feature.geometry.solid), pose: feature.geometry.pose});
			records.push(record);
		}
		return haxe.Json.stringify({version: VERSION, features: records});
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
			fields(g, ["solid", "pose"]);
			fields(d, ["degrees", "axis"]);
			fields(axis, ["x", "y", "z"]);
			var openings:Array<CastingOpening> = [];
			for (o in CastingRecordCodec.array(row, "openings")) {
				fields(o, ["kind", "target"]);
				openings.push({kind: cast CastingRecordCodec.string(o, "kind"), target: CastingRecordCodec.string(o, "target")});
			}
			var adjustable:Array<CastingDimension> = [];
			for (value in CastingRecordCodec.array(row, "adjustable")) {
				if (!Std.isOfType(value, String))
					throw "Invalid adjustable dimension";
				adjustable.push(cast value);
			}
			copy.push({
				id: CastingRecordCodec.string(row, "id"),
				kind: cast CastingRecordCodec.string(row, "kind"),
				geometry: {
					solid: CastingFeature.CastingGeometrySolids.snapshot(Reflect.field(g, "solid")),
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
			if (CastingFeatureKinds.values().indexOf(f.kind) < 0)
				throw "Unknown casting feature kind: " + f.id;
			var g = f.geometry;
			fields(g, ["solid", "pose"]);
			switch g.solid {
				case Cut(solid):
					if (f.kind == InsertRecess) throw "Insert recess requires source insert geometry: " + f.id;
					CastingSolid.CastingSolidCodec.validate(solid);
				case Insert(body):
					if (f.kind != InsertRecess) throw "Design cut cannot use insert retention geometry: " + f.id;
					InsertSpec.InsertBodyCodec.validate(body);
			}
			var radiusLimit = switch CastingFeature.CastingGeometrySolids.mouth(g.solid) {
				case Box(width, depth, _): Math.min(width, depth) / 2;
				case Cylinder(diameter, _): diameter / 2;
			};
			AssemblyCodec.validateFrame(g.pose);
			fields(f.draft, ["degrees", "axis"]);
			fields(f.draft.axis, ["x", "y", "z"]);
			var a = f.draft.axis;
			if (!Math.isFinite(f.draft.degrees)
				|| f.draft.degrees < 0
				|| f.draft.degrees >= 45
				|| Math.abs(a.x * a.x + a.y * a.y + a.z * a.z - 1) > 1e-8)
				throw "Invalid casting feature draft: " + f.id;
			if (!Math.isFinite(f.cornerRadius) || f.cornerRadius < 0 || f.cornerRadius > radiusLimit)
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
				if (CastingDimensions.tokens().indexOf(d) < 0 || dimensions.exists(d))
					throw "Invalid adjustable dimension: " + f.id;
				dimensions.set(d, true);
			}
		}
		for (f in copy)
			for (o in f.openings)
				if (o.kind == Feature && (openingTargets(copy, o.target).length == 0 || o.target == f.id || o.target == f.derivedFrom))
					throw "Unknown feature opening: " + f.id;
		copy.sort((a, b) -> Reflect.compare(a.id, b.id));
		return copy;
	}

	/** Openings may name an authored parent after retained supports fragment its cut. */
	public static function openingTargets(features:Array<CastingFeature>, id:String):Array<CastingFeature>
		return features.filter(feature -> feature.id == id || feature.derivedFrom == id);

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
