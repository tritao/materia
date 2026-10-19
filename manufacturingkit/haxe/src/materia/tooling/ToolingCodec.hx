package materia.tooling;

import haxe.Json;
import haxe.crypto.Sha256;
import materia.assembly.AssemblyCodec;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingPrism.ToolingPoint2;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.casting.CastingToolingInput.CastingToolingFeature;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import materia.tooling.casting.CastingToolingInput.CastingFeatureKind;
import materia.tooling.casting.CastingSetup;
import materia.tooling.casting.CastingSetup.CoreStrategy;
import materia.tooling.casting.CastingSetup.CastingFeatureStrategy;

/** Strict, detached and deterministic records. No native geometry or inventory effects.
 * Named collections are sorted by ID; profile vertices retain their geometric order.
 * Inputs are explicitly millimetres; other units require conversion before this boundary.
 */
class ToolingCodec {
	public static inline var FORMAT = "materia.casting-tooling";
	public static inline var VERSION = 1;
	public static inline var MAX_RECORDS = 512;
	public static inline var MAX_TEXT_BYTES = 2097152;

	public static function encode(document:ToolingDocument):String {
		var text = canonical({format: FORMAT, version: VERSION, document: snapshot(document)}) + "\n";
		checkSize(text);
		return text;
	}

	public static function decode(text:String):ToolingDocument {
		checkSize(text);
		var raw:Dynamic = Json.parse(text);
		object(raw, ["format", "version", "document"], "tooling");
		if (string(raw, "format") != FORMAT || integer(raw, "version") != VERSION)
			throw "Unsupported tooling format or version";
		return readDocument(Reflect.field(raw, "document"));
	}

	/** Accepts typed records through exactly the same validation as decoded JSON. */
	public static function snapshot(document:ToolingDocument):ToolingDocument return readDocument(document);

	static function readDocument(raw:Dynamic):ToolingDocument {
		object(raw, ["input", "setup", "recipe"], "document");
		var input = readInput(Reflect.field(raw, "input"));
		return {input: input, setup: readSetup(Reflect.field(raw, "setup"), input),
			recipe: readRecipe(Reflect.field(raw, "recipe"))};
	}

	public static function inputFingerprint(input:CastingToolingInput):String
		return Sha256.encode(canonical(inputSnapshot(input)));

	public static function recipeFingerprint(recipe:ToolingRecipe):String
		return Sha256.encode(canonical(recipeSnapshot(recipe)));

	public static function documentFingerprint(document:ToolingDocument):String
		return Sha256.encode(encode(document));

	public static function inputSnapshot(input:CastingToolingInput):CastingToolingInput return readInput(input);

	static function readInput(raw:Dynamic):CastingToolingInput {
		object(raw, ["id", "revision", "lengthUnit", "envelope", "features", "inserts"], "input");
		if (string(raw, "lengthUnit") != "mm") throw "Tooling input requires explicit mm units";
		var envelope:Array<ToolingPrism> = [for (p in array(raw, "envelope")) readPrism(p)];
		if (envelope.length == 0) throw "Tooling input needs an envelope";
		var features:Array<CastingToolingFeature> = [for (f in array(raw, "features")) readFeature(f)];
		var inserts:Array<CastingToolingInsert> = [for (i in array(raw, "inserts")) readInsert(i)];
		unique(envelope, p -> p.id, "envelope");
		unique(features, f -> f.id, "features");
		unique(inserts, i -> i.id, "inserts");
		var byId = new Map<String, CastingToolingInsert>();
		for (i in inserts) byId.set(i.id, i);
		var referenced = new Map<String, Bool>();
		for (f in features) {
			if (f.kind == Void) {
				if (f.insertId != null) throw 'Void ${f.id} cannot reference an insert';
			} else {
				var i = f.insertId == null ? null : byId.get(f.insertId);
				if (i == null || referenced.exists(f.insertId)) throw 'Invalid insert reference on ${f.id}';
				if (f.width != i.width || f.depth != i.depth || f.height != i.height || !sameFrame(f.pose, i.pose))
					throw 'Insert recess ${f.id} does not match insert ${i.id}';
				referenced.set(i.id, true);
			}
		}
		for (i in inserts) if (!referenced.exists(i.id)) throw 'Missing recess for insert ${i.id}';
		return {id: string(raw, "id"), revision: revision(raw), lengthUnit: "mm",
			envelope: envelope, features: features, inserts: inserts};
	}

	static function readPrism(raw:Dynamic):ToolingPrism {
		object(raw, ["id", "profile", "height", "pose"], "prism");
		var profile:Array<ToolingPoint2> = [for (p in array(raw, "profile", 64)) {
			object(p, ["x", "y"], "profile point");
			{x: number(p, "x"), y: number(p, "y")};
		}];
		if (profile.length < 3) throw "Prism needs at least three profile vertices";
		// Every other vertex must lie strictly left of every directed edge. This
		// rejects clockwise, degenerate, concave and self-intersecting profiles.
		for (n in 0...profile.length) {
			var a = profile[n], b = profile[(n + 1) % profile.length];
			for (k in 0...profile.length) if (k != n && k != (n + 1) % profile.length) {
				var c = profile[k], cross = (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
				if (!Math.isFinite(cross) || cross <= 0) throw "Prism profile must be strictly convex and CCW";
			}
		}
		return {id: string(raw, "id"), profile: profile, height: positive(raw, "height"), pose: frame(raw, "pose")};
	}

	static function readFeature(raw:Dynamic):CastingToolingFeature {
		object(raw, ["id", "kind", "insertId", "width", "depth", "height", "pose"], "feature");
		var kind:CastingFeatureKind = switch string(raw, "kind") {
			case "void": Void;
			case "insert-recess": InsertRecess;
			default: throw "Unsupported casting feature kind";
		};
		return {id: string(raw, "id"), kind: kind, insertId: nullableString(raw, "insertId"),
			width: positive(raw, "width"), depth: positive(raw, "depth"), height: positive(raw, "height"), pose: frame(raw, "pose")};
	}

	static function readInsert(raw:Dynamic):CastingToolingInsert {
		object(raw, ["id", "width", "depth", "height", "boreDiameter", "pose", "role", "positionTolerance", "finish"], "insert");
		var w = positive(raw, "width"), d = positive(raw, "depth"), bore = nonnegative(raw, "boreDiameter");
		if (bore >= Math.min(w, d)) throw "Insert bore consumes its seat";
		return {id: string(raw, "id"), width: w, depth: d, height: positive(raw, "height"), boreDiameter: bore,
			pose: frame(raw, "pose"), role: string(raw, "role"), positionTolerance: positive(raw, "positionTolerance"),
			finish: string(raw, "finish")};
	}

	static function readSetup(raw:Dynamic, input:CastingToolingInput):CastingSetup {
		object(raw, ["id", "revision", "sourceFingerprint", "sourceToMold", "features"], "setup");
		var fingerprint = string(raw, "sourceFingerprint");
		if (fingerprint != inputFingerprint(input)) throw "Stale tooling setup: source fingerprint does not match";
		var byId = new Map<String, CastingToolingFeature>();
		for (f in input.features) if (f.kind == Void) byId.set(f.id, f);
		var used = new Map<String, Bool>();
		var features:Array<CastingFeatureStrategy> = [];
		for (f in array(raw, "features")) {
			object(f, ["featureId", "strategy", "withdrawal"], "feature strategy");
			var id = string(f, "featureId");
			if (!byId.exists(id) || used.exists(id)) throw 'Unknown, duplicate or insert-recess strategy $id';
			used.set(id, true);
			var strategy:CoreStrategy = switch string(f, "strategy") {
				case "sacrificial-eps": SacrificialEPS;
				case "reusable-former": ReusableFormer;
				case "unsupported": Unsupported;
				default: throw "Unsupported core strategy token";
			};
			var direction:Null<{var x:Float; var y:Float; var z:Float;}> = null;
			if (Reflect.field(f, "withdrawal") != null) {
				var v = Reflect.field(f, "withdrawal"); object(v, ["x", "y", "z"], "withdrawal");
				direction = {x: number(v, "x"), y: number(v, "y"), z: number(v, "z")};
				var norm = direction.x*direction.x + direction.y*direction.y + direction.z*direction.z;
				if (Math.abs(norm - 1) > 1e-8) throw "Withdrawal direction must be unit length";
			}
			if (strategy == ReusableFormer && direction == null) throw "Reusable former needs a withdrawal direction";
			features.push({featureId: id, strategy: strategy, withdrawal: direction});
		}
		for (id in byId.keys()) if (!used.exists(id)) throw 'Missing core strategy for $id';
		features.sort((a, b) -> Reflect.compare(a.featureId, b.featureId));
		return {id: string(raw, "id"), revision: revision(raw), sourceFingerprint: fingerprint,
			sourceToMold: frame(raw, "sourceToMold"), features: features};
	}

	public static function recipeSnapshot(recipe:ToolingRecipe):ToolingRecipe return readRecipe(recipe);

	static function readRecipe(raw:Dynamic):ToolingRecipe {
		object(raw, ["id", "revision", "panel", "core", "jointClearance", "linerThickness", "routerDiameter",
			"minimumLigament", "resinSpec", "releaseSpec", "adhesiveSpec", "processEvidence"], "recipe");
		var p = Reflect.field(raw, "panel"), c = Reflect.field(raw, "core");
		object(p, ["materialSpec", "nominalThickness", "measuredThickness", "thicknessVariation", "width", "height",
			"coatedFace", "allowedRotations", "directionConstraint"], "panel stock");
		object(c, ["materialSpec", "width", "depth", "height", "densityKgM3", "compressionStrengthPa",
			"compressionStrainFraction", "maximumTemperatureC", "evidence"], "core stock");
		var thickness = positive(p, "measuredThickness"), variation = nonnegative(p, "thicknessVariation");
		if (variation >= thickness) throw "Panel thickness variation consumes stock";
		var face = string(p, "coatedFace");
		if (["positive-z", "negative-z", "both", "none"].indexOf(face) < 0) throw "Invalid coated face";
		var rotations:Array<Int> = [];
		for (value in array(p, "allowedRotations", 2)) {
			var rotation = integer({value: value}, "value");
			if ((rotation != 0 && rotation != 90) || rotations.indexOf(rotation) >= 0)
				throw "Invalid or duplicate panel stock rotation";
			rotations.push(rotation);
		}
		rotations.sort(Reflect.compare);
		var direction = string(p, "directionConstraint");
		if (rotations.length == 0 || (direction != "free" && direction != "width-aligned") ||
			(direction == "width-aligned" && rotations.indexOf(90) >= 0)) throw "Invalid panel direction constraint";
		var strength = nullableNumber(c, "compressionStrengthPa", true), strain = nullableNumber(c, "compressionStrainFraction", true);
		if ((strength == null) != (strain == null) || (strain != null && strain >= 1))
			throw "Compression strength requires a strain fraction between zero and one";
		return {id: string(raw, "id"), revision: revision(raw),
			panel: {materialSpec: string(p, "materialSpec"), nominalThickness: positive(p, "nominalThickness"),
				measuredThickness: thickness, thicknessVariation: variation, width: positive(p, "width"),
				height: positive(p, "height"), coatedFace: face, allowedRotations: rotations, directionConstraint: direction},
			core: {materialSpec: string(c, "materialSpec"), width: positive(c, "width"), depth: positive(c, "depth"),
				height: positive(c, "height"), densityKgM3: nullableNumber(c, "densityKgM3", true),
				compressionStrengthPa: strength, compressionStrainFraction: strain,
				maximumTemperatureC: nullableNumber(c, "maximumTemperatureC", false), evidence: evidence(c, "evidence")},
			jointClearance: nonnegative(raw, "jointClearance"), linerThickness: nonnegative(raw, "linerThickness"),
			routerDiameter: positive(raw, "routerDiameter"), minimumLigament: positive(raw, "minimumLigament"),
			resinSpec: string(raw, "resinSpec"), releaseSpec: string(raw, "releaseSpec"),
			adhesiveSpec: string(raw, "adhesiveSpec"), processEvidence: evidence(raw, "processEvidence")};
	}

	static function object(raw:Dynamic, fields:Array<String>, label:String):Void {
		if (raw == null || !Reflect.isObject(raw) || Std.isOfType(raw, Array) || Std.isOfType(raw, String))
			throw 'Expected tooling object: $label';
		for (key in Reflect.fields(raw)) if (fields.indexOf(key) < 0) throw 'Unknown $label field: $key';
		for (key in fields) if (!Reflect.hasField(raw, key)) throw 'Missing $label field: $key';
	}

	static function array(raw:Dynamic, key:String, limit:Int = MAX_RECORDS):Array<Dynamic> {
		var value:Dynamic = Reflect.field(raw, key);
		if (!Std.isOfType(value, Array)) throw 'Expected tooling array: $key';
		var result:Array<Dynamic> = cast value;
		if (result.length > limit) throw 'Too many tooling records: $key';
		return result;
	}

	static function string(raw:Dynamic, key:String):String {
		var value:Dynamic = Reflect.field(raw, key);
		if (!Std.isOfType(value, String)) throw 'Expected tooling text: $key';
		var result:String = cast value;
		if (StringTools.trim(result).length == 0 || result.length > 2048 || AssemblyCodec.containsNul(result))
			throw 'Invalid tooling text: $key';
		return result;
	}

	static function nullableString(raw:Dynamic, key:String):Null<String>
		return Reflect.field(raw, key) == null ? null : string(raw, key);

	static function number(raw:Dynamic, key:String):Float {
		var value:Dynamic = Reflect.field(raw, key);
		if ((!Std.isOfType(value, Float) && !Std.isOfType(value, Int)) || !Math.isFinite(value))
			throw 'Expected finite tooling number: $key';
		return value;
	}

	static function integer(raw:Dynamic, key:String):Int {
		var n = number(raw, key);
		if (n < 0 || n > 2147483647 || n != Math.floor(n)) throw 'Expected tooling integer: $key';
		return Std.int(n);
	}

	static function revision(raw:Dynamic):Int {
		var n = integer(raw, "revision");
		if (n < 1) throw "Tooling revision must be positive";
		return n;
	}

	static function positive(raw:Dynamic, key:String):Float {
		var n = number(raw, key); if (n <= 0) throw 'Expected positive tooling dimension: $key'; return n;
	}

	static function nonnegative(raw:Dynamic, key:String):Float {
		var n = number(raw, key); if (n < 0) throw 'Expected nonnegative tooling value: $key'; return n;
	}

	static function nullableNumber(raw:Dynamic, key:String, mustBePositive:Bool):Null<Float>
		return Reflect.field(raw, key) == null ? null : (mustBePositive ? positive(raw, key) : number(raw, key));

	static function evidence(raw:Dynamic, key:String):Array<String> {
		var result:Array<String> = [];
		for (value in array(raw, key, 64)) {
			var text = string({value: value}, "value");
			if (result.indexOf(text) >= 0) throw 'Duplicate evidence reference: $text';
			result.push(text);
		}
		result.sort(Reflect.compare);
		return result;
	}

	static function frame(raw:Dynamic, key:String):AssemblyFrame {
		var p = Reflect.field(raw, key);
		object(p, ["x", "y", "z", "qx", "qy", "qz", "qw"], key);
		var result:AssemblyFrame = {x: number(p, "x"), y: number(p, "y"), z: number(p, "z"),
			qx: number(p, "qx"), qy: number(p, "qy"), qz: number(p, "qz"), qw: number(p, "qw")};
		AssemblyCodec.validateFrame(result);
		var norm = result.qx*result.qx + result.qy*result.qy + result.qz*result.qz + result.qw*result.qw;
		if (Math.abs(norm - 1) > 1e-8) throw "Tooling frame requires a unit quaternion";
		return result;
	}

	static function sameFrame(a:AssemblyFrame, b:AssemblyFrame):Bool {
		if (Math.abs(a.x-b.x) > 1e-8 || Math.abs(a.y-b.y) > 1e-8 || Math.abs(a.z-b.z) > 1e-8) return false;
		var dot = a.qx*b.qx + a.qy*b.qy + a.qz*b.qz + a.qw*b.qw;
		var sign = dot < 0 ? -1.0 : 1.0;
		return Math.abs(a.qx-sign*b.qx) <= 1e-8 && Math.abs(a.qy-sign*b.qy) <= 1e-8 &&
			Math.abs(a.qz-sign*b.qz) <= 1e-8 && Math.abs(a.qw-sign*b.qw) <= 1e-8;
	}

	static function checkSize(text:String):Void {
		if (text == null || text.length > MAX_TEXT_BYTES || haxe.io.Bytes.ofString(text).length > MAX_TEXT_BYTES)
			throw "Tooling document exceeds size limit";
	}

	static function unique<T>(items:Array<T>, key:T->String, label:String):Void {
		var ids = new Map<String, Bool>();
		for (item in items) {
			var id = key(item);
			if (ids.exists(id)) throw 'Duplicate tooling $label ID: $id';
			ids.set(id, true);
		}
		items.sort((a, b) -> Reflect.compare(key(a), key(b)));
	}

	/** Sort object keys as well as named records so field construction order is irrelevant. */
	static function canonical(value:Dynamic):String {
		if (value == null) return "null";
		if (Std.isOfType(value, String) || Std.isOfType(value, Bool) || Std.isOfType(value, Float) || Std.isOfType(value, Int))
			return Json.stringify(value);
		if (Std.isOfType(value, Array)) {
			var values:Array<Dynamic> = cast value;
			return "[" + [for (v in values) canonical(v)].join(",") + "]";
		}
		var keys = Reflect.fields(value); keys.sort(Reflect.compare);
		return "{" + [for (key in keys) Json.stringify(key) + ":" + canonical(Reflect.field(value, key))].join(",") + "}";
	}
}
