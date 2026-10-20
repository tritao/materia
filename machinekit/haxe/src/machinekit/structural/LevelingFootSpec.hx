package machinekit.structural;

import cadkit.threads.ThreadSpec;
import machinekit.standard.FastenerThreads;

/** Nominal right-hand metric foot hardware. These dimensions do not qualify a load rating. */
enum FootStemEnvelope {
	Nominal;
	ThreadRoot;
}

enum FootCollar {
	NoCollar;
	RoundCollar(diameter:Float, height:Float, bottom:Float);
}

enum FootNutProfile {
	Round(diameter:Float);
	Hexagon(acrossFlats:Float);
}

/** A complete purchased foot remains one BOM item; loose plates and nuts are separate items.
 * All local solids start at Z=0. Foot mountHeight is an assembly datum, not stem length.
 */
enum LevelingFootSpec {
	Foot(padDiameter:Float, padHeight:Float, thread:ThreadSpec, stemBottom:Float,
		stemTop:Float, mountHeight:Float, envelope:FootStemEnvelope, collar:FootCollar);
	Pad(diameter:Float, height:Float);
	Screw(thread:ThreadSpec, length:Float, envelope:FootStemEnvelope);
	Plate(width:Float, depth:Float, thickness:Float, clearanceDiameter:Float);
	Nut(profile:FootNutProfile, height:Float, thread:ThreadSpec);
}

/** One strict saved representation; previous machine-specific foot recipes are not read. */
class LevelingFootCodec {
	public static inline final VERSION = 1;

	public static function encode(spec:LevelingFootSpec):String {
		var record:Dynamic = switch spec {
			case Foot(d, h, thread, bottom, top, mount, envelope, collar):
				positive([d, h, top, mount]);
				if (!Math.isFinite(bottom) || bottom < 0 || bottom > h || top <= h || mount > top)
					throw "Foot stem must meet the pad and contain the mounting datum";
				if (thread.majorDiameter >= d) throw "Foot stem must be smaller than its pad";
				var c:Dynamic = switch collar {
					case NoCollar: {kind: "none"};
					case RoundCollar(cd, ch, z):
						positive([cd, ch]);
						if (!Math.isFinite(z) || z < h || z + ch > top || cd <= thread.majorDiameter)
							throw "Foot collar must surround the stem";
						{kind: "round", diameter: cd, height: ch, bottom: z};
				};
				{kind: "foot", diameter: d, height: h, thread: threadRecord(thread),
					stemBottom: bottom, stemTop: top, mountHeight: mount, envelope: envelopeToken(envelope), collar: c};
			case Pad(d, h):
				positive([d, h]);
				{kind: "pad", diameter: d, height: h};
			case Screw(thread, length, envelope):
				positive([length]);
				{kind: "screw", thread: threadRecord(thread), length: length, envelope: envelopeToken(envelope)};
			case Plate(w, d, h, bore):
				positive([w, d, h, bore]);
				if (bore >= Math.min(w, d)) throw "Foot plate bore must leave a bearing land";
				{kind: "plate", width: w, depth: d, thickness: h, clearanceDiameter: bore};
			case Nut(profile, height, thread):
				positive([height]);
				var p:Dynamic = switch profile {
					case Round(d): positive([d]); {kind: "round", size: d};
					case Hexagon(af): positive([af]); {kind: "hexagon", size: af};
				};
				if (p.size <= thread.majorDiameter) throw "Foot nut must leave material around its thread";
				{kind: "nut", profile: p, height: height, thread: threadRecord(thread)};
		};
		return haxe.Json.stringify({version: VERSION, part: record});
	}

	public static function decode(text:String):LevelingFootSpec {
		var root:Dynamic = haxe.Json.parse(text);
		fields(root, ["version", "part"]);
		if (number(root, "version") != VERSION) throw "Unsupported leveling-foot schema version";
		var p:Dynamic = root.part;
		var spec:LevelingFootSpec = switch token(p, "kind") {
			case "foot":
				fields(p, ["kind", "diameter", "height", "thread", "stemBottom", "stemTop", "mountHeight", "envelope", "collar"]);
				var c = p.collar;
				var collar:FootCollar = switch token(c, "kind") {
					case "none": fields(c, ["kind"]); NoCollar;
					case "round": fields(c, ["kind", "diameter", "height", "bottom"]);
						RoundCollar(number(c, "diameter"), number(c, "height"), number(c, "bottom"));
					default: throw "Unknown foot collar";
				};
				Foot(number(p, "diameter"), number(p, "height"), readThread(p.thread), number(p, "stemBottom"),
					number(p, "stemTop"), number(p, "mountHeight"), readEnvelope(p), collar);
			case "pad":
				fields(p, ["kind", "diameter", "height"]);
				Pad(number(p, "diameter"), number(p, "height"));
			case "screw":
				fields(p, ["kind", "thread", "length", "envelope"]);
				Screw(readThread(p.thread), number(p, "length"), readEnvelope(p));
			case "plate":
				fields(p, ["kind", "width", "depth", "thickness", "clearanceDiameter"]);
				Plate(number(p, "width"), number(p, "depth"), number(p, "thickness"), number(p, "clearanceDiameter"));
			case "nut":
				fields(p, ["kind", "profile", "height", "thread"]);
				fields(p.profile, ["kind", "size"]);
				var profile:FootNutProfile = switch token(p.profile, "kind") {
					case "round": Round(number(p.profile, "size"));
					case "hexagon": Hexagon(number(p.profile, "size"));
					default: throw "Unknown foot nut profile";
				};
				Nut(profile, number(p, "height"), readThread(p.thread));
			default: throw "Unknown leveling-foot part";
		};
		encode(spec);
		return spec;
	}

	static function threadRecord(t:ThreadSpec):Dynamic {
		if (t == null || t.internal || t.profile != Metric || t.hand != RightHand || t.starts != 1 || t.toleranceClass != "6g")
			throw "Foot hardware requires a single-start right-hand metric external thread";
		if (t.externalMinorDiameter() <= 0) throw "Foot thread must have a positive root diameter";
		return {diameter: t.majorDiameter, pitch: t.pitch};
	}
	static function readThread(p:Dynamic):ThreadSpec {
		fields(p, ["diameter", "pitch"]);
		return FastenerThreads.metric(number(p, "diameter"), number(p, "pitch"));
	}
	static function envelopeToken(e:FootStemEnvelope):String return switch e {
		case Nominal: "nominal";
		case ThreadRoot: "thread-root";
	};
	static function readEnvelope(p:Dynamic):FootStemEnvelope return switch token(p, "envelope") {
		case "nominal": Nominal;
		case "thread-root": ThreadRoot;
		default: throw "Unknown foot stem envelope";
	};
	static function positive(values:Array<Float>):Void {
		for (v in values) if (!Math.isFinite(v) || v <= 0) throw "Foot dimensions must be finite and positive";
	}
	static function fields(p:Dynamic, names:Array<String>):Void {
		if (p == null || !Reflect.isObject(p) || Std.isOfType(p, Array) || Std.isOfType(p, String))
			throw "Foot record must be an object";
		var actual = Reflect.fields(p);
		if (actual.length != names.length) throw "Unexpected leveling-foot fields";
		for (n in names) if (!Reflect.hasField(p, n)) throw 'Missing leveling-foot field $n';
	}
	static function token(p:Dynamic, key:String):String {
		var value:Dynamic = Reflect.field(p, key);
		if (!Std.isOfType(value, String)) throw 'Foot $key must be a token';
		return value;
	}
	static function number(p:Dynamic, key:String):Float {
		var value:Dynamic = Reflect.field(p, key);
		if ((!Std.isOfType(value, Float) && !Std.isOfType(value, Int)) || !Math.isFinite(value))
			throw 'Foot $key must be a finite number';
		return value;
	}
}
