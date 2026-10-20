package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;

enum abstract HoleExtent(String) to String {
	var Through = "through";
	var Blind = "blind";
}

/** Drilling frame +Z points into the insert from its entry face. */
enum abstract InsertThreadHand(String) to String {
	var Right = "right";
	var Left = "left";
}

enum abstract InsertThreadProfile(String) to String {
	var Metric = "metric";
	var Trapezoidal = "trapezoidal";
	var Acme = "acme";
	var Ball = "ball";
}

typedef InsertThread = {var designation:String; var majorDiameter:Float; var pitch:Float; var starts:Int; var hand:InsertThreadHand; var internal:Bool; var toleranceClass:String; var profile:InsertThreadProfile;}
typedef InsertCounterbore = {var diameter:Float; var depth:Float;}

typedef InsertHole = {
	var id:String;
	var frame:AssemblyFrame;
	var diameter:Float;
	var depth:Float;
	var extent:HoleExtent;
	var thread:Null<InsertThread>;
	var tapDrill:Null<Float>;
	var counterbore:Null<InsertCounterbore>;
}

class InsertHoleCodec {
	public static function snapshot(raw:Dynamic):Array<InsertHole> {
		if (!Std.isOfType(raw, Array))
			throw "Invalid insert hole list";
		var normalized:Dynamic = haxe.Json.parse(haxe.Json.stringify(raw)),
			rows:Array<Dynamic> = cast normalized;
		if (rows.length > 256)
			throw "Too many insert holes";
		var result:Array<InsertHole> = [], ids = new Map<String, Bool>();
		for (row in rows) {
			CastingFeatureCodec.fields(row, [
				"id",
				"frame",
				"diameter",
				"depth",
				"extent",
				"thread",
				"tapDrill",
				"counterbore"
			]);
			var t:Dynamic = Reflect.field(row, "thread"),
				cb:Dynamic = Reflect.field(row, "counterbore");
			var thread:Null<InsertThread> = null;
			if (t != null) {
				CastingFeatureCodec.fields(t, [
					"designation",
					"majorDiameter",
					"pitch",
					"starts",
					"hand",
					"internal",
					"toleranceClass",
					"profile"
				]);
				var hand = switch CastingRecordCodec.string(t, "hand") {
					case "right": Right;
					case "left": Left;
					default: throw "Invalid thread hand";
				};
				var profile = switch CastingRecordCodec.string(t, "profile") {
					case "metric": Metric;
					case "trapezoidal": Trapezoidal;
					case "acme": Acme;
					case "ball": Ball;
					default: throw "Invalid thread profile";
				};
				var starts = CastingRecordCodec.number(t, "starts"),
					internal:Dynamic = Reflect.field(t, "internal");
				if (starts != Math.floor(starts) || !Std.isOfType(internal, Bool))
					throw "Invalid thread specification";
				thread = {
					designation: CastingRecordCodec.string(t, "designation"),
					majorDiameter: CastingRecordCodec.number(t, "majorDiameter"),
					pitch: CastingRecordCodec.number(t, "pitch"),
					starts: Std.int(starts),
					hand: hand,
					internal: cast internal,
					toleranceClass: CastingRecordCodec.string(t, "toleranceClass"),
					profile: profile
				};
			}
			var counterbore:Null<InsertCounterbore> = null;
			if (cb != null) {
				CastingFeatureCodec.fields(cb, ["diameter", "depth"]);
				counterbore = {diameter: CastingRecordCodec.number(cb, "diameter"), depth: CastingRecordCodec.number(cb, "depth")};
			}
			result.push({
				id: CastingRecordCodec.string(row, "id"),
				frame: CastingRecordCodec.frame(Reflect.field(row, "frame")),
				diameter: CastingRecordCodec.number(row, "diameter"),
				depth: CastingRecordCodec.number(row, "depth"),
				extent: cast CastingRecordCodec.string(row, "extent"),
				thread: thread,
				tapDrill: Reflect.field(row, "tapDrill") == null ? null : CastingRecordCodec.number(row, "tapDrill"),
				counterbore: counterbore
			});
		}
		for (h in result) {
			CastingFeatureCodec.fields(h, [
				"id",
				"frame",
				"diameter",
				"depth",
				"extent",
				"thread",
				"tapDrill",
				"counterbore"
			]);
			CastingFeatureCodec.identifier(h.id);
			if (ids.exists(h.id))
				throw "Duplicate insert hole";
			ids.set(h.id, true);
			materia.assembly.AssemblyCodec.validateFrame(h.frame);
			if (!Math.isFinite(h.diameter + h.depth) || h.diameter <= 0 || h.depth <= 0)
				throw "Invalid insert hole dimensions";
			if (h.extent != Through && h.extent != Blind)
				throw "Invalid insert hole extent";
			if (h.thread != null) {
				var thread:InsertThread = cast h.thread;
				if (thread.designation.length == 0 || thread.toleranceClass.length == 0 || thread.majorDiameter <= 0 || thread.pitch <= 0 || thread.starts < 1)
					throw "Invalid insert thread";
				if (!thread.internal
					|| h.tapDrill == null
					|| h.tapDrill <= 0
					|| h.tapDrill >= thread.majorDiameter
					|| Math.abs(h.tapDrill - h.diameter) > 1e-8)
					throw "Invalid insert tapping geometry";
			} else if (h.tapDrill != null)
				throw "Tap drill requires a thread specification";
			if (h.counterbore != null) {
				var cb:InsertCounterbore = cast h.counterbore;
				CastingFeatureCodec.fields(cb, ["diameter", "depth"]);
				if (!Math.isFinite(cb.diameter + cb.depth) || cb.diameter <= h.diameter || cb.depth <= 0 || cb.depth >= h.depth)
					throw "Invalid insert counterbore";
			}
		}
		result.sort((a, b) -> Reflect.compare(a.id, b.id));
		return result;
	}
}
