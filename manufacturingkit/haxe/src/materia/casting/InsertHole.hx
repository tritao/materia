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
/** Enlarged blind bore beyond a short entry thread, continuing to hole depth. */
typedef InsertBoreRelief = {var diameter:Float; var start:Float;}
typedef InsertHoleLimits = {
	var policyId:String;
	var threaded:Null<{minEngagement:Float, minTipGap:Float}>;
	var minBack:Float;
	var minWall:Float;
	var minClearance:Float;
	var overrides:Array<{name:String, value:Float}>;
}

/** Source entry frame and depths refer to finished steel. Raw manufacturing
 * states derive extensions through finishing stock without moving service holes.
 */
typedef InsertHole = {
	var id:String;
	var frame:AssemblyFrame;
	var diameter:Float;
	var depth:Float;
	var extent:HoleExtent;
	var thread:Null<InsertThread>;
	var tapDrill:Null<Float>;
	/** Full-thread endpoint measured from the entry face, excluding blind-tap
	 * runout. A counterbore removes the beginning of that threaded interval. */
	var threadDepth:Null<Float>;
	/** ISO 273 medium bolt-clearance diameter, resolved by the source design. */
	var clearanceDiameter:Float;
	var counterbore:Null<InsertCounterbore>;
	var relief:Null<InsertBoreRelief>;
	var limits:InsertHoleLimits;
}

class InsertHoleCodec {
	/** Thread-only design rules are absent on plain bores, never unknown on taps. */
	public static function threadLimits(hole:InsertHole):{minEngagement:Float, minTipGap:Float} {
		if (hole.thread == null || hole.limits.threaded == null)
			throw "Threaded hole requires resolved engagement and tip gap: " + hole.id;
		return cast hole.limits.threaded;
	}

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
				"tapDrill", "clearanceDiameter", "threadDepth",
				"counterbore", "relief", "limits"
			]);
			var t:Dynamic = Reflect.field(row, "thread"),
				cb:Dynamic = Reflect.field(row, "counterbore");
			var thread:Null<InsertThread> = t==null ? null : InsertThreadCodec.snapshot(t);
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
				clearanceDiameter: CastingRecordCodec.number(row, "clearanceDiameter"),
				threadDepth: Reflect.field(row, "threadDepth") == null ? null : CastingRecordCodec.number(row, "threadDepth"),
				counterbore: counterbore,
				relief: relief(Reflect.field(row, "relief")),
				limits: limits(Reflect.field(row, "limits"), thread != null)
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
				"tapDrill", "clearanceDiameter", "threadDepth",
				"counterbore", "relief", "limits"
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
			if (!Math.isFinite(h.clearanceDiameter) || h.clearanceDiameter < (h.thread == null ? h.diameter : h.thread.majorDiameter))
				throw "Invalid insert bolt clearance diameter: " + h.id;
			if (h.thread != null) {
				var thread:InsertThread = cast h.thread;
				if (thread.designation.length == 0 || thread.toleranceClass.length == 0 || thread.majorDiameter <= 0 || thread.pitch <= 0 || thread.starts < 1)
					throw "Invalid insert thread";
				if (h.threadDepth == null || h.threadDepth <= 0 || h.threadDepth > h.depth)
					throw "Invalid usable insert thread depth: " + h.id;
				var counterbore=h.counterbore;
				var threadStart=counterbore==null ? 0.0 : counterbore.depth;
				if (threadLimits(h).minEngagement > (cast h.threadDepth:Float)-threadStart+1e-8)
					throw "Insert thread cannot meet engagement rule: " + h.id;
				if (!thread.internal
					|| h.tapDrill == null
					|| h.tapDrill <= 0
					|| h.tapDrill >= thread.majorDiameter
					|| Math.abs(h.tapDrill - h.diameter) > 1e-8)
					throw "Invalid insert tapping geometry";
			} else if (h.tapDrill != null || h.threadDepth != null)
				throw "Tap drill requires a thread specification";
			if (h.relief != null) {
				var r:InsertBoreRelief = cast h.relief;
				if (h.thread == null || h.extent != Blind || h.counterbore != null
					|| r.start <= 0 || r.start >= h.depth || Math.abs(r.start - (cast h.threadDepth:Float)) > 1e-8
					|| r.diameter <= h.thread.majorDiameter)
					throw "Invalid insert thread relief: " + h.id;
			}
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

	static function relief(raw:Dynamic):Null<InsertBoreRelief> {
		if (raw == null) return null;
		CastingFeatureCodec.fields(raw, ["diameter", "start"]);
		return {diameter: CastingRecordCodec.number(raw, "diameter"), start: CastingRecordCodec.number(raw, "start")};
	}

	static function limits(raw:Dynamic, threaded:Bool):InsertHoleLimits {
		var names = ["policyId", "threaded", "minBack", "minWall", "minClearance", "overrides"];
		CastingFeatureCodec.fields(raw, names);
		var result:InsertHoleLimits = {
			policyId: CastingRecordCodec.string(raw, "policyId"),
			threaded: null,
			minBack: CastingRecordCodec.number(raw, "minBack"),
			minWall: CastingRecordCodec.number(raw, "minWall"),
			minClearance: CastingRecordCodec.number(raw, "minClearance"),
			overrides: []
		};
		if (threaded) {
			var rules:Dynamic = Reflect.field(raw, "threaded");
			CastingFeatureCodec.fields(rules, ["minEngagement", "minTipGap"]);
			var engagement = CastingRecordCodec.number(rules, "minEngagement"), tip = CastingRecordCodec.number(rules, "minTipGap");
			if (engagement <= 0 || tip <= 0) throw "Invalid threaded insert limits";
			result.threaded = {minEngagement: engagement, minTipGap: tip};
		}
		else if (Reflect.field(raw, "threaded") != null) throw "Plain insert bore cannot carry thread limits";
		CastingFeatureCodec.identifier(result.policyId);
		for (name in ["minBack", "minWall", "minClearance"]) {
			var value:Float = Reflect.field(result, name);
			if (!Math.isFinite(value) || value <= 0)
				throw "Invalid insert hole limit: " + name;
		}
		if (!Std.isOfType(raw.overrides, Array))
			throw "Invalid insert hole limit overrides";
		var rows:Array<Dynamic> = cast raw.overrides, seen = new Map<String, Bool>();
		for (row in rows) {
			CastingFeatureCodec.fields(row, ["name", "value"]);
			var name = CastingRecordCodec.string(row, "name"), value = CastingRecordCodec.number(row, "value");
			if ((threaded ? ["minEngagement", "minTipGap", "minBack", "minWall", "minClearance"] : ["minBack", "minWall", "minClearance"]).indexOf(name) < 0
				|| seen.exists(name) || value != Reflect.field(name == "minEngagement" || name == "minTipGap" ? result.threaded : result, name))
				throw "Inconsistent insert hole limit override: " + name;
			seen.set(name, true);
			result.overrides.push({name: name, value: value});
		}
		result.overrides.sort((a, b) -> Reflect.compare(a.name, b.name));
		return result;
	}
}
