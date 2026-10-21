package materia.casting;

import materia.casting.InsertHole.InsertThread;
import materia.casting.InsertHole.InsertThreadHand;
import materia.casting.InsertHole.InsertThreadProfile;

/** Shared nominal thread encoding for source bores and real external screws. */
class InsertThreadCodec {
	public static function snapshot(t:Dynamic):InsertThread {
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
		var thread:InsertThread = {
			designation: CastingRecordCodec.string(t, "designation"),
			majorDiameter: CastingRecordCodec.number(t, "majorDiameter"),
			pitch: CastingRecordCodec.number(t, "pitch"),
			starts: Std.int(starts),
			hand: hand,
			internal: cast internal,
			toleranceClass: CastingRecordCodec.string(t, "toleranceClass"),
			profile: profile
		};
		if (StringTools.trim(thread.designation).length==0 || StringTools.trim(thread.toleranceClass).length==0
			|| thread.majorDiameter<=0 || thread.pitch<=0 || thread.starts<1 || !Math.isFinite(thread.pitch*thread.starts))
			throw "Invalid insert thread";
		return thread;
	}
}
