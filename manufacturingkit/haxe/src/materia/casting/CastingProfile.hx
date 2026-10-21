package materia.casting;

import materia.casting.CastingPrism.CastingPoint2;

enum abstract CastingCurveKind(String) to String {
	var Line = "line";
	var Arc = "arc";
}

/** Directed exact curve. An arc stores its centre and signed sweep; a line
 * stores null/zero. A circle uses two distinct-ended half arcs. */
typedef CastingSegment = {
	var kind:CastingCurveKind;
	var start:CastingPoint2;
	var end:CastingPoint2;
	var center:Null<CastingPoint2>;
	var sweep:Float;
}

/** Strict neutral records. The containing feature owns schema versioning. */
class CastingProfileCodec {
	public static function snapshot(raw:Dynamic):Array<CastingSegment> {
		if (!Std.isOfType(raw,Array)) throw "Invalid casting profile";
		var rows:Array<Dynamic> =cast raw,result:Array<CastingSegment> =[];
		for (row in rows) {
			CastingFeatureCodec.fields(row,["kind","start","end","center","sweep"]);
			result.push({kind:cast CastingRecordCodec.string(row,"kind"),
				start:point(Reflect.field(row,"start")),end:point(Reflect.field(row,"end")),
				center:Reflect.field(row,"center")==null ? null : point(Reflect.field(row,"center")),
				sweep:CastingRecordCodec.number(row,"sweep")});
		}
		CastingProfileGeometry.validate(result);
		return result;
	}
	static function point(raw:Dynamic):CastingPoint2 {
		CastingFeatureCodec.fields(raw,["x","y"]);
		return {x:CastingRecordCodec.number(raw,"x"),y:CastingRecordCodec.number(raw,"y")};
	}
}
