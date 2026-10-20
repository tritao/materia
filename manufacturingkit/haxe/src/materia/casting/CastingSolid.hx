package materia.casting;

/** Solid primitive in its own frame: centred in XY, from Z=0 to height.
 * Insert bodies and design cuts share this geometry. Insert holes remain separate.
 * A cylinder is exact geometry, never its bounding box for box-only analysis.
 */
enum CastingSolid {
	Box(width:Float, depth:Float, height:Float);
	Cylinder(diameter:Float, height:Float);
}

/** One neutral encoding for solid primitives. The containing document owns versioning. */
class CastingSolidCodec {
	public static function encode(body:CastingSolid):String
		return haxe.Json.stringify(record(body));

	public static function decode(text:String):CastingSolid
		return snapshot(haxe.Json.parse(text));

	/** Explicit box-only gate; never projects a round body onto a rectangular one. */
	public static function box(body:CastingSolid, id:String):{width:Float, depth:Float, height:Float} {
		validate(body);
		return switch body {
			case Box(width, depth, height): {width: width, depth: depth, height: height};
			case Cylinder(_, _): throw "Unsupported non-box solid: " + id;
		};
	}

	public static function height(body:CastingSolid):Float {
		validate(body);
		return switch body {
			case Box(_, _, height), Cylinder(_, height): height;
		};
	}

	public static function snapshot(raw:Dynamic):CastingSolid {
		if (Std.isOfType(raw, CastingSolid))
			raw = record(cast raw);
		var kind = CastingRecordCodec.string(raw, "kind");
		var body = switch kind {
			case "box":
				CastingFeatureCodec.fields(raw, ["kind", "width", "depth", "height"]);
				Box(CastingRecordCodec.number(raw, "width"), CastingRecordCodec.number(raw, "depth"), CastingRecordCodec.number(raw, "height"));
			case "cylinder":
				CastingFeatureCodec.fields(raw, ["kind", "diameter", "height"]);
				Cylinder(CastingRecordCodec.number(raw, "diameter"), CastingRecordCodec.number(raw, "height"));
			default:
				throw "Unsupported casting solid kind: " + kind;
		};
		validate(body);
		return body;
	}

	public static function record(body:CastingSolid):Dynamic {
		validate(body);
		return switch body {
			case Box(width, depth, height): {kind: "box", width: width, depth: depth, height: height};
			case Cylinder(diameter, height): {kind: "cylinder", diameter: diameter, height: height};
		};
	}

	public static function validate(body:CastingSolid):Void {
		if (body == null)
			throw "Missing casting solid";
		var dimensions = switch body {
			case Box(width, depth, height): [width, depth, height];
			case Cylinder(diameter, height): [diameter, height];
		};
		for (dimension in dimensions)
			if (!Math.isFinite(dimension) || dimension <= 0)
				throw "Casting solid dimensions must be finite and positive";
	}
}
