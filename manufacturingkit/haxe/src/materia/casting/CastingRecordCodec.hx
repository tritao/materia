package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;

class CastingRecordCodec {
	/** Assembly instance paths qualify local IDs without allowing empty or parent segments. */
	public static function referencePath(value:String):Void {
		if (value==null || value.length==0) throw "Invalid casting reference path";
		for (segment in value.split("/")) {
			if (segment=="." || segment=="..") throw "Invalid casting reference path";
			CastingFeatureCodec.identifier(segment);
		}
	}

	public static function number(raw:Dynamic, name:String):Float {
		var v:Dynamic = Reflect.field(raw, name);
		if (!(Std.isOfType(v, Int) || Std.isOfType(v, Float)))
			throw "Invalid casting number: " + name;
		var n:Float = cast v;
		if (!Math.isFinite(n))
			throw "Nonfinite casting number: " + name;
		return n;
	}

	public static function integer(raw:Dynamic, name:String):Int {
		var n = number(raw, name);
		if (n != Math.floor(n))
			throw "Invalid casting integer: " + name;
		return Std.int(n);
	}

	public static function string(raw:Dynamic, name:String):String {
		var v:Dynamic = Reflect.field(raw, name);
		if (!Std.isOfType(v, String))
			throw "Invalid casting text: " + name;
		return cast v;
	}

	public static function optionalString(raw:Dynamic, name:String):Null<String>
		return Reflect.field(raw, name) == null ? null : string(raw, name);

	public static function array(raw:Dynamic, name:String):Array<Dynamic> {
		var v:Dynamic = Reflect.field(raw, name);
		if (!Std.isOfType(v, Array))
			throw "Invalid casting array: " + name;
		return cast v;
	}

	public static function identifiers(row:Dynamic, field:String, maximum:Int = 512):Array<String> {
		var values:Array<String> = [];
		var raw = CastingRecordCodec.array(row, field);
		if (raw.length > maximum) throw "Too many casting references: " + field;
		for (value in raw) {
			if (!Std.isOfType(value, String)) throw "Invalid casting reference: " + field;
			var id:String = cast value;
			CastingFeatureCodec.identifier(id);
			if (values.indexOf(id) >= 0) throw "Duplicate casting reference: " + field;
			values.push(id);
		}
		values.sort(Reflect.compare);
		return values;
	}


	public static function frame(raw:Dynamic):AssemblyFrame {
		CastingFeatureCodec.fields(raw, ["x", "y", "z", "qx", "qy", "qz", "qw"]);
		var f = {
			x: number(raw, "x"),
			y: number(raw, "y"),
			z: number(raw, "z"),
			qx: number(raw, "qx"),
			qy: number(raw, "qy"),
			qz: number(raw, "qz"),
			qw: number(raw, "qw")
		};
		materia.assembly.AssemblyCodec.validateFrame(f);
		return f;
	}
}
