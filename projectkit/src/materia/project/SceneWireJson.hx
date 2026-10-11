package materia.project;

/** Preserve JSON numbers before haxeon's integer-token parser can truncate them to Int32. */
class SceneWireJson {
	public static function preserveNumbers(source:String):String {
		var output = new StringBuf(), copied = 0, index = 0;
		var quoted = false, escaped = false;
		while (index < source.length) {
			var code = source.charCodeAt(index);
			if (quoted) {
				if (escaped) escaped = false;
				else if (code == 92) escaped = true;
				else if (code == 34) quoted = false;
				index++; continue;
			}
			if (code == 34) {quoted = true; index++; continue;}
			if (code != 45 && !digit(code)) {index++; continue;}
			var start = index, floating = false;
			if (code == 45) index++;
			while (index < source.length && digit(source.charCodeAt(index))) index++;
			if (index < source.length && source.charCodeAt(index) == 46) {
				floating = true; index++;
				while (index < source.length && digit(source.charCodeAt(index))) index++;
			}
			if (index < source.length && (source.charCodeAt(index) == 69 || source.charCodeAt(index) == 101)) {
				floating = true; index++;
				if (index < source.length && (source.charCodeAt(index) == 43 || source.charCodeAt(index) == 45)) index++;
				while (index < source.length && digit(source.charCodeAt(index))) index++;
			}
			if (!floating) {
				var number = Std.parseFloat(source.substr(start,index-start));
				if (number > 2147483647.0 || number < -2147483648.0) {
					output.addSub(source,copied,index-copied); output.add(".0"); copied = index;
				}
			}
		}
		if (copied == 0) return source;
		output.addSub(source,copied,source.length-copied);
		return output.toString();
	}
	static function digit(code:Int):Bool return code >= 48 && code <= 57;
	/** Integer values cannot use the reader's annotated-string map-key conversion. */
	public static function integer(raw:Dynamic):Void {
		if (!Std.isOfType(raw,Int) && !Std.isOfType(raw,Float)) throw "Scene wire integer must be numeric";
		var value:Float = cast raw;
		if (!Math.isFinite(value) || value < -2147483648.0 || value > 2147483647.0 || Math.floor(value) != value)
			throw "Scene wire integer is fractional or outside Int32";
	}
	static function fieldId(raw:Dynamic):Int {
		if (Std.isOfType(raw,String)) {
			var label:String = cast raw, colon = label.indexOf(":");
			var prefix = colon < 0 ? label : label.substr(0,colon);
			if (prefix.length == 0 || (prefix.length > 1 && prefix.charCodeAt(0) == 48)) throw "Scene wire field ID is not decimal";
			for (index in 0...prefix.length) if (!digit(prefix.charCodeAt(index))) throw "Scene wire field ID is not decimal";
			var value = Std.parseFloat(prefix);
			if (!Math.isFinite(value) || value < 1 || value > 2147483647.0) throw "Scene wire field ID is outside Int32";
			return Std.int(value);
		}
		integer(raw);
		var value:Int = cast raw;
		if (value <= 0) throw "Scene wire field ID must be positive";
		return value;
	}
	static function pairs(raw:Dynamic):Array<Array<Dynamic>> {
		if (!Std.isOfType(raw,Array)) throw "Scene wire map needs its current tuple representation";
		var result:Array<Array<Dynamic>> = [];
		for (item in (cast raw:Array<Dynamic>)) {
			if (!Std.isOfType(item,Array)) throw "Scene wire field needs a tuple";
			var pair:Array<Dynamic> = cast item;
			if (pair.length != 2) throw "Scene wire field needs an ID and value";
			var checked = fieldId(pair[0]);
			result.push(pair);
		}
		return result;
	}
	/** Field IDs come from the current generated codec rather than a second authored schema. */
	public static function integerFields(raw:Dynamic, canonical:Dynamic, names:Array<String>):Void {
		fieldMaps(raw,canonical);
		var ids:Array<Int> = [];
		for (pair in pairs(canonical)) {
			var label:String = cast pair[0], colon = label.indexOf(":");
			if (colon >= 0 && names.indexOf(label.substr(colon+1)) >= 0) ids.push(fieldId(pair[0]));
		}
		if (ids.length != names.length) throw "Scene wire integer fields are missing from the current codec";
		for (pair in pairs(raw)) if (ids.indexOf(fieldId(pair[0])) >= 0) integer(pair[1]);
	}
	/** Nested qualified frames and settings use the same strict annotated field IDs. */
	static function fieldMaps(raw:Dynamic, canonical:Dynamic):Void {
		if (!Std.isOfType(canonical,Array) || !Std.isOfType(raw,Array)) return;
		var expected:Array<Dynamic> = cast canonical, source:Array<Dynamic> = cast raw;
		if (expected.length == 0) return;
		var first:Dynamic = expected[0];
		if (Std.isOfType(first,Array)) {
			var tuple:Array<Dynamic> = cast first;
			if (tuple.length == 2 && Std.isOfType(tuple[0],String)) {
				var label:String = cast tuple[0];
				if (label.indexOf(":") > 0) {
					var fields = pairs(canonical);
					for (pair in pairs(raw)) for (field in fields) if (fieldId(pair[0]) == fieldId(field[0])) {
						fieldMaps(pair[1],field[1]); break;
					}
					return;
				}
			}
		}
		for (index in 0...source.length) if (index < expected.length) fieldMaps(source[index],expected[index]);
	}
	/** Current enum arguments are the CAD job, the numeric snug slot or the named final reference. */
	public static function enumArguments(raw:Dynamic, canonical:Dynamic):Array<Dynamic> {
		var source = pairs(raw), expected = pairs(canonical);
		if (source.length != 1 || expected.length != 1 || fieldId(source[0][0]) != fieldId(expected[0][0]))
			throw "Scene wire invocation needs one current variant";
		if (!Std.isOfType(source[0][1],Array) || !Std.isOfType(expected[0][1],Array)) throw "Scene wire invocation needs arguments";
		var actual:Array<Dynamic> = cast source[0][1], arguments:Array<Dynamic> = cast expected[0][1];
		if (actual.length != arguments.length) throw "Scene wire invocation has the wrong argument count";
		for (index in 0...arguments.length) if (Std.isOfType(arguments[index],Int)) integer(actual[index]);
		return actual;
	}
}
