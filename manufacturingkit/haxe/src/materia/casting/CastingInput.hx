package materia.casting;

/** An input is either supplied or explicitly unresolved. Design-rule limits
 * remain ordinary numbers; this type carries missing accuracy/evidence inputs.
 */
@:wire enum CastingInput {
	@:id(1) Known(value:Float);
	@:id(2) Unresolved(reason:String);
}

/** Text evidence uses the same states, with a concrete wire-safe value type. */
@:wire enum CastingTextInput {
	@:id(1) Known(value:String);
	@:id(2) Unresolved(reason:String);
}

class CastingInputs {
	public static function require(input:CastingInput):Float return switch input {
		case Known(value):value;
		case Unresolved(reason):throw "Unresolved casting input: "+reason;
	};

	public static function isKnown(input:CastingInput):Bool return switch input {
		case Known(_):true; case Unresolved(_):false;
	};

	public static function missing(inputs:Array<CastingInput>):Array<String> {
		var result:Array<String>=[];
		for (input in inputs) switch input {
			case Known(_):
			case Unresolved(reason):if (result.indexOf(reason)<0) result.push(reason);
		}
		return result;
	}

	public static function record(input:CastingInput):Dynamic return switch input {
		case Known(value):{kind:"known",value:value};
		case Unresolved(reason):{kind:"unresolved",reason:reason};
	};

	public static function requireText(input:CastingTextInput):String return switch input {
		case Known(value):value; case Unresolved(reason):throw "Unresolved casting input: "+reason;
	};
	public static function textIsKnown(input:CastingTextInput):Bool return switch input {
		case Known(_):true; case Unresolved(_):false;
	};
	public static function textRecord(input:CastingTextInput):Dynamic return switch input {
		case Known(value):{kind:"known",value:value}; case Unresolved(reason):{kind:"unresolved",reason:reason};
	};

	static function reason(raw:Dynamic):String {
		CastingFeatureCodec.fields(raw,["kind","reason"]);
		var value=CastingRecordCodec.string(raw,"reason");
		if (StringTools.trim(value).length==0)
			throw "Missing unresolved casting input reason";
		for (index in 0...value.length)
			if (value.charCodeAt(index)==0) throw "Invalid unresolved casting input reason";
		return value;
	}

	public static function number(raw:Dynamic):CastingInput {
		return switch CastingRecordCodec.string(raw,"kind") {
			case "known":
				CastingFeatureCodec.fields(raw,["kind","value"]);
				Known(CastingRecordCodec.number(raw,"value"));
			case "unresolved":Unresolved(reason(raw));
			default:throw "Unknown casting input state";
		};
	}

	public static function text(raw:Dynamic):CastingTextInput {
		return switch CastingRecordCodec.string(raw,"kind") {
			case "known":
				CastingFeatureCodec.fields(raw,["kind","value"]);
				var value=CastingRecordCodec.string(raw,"value");
				if (StringTools.trim(value).length==0) throw "Empty known casting input";
				for (index in 0...value.length)
					if (value.charCodeAt(index)==0) throw "Invalid known casting input text";
				CastingTextInput.Known(value);
			case "unresolved":CastingTextInput.Unresolved(reason(raw));
			default:throw "Unknown casting input state";
		};
	}
}
