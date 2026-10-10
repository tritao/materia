package machinekit.standard;

/** Mechanical class of a steel or stainless fastener. */
@:wire
enum PropertyClass {
	@:id(1) Steel88;
	@:id(2) Steel109;
	@:id(3) Steel129;
	@:id(4) A270;
}

/** Recipe tokens and physical material derived from the mechanical class. */
class PropertyClasses {
	public static function token(value:PropertyClass):String return switch value {
		case Steel88: "8.8";
		case Steel109: "10.9";
		case Steel129: "12.9";
		case A270: "A2-70";
	};

	public static function parse(value:String):PropertyClass return switch value {
		case "8.8": Steel88;
		case "10.9": Steel109;
		case "12.9": Steel129;
		case "A2-70": A270;
		default: throw 'Unknown fastener property class "$value"';
	};

	public static function tokens():Array<String> return ["8.8", "10.9", "12.9", "A2-70"];

	public static function material(value:PropertyClass):String return switch value {
		case Steel88: "steel 8.8";
		case Steel109: "steel 10.9";
		case Steel129: "steel 12.9";
		case A270: "stainless A2-70";
	};
}
