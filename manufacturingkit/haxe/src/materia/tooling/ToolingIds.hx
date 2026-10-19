package materia.tooling;

import haxe.crypto.Sha256;

/** IDs share one ASCII vocabulary across source and generated tooling records. */
class ToolingIds {
	public static function validate(value:String):Void {
		if (value == null || value.length == 0 || value.length > 128)
			throw "Invalid tooling identifier";
		for (i in 0...value.length)
			if ("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-".indexOf(value.charAt(i)) < 0)
				throw "Invalid tooling identifier";
	}

	/** Reserve suffix space without losing source identity when an ID is long. */
	public static function derive(prefix:String, source:String, suffix:String = ""):String {
		validate(source);
		var result = prefix + source + suffix;
		if (result.length > 128)
			result = result.substr(0, 63) + "-" + Sha256.encode(result);
		validate(result);
		return result;
	}
}
