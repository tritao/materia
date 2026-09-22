package cadkit.parametric;

import cadkit.parametric.ParametricError;

/** Process-independent opaque identifiers used by persisted document records. */
class PersistentId {
	private static var sequence:Int = 0;

	public static function create(prefix:String):String {
		sequence++;
		var time = Std.int((Sys.time() * 1000) % 0x3fffffff);
		return prefix + "-" + StringTools.hex(time, 8).toLowerCase()
			+ StringTools.hex(Std.random(0x3fffffff), 8).toLowerCase()
			+ StringTools.hex(sequence, 8).toLowerCase();
	}

	public static function validate(value:String, label:String):String {
		if (value == null || StringTools.trim(value) == "")
			throw new ParametricError(label + " must not be empty");
		return value;
	}
}
