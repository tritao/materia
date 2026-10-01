package cadkit;

import CadKit;

/** How a stored name matches one candidate (see `ElementNames.match`). */
class ElementMatch {
	/** `ElementNames.NONE`, `RELATIVE`, `WEAK` or `EXACT`. */
	public final grade:Int;
	/** For a relative: how much its split pieces agree with the reference's, in [0, 1] (1 when neither is split). */
	public final overlap:Float;

	public function new(grade:Int, overlap:Float) {
		this.grade = grade;
		this.overlap = overlap;
	}
}

/** Matching of topological names (plans/TOPOLOGICAL_NAMING.md); the names themselves are opaque text. */
class ElementNames {
	public static inline var NONE:Int = 0;
	/** The same element once split pieces, ordinals and input slots are set aside. */
	public static inline var RELATIVE:Int = 1;
	/** Equal, but the name is weak: only the geometry can confirm it. */
	public static inline var WEAK:Int = 2;
	public static inline var EXACT:Int = 3;

	/** How `reference` matches each of `candidates`. */
	public static function match(reference:String, candidates:Array<String>):Array<ElementMatch> {
		if (candidates.length == 0)
			return [];
		var bytes = CadKit.elementNameMatchBytesChecked(reference, candidates.join("\n"));
		return [for (index in 0...candidates.length) new ElementMatch(bytes.getInt32(index * 16), bytes.getDouble(index * 16 + 8))];
	}

	/** Whether `name` is strong: an exact match on it needs no geometry to confirm it. */
	public static function isStrong(name:String):Bool
		return match(name, [name])[0].grade == EXACT;

	/**
		`name` in words for people ("f3 › edge between top and right"), with tags kept before a "›" for the caller to
		replace (a feature's name for `f3`). Display only: never store or parse it.
	*/
	public static function label(name:String):String {
		var bytes = CadKit.elementNameLabelBytesChecked(name);
		return bytes.length == 0 ? "" : bytes.getString(0, bytes.length);
	}

	/** The tag that created the named element (`f7` in `f7:fillet(...)`), or "" when it has none. */
	public static function creatorTag(name:String):String {
		var bytes = CadKit.elementNameTagBytesChecked(name);
		return bytes.length == 0 ? "" : bytes.getString(0, bytes.length);
	}
}
