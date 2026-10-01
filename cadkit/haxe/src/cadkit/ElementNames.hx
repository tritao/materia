package cadkit;

import CadKit;

/** Matching of topological names (plans/TOPOLOGICAL_NAMING.md); the names themselves are opaque text. */
class ElementNames {
	/** A unique, strong name equal to the reference. */
	public static inline var EXACT:Float = 3;
	/** Equal, but weak: only the geometry can confirm it. */
	public static inline var WEAK:Float = 2;
	/** The same element once split pieces, ordinals and input slots are set aside; [1, 2), higher when the pieces agree more. */
	public static inline var RELATIVE:Float = 1;

	/** How well `reference` matches each of `candidates`: `EXACT`, `WEAK`, a relative score in [1, 2), or 0. */
	public static function match(reference:String, candidates:Array<String>):Array<Float> {
		if (candidates.length == 0)
			return [];
		var bytes = CadKit.elementNameMatchBytesChecked(reference, candidates.join("\n"));
		return [for (index in 0...candidates.length) bytes.getDouble(index * 8)];
	}
}
