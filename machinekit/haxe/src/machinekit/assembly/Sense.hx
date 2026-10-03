package machinekit.assembly;

/** Orientation of a transmission relative to its joints. */
@:wire enum Sense {
	@:id(1) Same;
	@:id(2) Opposite;
}

class SenseTools {
	public static function sign(sense:Sense):Float return switch sense {
		case Same: 1;
		case Opposite: -1;
	};

	public static function fromAlignment(alignment:Float):Sense {
		if (alignment == 1) return Same;
		if (alignment == -1) return Opposite;
		throw "Transmission alignment must be 1 or -1";
	}
}
