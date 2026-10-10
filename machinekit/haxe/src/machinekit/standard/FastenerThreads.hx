package machinekit.standard;

import cadkit.threads.ThreadSpec;
import machinekit.component.Dimension;

/** Nominal metric designation from the catalog dimensions, including custom fine pitches. */
class FastenerThreads {
	/** Coarse pitch comes from the shared fastener catalog, never from a size-string guess. */
	public static function coarse(diameter:Float):ThreadSpec {
		var name = "M" + Dimension.format(diameter);
		var spec = SocketHeadCapScrew.catalog().get(name);
		return metric(diameter, spec.pitch);
	}

	public static function metric(diameter:Float, pitch:Float):ThreadSpec {
		var name = "M" + Dimension.format(diameter);
		var coarse = SocketHeadCapScrew.catalog();
		var designation = coarse.exists(name) && coarse.get(name).pitch == pitch
			? name : name + "x" + Dimension.format(pitch);
		return new ThreadSpec(designation, diameter, pitch);
	}
}
