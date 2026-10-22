package machinekit.motion;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;
import machinekit.motion.LinearRailProfile.LinearRailProfileSpec;
import machinekit.standard.SocketHeadCapScrew;

/**
 * Nominal profile-rail and carriage-block shapes from catalogue data, for profiles without a measured section
 * (HGR15 has one, `Hgr15RailGeometry`). Rail frame: section centred on x = 0, y from -railHeight to the rail top
 * at 0, running along +Z. Block frame: rail-facing side at y = 0, top at blockHeight - railHeight, centred on
 * z = 0. Screw clearances and counterbores come from the socket-head cap screw catalogue.
 */
class ProfileRailShapes {
	/**
	 * The rail: its rectangular section with chamfered top edges and a ball-track groove along each side (two on
	 * four-row HG-type profiles, one on MGN-type miniature ones), and counterbored mounting holes at `holes` (z).
	 */
	public static function rail(spec:LinearRailProfileSpec, length:Float, holes:Array<Float>):Part {
		var w = spec.railWidth, h = spec.railHeight;
		var screw = SocketHeadCapScrew.catalog().get(screwSize(spec.railMountScrew));
		return Solids.building([], owned -> {
			var body = Part.prism([new Vector(-w / 2, -h), new Vector(w / 2, -h), new Vector(w / 2, 0), new Vector(-w / 2, 0)], 0, length);
			owned.push(body);
			var tools:Array<Part> = [];
			function tool(part:Part):Void {
				owned.push(part);
				tools.push(part);
			}
			var chamfer = Math.min(w, h) * 0.06;
			for (side in [-1.0, 1.0])
				tool(Part.prism([new Vector(side * w / 2, 0), new Vector(side * (w / 2 - chamfer), 0.1), new Vector(side * (w / 2 + 0.1), -chamfer)],
					-0.1, length + 0.1));
			var groove = h * 0.11;
			var tracks = spec.family == "HIWIN" && StringTools.startsWith(spec.designation, "MGN") ? [-h * 0.3] : [-h * 0.22, -h * 0.62];
			for (side in [-1.0, 1.0])
				for (y in tracks)
					tool(Part.cylinderSpan(groove, -0.1, length + 0.1, side * w / 2, y));
			for (z in holes) {
				tool(Part.cylinderAlongY(screw.clearanceMedium / 2, -h - 1, 1, 0, z));
				tool(Part.cylinderAlongY(screw.counterboreDiameter / 2, -Math.min(screw.counterboreDepth, h * 0.6), 1, 0, z));
			}
			return Solids.cut(body, tools);
		});
	}

	/**
	 * The carriage block above the rail top: a steel body with chamfered top edges, plastic end caps and seals at
	 * both ends, and four tapped mounting holes at the catalogue's B (across) and C (along) pitches.
	 */
	public static function block(spec:LinearRailProfileSpec):Part {
		var w = spec.blockWidth, top = spec.blockHeight - spec.railHeight, l = spec.blockLength;
		var cap = l * 0.12, seal = l * 0.03, steel = l - 2 * (cap + seal);
		var thread = SocketHeadCapScrew.catalog().get(screwSize(spec.blockMountScrew)).diameter;
		return Solids.building([], owned -> {
			var pieces:Array<Part> = [];
			function keep(part:Part):Void {
				owned.push(part);
				pieces.push(part);
			}
			var c = Math.min(w, top) * 0.08;
			keep(Solids.named(Part.prism([
				new Vector(-w / 2, 0), new Vector(w / 2, 0), new Vector(w / 2, top - c), new Vector(w / 2 - c, top),
				new Vector(-w / 2 + c, top), new Vector(-w / 2, top - c)
			], -steel / 2, steel / 2), "body"));
			for (end in [-1.0, 1.0]) {
				var capWidth = w * 0.94, capTop = top * 0.9;
				var capStart = end > 0 ? steel / 2 : -steel / 2 - cap;
				keep(Solids.named(Part.box(capWidth, capTop, cap, Center, Min, Min).translated(new Vector(0, 0, capStart)), "end-cap"));
				var sealStart = end > 0 ? steel / 2 + cap : -steel / 2 - cap - seal;
				keep(Solids.named(Part.box(capWidth * 0.97, capTop * 0.95, seal, Center, Min, Min).translated(new Vector(0, 0, sealStart)), "seal"));
			}
			var body = Solids.union(pieces);
			var holes = [
				for (x in [-1.0, 1.0])
					for (z in [-1.0, 1.0])
						Solids.named(Part.cylinderAlongY(thread / 2, top - Math.min(top * 0.6, thread * 2), top + 1, x * spec.blockHolePitchB / 2,
							z * spec.blockHolePitchC / 2), "mount")
			];
			for (hole in holes)
				owned.push(hole);
			return Solids.cut(body, holes);
		});
	}

	/** "M3x8" -> "M3". */
	static function screwSize(designation:String):String
		return designation.split("x")[0];
}
