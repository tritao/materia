package machinekit.motion;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;
import machinekit.motion.LinearRailProfile.LinearRailProfileSpec;
import machinekit.standard.SocketHeadCapScrew;

/**
 * Nominal profile-rail and carriage-block shapes from catalogue data (`LinearRail.NOMINAL`, the nominal carriage).
 * Rail frame: section centred on x = 0, y from -railHeight to the rail top at 0, running along +Z. Block frame:
 * rail-facing side at y = 0, top at blockHeight - railHeight, centred on z = 0. Rail bores come from the rail table,
 * carriage threads from the socket-head cap screw catalogue.
 */
class ProfileRailShapes {
	/**
	 * The rail: its rectangular section with chamfered top edges and a ball-track groove along each side (two on
	 * four-row HG-type profiles, one on MGN-type miniature ones), and counterbored mounting holes at `holes` (z).
	 */
	public static function rail(spec:LinearRailProfileSpec, length:Float, holes:Array<Float>):Part {
		var w = spec.railWidth, h = spec.railHeight, bore = spec.railBore;
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
			var tracks = StringTools.startsWith(spec.rail, "MGN") ? [-h * 0.3] : [-h * 0.22, -h * 0.62];
			for (side in [-1.0, 1.0])
				for (y in tracks)
					tool(Part.cylinderSpan(groove, -0.1, length + 0.1, side * w / 2, y));
			for (z in holes) {
				tool(Part.cylinderAlongY(bore.through / 2, -h - 1, 1, 0, z));
				tool(Part.cylinderAlongY(bore.counterbore / 2, -bore.depth, 1, 0, z));
			}
			return Solids.cut(body, tools);
		});
	}

	/**
	 * The carriage block above the rail top: a steel body with chamfered top edges, plastic end caps and seals at
	 * both ends, and four tapped mounting holes at the catalogue's spacings across and along the rail.
	 */
	public static function block(spec:LinearRailProfileSpec):Part {
		var w = spec.blockWidth, top = spec.blockHeight - spec.railHeight, l = spec.blockLength;
		var cap = l * 0.12, seal = l * 0.03, steel = l - 2 * (cap + seal);
		var thread = SocketHeadCapScrew.catalog().get(spec.blockMountThread).diameter;
		var depth = spec.blockMountThreadDepth == null ? Math.min(top * 0.6, thread * 2) : Math.min(top * 0.8, spec.blockMountThreadDepth);
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
						Solids.named(Part.cylinderAlongY(thread / 2, top - depth, top + 1, x * spec.blockHoleSpacingAcross / 2,
							z * spec.blockHoleSpacingAlong / 2), "mount")
			];
			for (hole in holes)
				owned.push(hole);
			return Solids.cut(body, holes);
		});
	}
}
