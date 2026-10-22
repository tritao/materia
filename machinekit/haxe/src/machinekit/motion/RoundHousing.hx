package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.Solids;

/** One coaxial step of a round housing: a cylinder of `diameter` from z0 to z1. */
typedef HousingStep = {var z0:Float; var z1:Float; var diameter:Float;}

/**
 * A round housing along +Z as reducers, gearboxes and encoders are built: coaxial steps (flanges, body, adapters),
 * an optional through bore, and bolt holes through one flange on a circle. Every step lies inside the cylinder of
 * the largest diameter over the steps' span, the part's envelope.
 */
class RoundHousing {
	/**
	 * `holes` bolt holes of `holeDiameter` on a circle of `holeCircle` diameter, drilled through the step at index
	 * `holeStep`; none when `holes` is zero. `bore` zero leaves the housing solid.
	 */
	public static function part(steps:Array<HousingStep>, bore:Float, holeStep:Int = -1, holes:Int = 0, holeCircle:Float = 0,
			holeDiameter:Float = 0):Part {
		if (steps.length == 0)
			throw "A round housing needs at least one step";
		var low = Math.POSITIVE_INFINITY, high = Math.NEGATIVE_INFINITY;
		for (step in steps) {
			if (!(step.diameter > 0) || !(step.z1 > step.z0))
				throw "A housing step needs a positive diameter and length";
			low = Math.min(low, step.z0);
			high = Math.max(high, step.z1);
		}
		return Solids.building([], owned -> {
			var pieces:Array<Part> = [];
			for (index in 0...steps.length) {
				var step = steps[index];
				var piece = Solids.named(Part.cylinderSpan(step.diameter / 2, step.z0, step.z1), index == holeStep ? "flange" : "housing");
				owned.push(piece);
				pieces.push(piece);
			}
			var body = pieces.length == 1 ? pieces[0] : Solids.union(pieces);
			var tools:Array<Part> = [];
			if (bore > 0)
				tools.push(Solids.named(Part.cylinderSpan(bore / 2, low - 1, high + 1), "bore"));
			if (holes > 0 && holeStep >= 0) {
				var flange = steps[holeStep];
				for (index in 0...holes) {
					var angle = Math.PI / 4 + 2 * Math.PI * index / holes;
					tools.push(Solids.named(Part.cylinderSpan(holeDiameter / 2, flange.z0 - 0.1, flange.z1 + 0.1, holeCircle / 2 * Math.cos(angle),
						holeCircle / 2 * Math.sin(angle)), "bolt-hole"));
				}
			}
			for (tool in tools)
				owned.push(tool);
			return Solids.cut(body, tools);
		});
	}
}
