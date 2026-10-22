package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.Solids;

/**
 * Drawings of annular purchased parts inside their envelope, the ring from `bore` to `outside` diameter and z from
 * 0 to `height` (`ComponentEnvelopes.annular`): bearings, ring encoders and frameless motors. Each is revolved
 * from its section, so chamfers, steps and recesses cost nothing, and names its bodies.
 */
class RingParts {
	/**
	 * A bearing: chamfered outer and inner rings, each a third of the radial section, with a shield recessed 0.4 mm
	 * into both faces between them. `holes` > 0 drills that many axial mounting holes through each ring, on circles
	 * through the rings' middles, as crossed-roller units are fixed.
	 */
	public static function bearing(outside:Float, bore:Float, height:Float, holes:Int = 0):Part {
		var outer = outside / 2, inner = bore / 2, wall = (outer - inner) / 3;
		var c = Math.min(height, wall) * 0.1, recess = Math.min(0.4, height * 0.05);
		return Solids.building([], owned -> {
			function keep(part:Part, tag:String):Part {
				var named = Solids.named(part, tag);
				owned.push(named);
				return named;
			}
			var body = Solids.union([
				keep(ring(outer - wall, outer, 0, height, c, false), "outer-ring"),
				keep(ring(inner, inner + wall, 0, height, c, true), "inner-ring"),
				keep(Part.revolve([{r: inner + wall, z: recess}, {r: outer - wall, z: recess}, {r: outer - wall, z: height - recess},
					{r: inner + wall, z: height - recess}]), "shield")
			]);
			if (holes <= 0) return body;
			var tools:Array<Part> = [];
			var diameter = Math.min(wall * 0.45, 9);
			for (circle in [outer - wall / 2, inner + wall / 2])
				for (index in 0...holes) {
					var angle = Math.PI / holes + 2 * Math.PI * index / holes;
					tools.push(keep(Part.cylinderSpan(diameter / 2, -0.1, height + 0.1, circle * Math.cos(angle), circle * Math.sin(angle)), "bolt-hole"));
				}
			return Solids.cut(body, tools);
		});
	}

	/**
	 * A ring encoder: a mounting hub on the bore, the full height, carrying a graduated scale band on the outside
	 * diameter, 60 % of the height, and `holes` mounting holes through the hub.
	 */
	public static function encoder(outside:Float, bore:Float, height:Float, holes:Int = 6):Part {
		var outer = outside / 2, inner = bore / 2, hub = inner + (outer - inner) * 0.45, band = height * 0.2;
		return Solids.building([], owned -> {
			var body = Solids.named(Part.revolve([{r: inner, z: 0}, {r: hub, z: 0}, {r: hub, z: band}, {r: outer, z: band},
				{r: outer, z: height - band}, {r: hub, z: height - band}, {r: hub, z: height}, {r: inner, z: height}]), "hub");
			owned.push(body);
			var circle = (inner + hub) / 2, diameter = Math.min(3.4, (hub - inner) * 0.5);
			var tools = [
				for (index in 0...holes)
					Solids.named(Part.cylinderSpan(diameter / 2, -0.1, height + 0.1, circle * Math.cos(2 * Math.PI * index / holes),
						circle * Math.sin(2 * Math.PI * index / holes)), "bolt-hole")
			];
			for (tool in tools) owned.push(tool);
			return Solids.cut(body, tools);
		});
	}

	/**
	 * A frameless motor: the stator, its outer 40 % of the section with recessed end turns on both faces, and the
	 * rotor, the inner 40 % with a stepped hub, across a 0.5 mm air gap. They are separate solids, as delivered.
	 */
	public static function framelessMotor(outside:Float, bore:Float, height:Float):Part {
		var outer = outside / 2, inner = bore / 2, section = outer - inner;
		var stator = outer - section * 0.4, rotor = inner + section * 0.4, gap = Math.min(0.5, (stator - rotor) / 2);
		var c = Math.min(height, section) * 0.04, turns = height * 0.12, step = height * 0.08;
		return Solids.building([], owned -> {
			function keep(part:Part, tag:String):Part {
				var named = Solids.named(part, tag);
				owned.push(named);
				return named;
			}
			// The stator iron runs to its inner face; the end turns stand off it, recessed from both faces.
			var statorBody = Solids.union([
				keep(ring(stator, outer, 0, height, c, false), "stator"),
				keep(Part.revolve([{r: rotor + gap, z: turns}, {r: stator, z: turns}, {r: stator, z: height - turns}, {r: rotor + gap, z: height - turns}]),
					"stator-teeth")
			]);
			var rotorBody = keep(Part.revolve([{r: inner, z: 0}, {r: inner + (rotor - inner) * 0.5, z: 0}, {r: inner + (rotor - inner) * 0.5, z: step},
				{r: rotor, z: step}, {r: rotor, z: height - step}, {r: inner, z: height - step}]), "rotor");
			return Solids.union([statorBody, rotorBody]);
		});
	}

	/** A ring from radius `r0` to `r1` and z from `z0` to `z1`, chamfered by `c` on its outer (or, `inside`, inner) edges. */
	static function ring(r0:Float, r1:Float, z0:Float, z1:Float, c:Float, inside:Bool):Part
		return Part.revolve(inside ? [{r: r0 + c, z: z0}, {r: r1, z: z0}, {r: r1, z: z1}, {r: r0 + c, z: z1}, {r: r0, z: z1 - c}, {r: r0, z: z0 + c}]
			: [{r: r0, z: z0}, {r: r1 - c, z: z0}, {r: r1, z: z0 + c}, {r: r1, z: z1 - c}, {r: r1 - c, z: z1}, {r: r0, z: z1}]);
}
