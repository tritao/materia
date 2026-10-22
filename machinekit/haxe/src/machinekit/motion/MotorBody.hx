package machinekit.motion;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;

/**
 * A square-flanged motor housing as servo and spindle motors are built: a flange of side `face` and thickness
 * `flange` with bolt holes, a housing with chamfered corners, a rear (encoder) cap, and a connector boss on the
 * housing's +Y side. Motor CAD frame: mounting face at z = 0, body towards -Z over `length`; the shaft is its own
 * part. Everything lies inside the box `face` × `face` × `length` behind the face, the motor's envelope.
 */
class MotorBody {
	public static function part(face:Float, length:Float, flange:Float, holeDiameter:Float, holes:Array<{x:Float, y:Float}>):Part {
		if (!(face > 0) || !(length > flange) || !(flange > 0))
			throw "Motor body needs a positive face, flange and a longer body";
		var housing = face * 0.9, cap = face * 0.84, capLength = Math.min(length * 0.22, face * 0.5);
		return Solids.building([], owned -> {
			var pieces:Array<Part> = [];
			function keep(part:Part):Part {
				owned.push(part);
				pieces.push(part);
				return part;
			}
			keep(Solids.named(Part.prism(chamfered(face, face * 0.08), -flange, 0), "flange"));
			keep(Solids.named(Part.prism(chamfered(housing, housing * 0.16), -(length - capLength), -flange), "housing"));
			keep(Solids.named(Part.prism(chamfered(cap, cap * 0.2), -length, -(length - capLength)), "rear-cap"));
			// Power and feedback connector on the housing's +Y face, inside the flange's footprint.
			var boss = face * 0.18;
			keep(Solids.named(Part.box(boss * 1.4, face / 2 - housing / 2 + 0.01, boss, Center, Min, Min)
				.translated(new Vector(0, housing / 2 - 0.01, -(length - capLength) + face * 0.06)), "connector"));
			var body = Solids.union(pieces);
			var tools = [
				for (hole in holes)
					Solids.named(Part.cylinderSpan(holeDiameter / 2, -flange - 0.1, 0.1, hole.x, hole.y), "bolt-hole")
			];
			for (tool in tools)
				owned.push(tool);
			return tools.length == 0 ? body : Solids.cut(body, tools);
		});
	}

	/** A square of side `side` centred on the axis with 45° chamfers of leg `chamfer` at its corners. */
	static function chamfered(side:Float, chamfer:Float):Array<Vector> {
		var h = side / 2, c = Math.min(chamfer, h * 0.9);
		return [
			new Vector(h - c, -h), new Vector(h, -h + c), new Vector(h, h - c), new Vector(h - c, h),
			new Vector(-h + c, h), new Vector(-h, h - c), new Vector(-h, -h + c), new Vector(-h + c, -h)
		];
	}
}
