package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Curve;
import cadkit.modeling.Vector;
import machinekit.component.Solids;
import materia.casting.InsertSpec.InsertBody;
import materia.casting.InsertSpec.InsertBodyCodec;

/** Realize insert-only retention geometry separately from design-cut primitives. */
class InsertBodies {
	public static function part(body:InsertBody):Part {
		InsertBodyCodec.validate(body);
		return switch body {
			case Primitive(solid): CastingSolids.part(solid);
			case CrimpedSocket(mouth, tail):
				switch mouth {
					case Cylinder(diameter, height): crimped(diameter, height + tail.length, tail);
					default: throw "Crimped socket mouth must be cylindrical";
				}
		};
	}

	static function crimped(diameter:Float, height:Float, tail:materia.casting.InsertSpec.InsertCrimpedTail):Part {
		var radius = diameter / 2, wide = tail.width / 2, thin = tail.thickness / 2;
		var span = tail.length;
		var sections:Array<Curve> = [];
		var formed:Part;
		try {
			for (s in [[0.0, wide, thin, 0.0], [span * 0.25, wide, thin, tail.wave],
				[span * 0.60, wide * 0.92, thin, -tail.wave],
				[span - tail.blend, radius, radius, 0.0], [span + tail.blend, radius, radius, 0.0]]) {
				sections.push(Curve.polyline([for (i in 0...24) {
					var angle = 2 * Math.PI * i / 24;
					new Vector(s[1] * Math.cos(angle), s[2] * Math.sin(angle) + s[3], s[0]);
				}], true));
			}
			formed = Part.loft(sections, true);
		} catch (error:Dynamic) {
			for (section in sections) section.close();
			throw error;
		}
		for (section in sections) section.close();
		return Solids.building([formed], owned -> {
			owned.push(Part.cylinderSpan(radius, span, height));
			return Solids.union(owned);
		});
	}
}
