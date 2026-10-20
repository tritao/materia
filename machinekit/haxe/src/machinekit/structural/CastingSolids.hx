package machinekit.structural;

import cadkit.modeling.Part;
import materia.casting.CastingSolid;
import materia.casting.CastingSolid.CastingSolidCodec;

/** Source and tooling realize the same neutral exterior in the insert frame.
 * These are solid exteriors; drilling is applied separately from source holes.
 */
class CastingSolids {
	public static function part(body:CastingSolid):Part {
		CastingSolidCodec.validate(body);
		return switch body {
			case Box(width, depth, height): Part.box(width, depth, height);
			case Cylinder(diameter, height): Part.cylinderSpan(diameter / 2, 0, height);
		};
	}

}
