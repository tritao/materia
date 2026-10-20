package machinekit.structural;

import materia.casting.CastingFeature;
import materia.casting.CastingPatterns;
import materia.casting.CastingPrism;
import materia.casting.CastingCavities;
import materia.casting.BoxAlgebra;
import materia.casting.Box3;

/** Shared source patterns; constants and ownership belong to the caller. */
class CastingFeaturePatterns {
	public static function box(body:Array<CastingPrism>, id:String, kind:CastingFeatureKind, width:Float, depth:Float, height:Float, x:Float = 0, y:Float = 0,
			z:Float = 0, ?owner:String, ?adjustable:Array<String>):CastingFeature {
		var feature = CastingPatterns.box(id, kind, [], width, depth, height, x, y, z, owner, adjustable);
		feature.openings = CastingCavities.reachedOpenings(body, feature.geometry);
		return feature;
	}

	public static function underside(body:Array<CastingPrism>, id:String, width:Float, depth:Float, height:Float, x:Float, y:Float):CastingFeature
		return box(body, id, Lightening, width, depth, height, x, y, 0, null, ["width", "depth", "height"]);

	public static function foot(body:Array<CastingPrism>, id:String, x:Float, y:Float, edge:Float, alongX:Bool, accessWidth:Float, accessDepth:Float,
			floor:Float, roof:Float, stemSize:Float, stemTop:Float, ?plate:{
			owner:String,
			width:Float,
			depth:Float,
			thickness:Float
		}):Array<CastingFeature> {
		var centre = alongX ? x : y,
			low = Math.min(centre - accessWidth / 2, edge),
			high = Math.max(centre + accessWidth / 2, edge);
		var out = [
			box(body, id
				+ "-access", ServiceOpening, alongX ? high - low : accessDepth, alongX ? accessDepth : high - low,
				roof
				- floor
				- (plate == null ? 0 : plate.thickness), alongX ? (low + high) / 2 : x, alongX ? y : (low + high) / 2, floor),
			box(body, id + "-stem", Clearance, stemSize, stemSize, stemTop, x, y, 0)
		];
		if (plate != null)
			out.push(box(body, id + "-plate-register", InterfaceRecess, plate.width, plate.depth, plate.thickness, x, y, roof - plate.thickness, plate.owner));
		out[1].openings.push(CastingPatterns.feature(out[0].id));
		return out;
		}

	public static function retain(features:Array<CastingFeature>, supports:Array<Box3>):Array<CastingFeature>
		return BoxAlgebra.retain(features, supports);
}
