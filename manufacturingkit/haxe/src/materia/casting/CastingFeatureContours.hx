package materia.casting;

import materia.casting.CastingFeature;
import materia.casting.CastingProfile.CastingSegment;

/** One finished wall contour for source CAD, former drawings and CAM. Positive
 * cornerRadius fillets a line polygon; an authored arc profile already carries
 * its exact radii and must not also request an implicit second fillet.
 */
class CastingFeatureContours {
	public static function contour(feature:CastingFeature):Array<CastingSegment> {
		try {
			var radius=feature.cornerRadius;
			var raw=switch feature.geometry.solid {
				case Cut(solid): switch solid {
					case Box(width,depth,_): CastingProfiles.rectangle(-width/2,-depth/2,width,depth);
					case Cylinder(diameter,_):
						if (radius!=0) throw "Circular profile has no sharp wall corners";
						CastingProfiles.circle(0,0,diameter/2);
				};
				case Prism(profile,_): CastingProfile.CastingProfileCodec.snapshot(profile);
				case Insert(_): throw "Insert recess must use its source exterior";
			};
			if (radius==0) return raw;
			if (raw.filter(segment -> segment.center!=null).length>0)
				throw "Authored arc profile cannot also request implicit corner fillets";
			return CastingProfiles.roundedPolygon([for (segment in raw) segment.start],radius);
		} catch (error:Dynamic) {
			throw "Casting feature "+feature.id+": "+Std.string(error);
		}
	}
}
