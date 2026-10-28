package machinekit.structural;

import materia.casting.CastingFeature;
import materia.casting.CastingPatterns;

/** Shared source patterns; constants and ownership belong to the caller. */
class CastingFeaturePatterns {
  public static function box(
    id:String,
    kind:CastingFeatureKind,
    openings:Array<CastingOpening>,
    width:Float,
    depth:Float,
    height:Float,
    x:Float = 0,
    y:Float = 0,
    z:Float = 0,
    ?owner:String,
    ?adjustable:Array<CastingDimension>
  ):CastingFeature {
    return CastingPatterns.box(id, kind, openings, width, depth, height, x, y, z, owner, adjustable);
  }

  public static function underside(id:String,
    width:Float, depth:Float, height:Float, x:Float, y:Float):CastingFeature return box(
      id,
      Lightening,
      [CastingPatterns.face("body.z-min")],
      width,
      depth,
      height,
      x,
      y,
      0,
      null,
      [CastingDimension.Width, CastingDimension.Depth, CastingDimension.Height]
    );

  /** Compression-foot pocket: the removable plate bears directly on the solid
	 * ceiling. Only the lower stud passage is cast; no plate register or upper bore. */
  public static function foot(
    ids: {access: String, stem: String},
    x:Float,
    y:Float,
    edge:Float,
    alongX:Bool,
    accessWidth:Float,
    accessDepth:Float,
    floor:Float,
    roof:Float,
    stemSize:Float,
    accessOpenings:Array<CastingOpening>,
    stemOpenings:Array<CastingOpening>
  ):Array<CastingFeature> {
    var centre = alongX ? x : y;
    var low = Math.min(centre - accessWidth / 2, edge), high = Math.max(centre + accessWidth / 2, edge);
    return [
      box(
        ids.access,
        ServiceOpening,
        accessOpenings,
        alongX ? high - low : accessDepth,
        alongX ? accessDepth : high - low,
        roof - floor,
        alongX ? (low + high) / 2 : x,
        alongX ? y : (low + high) / 2,
        floor
      ),
      box(
        ids.stem,
        Clearance,
        stemOpenings.concat([CastingPatterns.feature(ids.access)]),
        stemSize,
        stemSize,
        floor,
        x,
        y,
        0
      )
    ];
  }
}
