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

  public static function foot(
    ids: {access: String, stem: String, register: Null<String>},
    x:Float,
    y:Float,
    edge:Float,
    alongX:Bool,
    accessWidth:Float,
    accessDepth:Float,
    floor:Float,
    roof:Float,
    stemSize:Float,
    stemTop:Float,
    accessOpenings:Array<CastingOpening>,
    stemOpenings:Array<CastingOpening>,
    ?plate: {owner: String, openings: Array<CastingOpening>, width: Float, depth: Float, thickness: Float}
  ):Array<CastingFeature> {
    var centre = alongX ? x : y, low = Math.min(centre - accessWidth / 2, edge), high = Math.max(
      centre + accessWidth / 2,
      edge
    );
    // The bore enters the dry access pocket at its floor. If it continues
    // above the register, that is a separate, directly authored upper bore.
    var lowerTop = Math.min(stemTop, floor);
    var upperFloor = roof;
    var out = [
      box(
        ids.access,
        ServiceOpening,
        accessOpenings,
        alongX ? high - low : accessDepth,
        alongX ? accessDepth : high - low,
        roof - floor - (plate == null ? 0 : plate.thickness),
        alongX ? (low + high) / 2 : x,
        alongX ? y : (low + high) / 2,
        floor
      ),
      box(
        ids.stem,
        Clearance,
        stemOpenings.filter(opening -> opening.kind == BodyFace),
        stemSize,
        stemSize,
        lowerTop,
        x,
        y,
        0
      )
    ];
    if (stemTop > upperFloor) {
      var upperOpenings = stemOpenings.filter(opening -> opening.kind == Feature);
      upperOpenings.push(CastingPatterns.feature(plate == null ? ids.access : ids.register));
      out.push(box(
        ids.stem + "-upper",
        Clearance,
        upperOpenings,
        stemSize,
        stemSize,
        stemTop - upperFloor,
        x,
        y,
        upperFloor
      ));
    }
    if (plate != null) {
      if (ids.register == null) throw "Foot plate requires an explicit register feature ID";
      out.push(box(
        ids.register,
        InterfaceRecess,
        plate.openings,
        plate.width,
        plate.depth,
        plate.thickness,
        x,
        y,
        roof - plate.thickness,
        plate.owner
      ));
    }
    out[1].openings = out[1].openings.copy();
    out[1].openings.push(CastingPatterns.feature(out[0].id));
    return out;
  }

}
