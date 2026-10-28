package materia.casting;

import materia.casting.CastingPrism;
import materia.assembly.AssemblyFrames as F;

/** Disjoint convex envelope pieces for the supported planar body family. */
class CastingEnvelope {
  public static function build(width:Float, depth:Float, height:Float, chamfer:Float = 0):Array<CastingPrism> {
    var x = width / 2, y = depth / 2, c = chamfer;
    var profile = c == 0 ? [{x : -x, y : -y}, {x : x, y : -y}, {x : x, y : y}, {x : -x, y : y}] : [
      {x : -x + c, y : -y},
      {x : x - c, y : -y},
      {x : x, y : -y + c},
      {x : x, y : y - c},
      {x : x - c, y : y},
      {x : -x + c, y : y},
      {x : -x, y : y - c},
      {x : -x, y : -y + c}
    ];
    return [{id: "body", profile: profile, height: height, pose: F.identity()}];
  }
}
