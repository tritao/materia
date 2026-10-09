package camkit.additive;

import cadkit.modeling.Axis;
import cadkit.modeling.Part;

/**
 * Parts to print for testing, built in code so tests need no files. Millimetres, standing on z = 0 and
 * centred on the origin; the caller closes them.
 */
class TestParts {
  /** A 20 mm cube with a 6 mm hole through it along z, off centre: for dimensional checks. */
  public static function calibrationCube():Part {
    var cube = Part.box(20, 20, 20);
    var hole = Part.cylinderSpan(3, -1, 21, 2, -1);
    var part = cube.subtract(hole);
    cube.close(); hole.close();
    return part;
  }

  /**
   * 20 x 20 x 12 mm: a block with a 45 degree chamfer round its top edges, a round boss on it, and a
   * hole through both with a counterbore in the boss. It has arcs, an island, and sloped and flat skins.
   */
  public static function testTower():Part {
    var block = Part.box(20, 20, 8);
    var top = block.edges().extreme(Axis.Z(), true);
    var chamfered = block.chamfer(top, 1.5);
    top.close();
    var boss = Part.cylinderSpan(5, 7.5, 12);
    var body = chamfered.combine(boss);
    var bore = Part.cylinderSpan(2, -1, 13);
    var counterbore = Part.cylinderSpan(4, 10, 13);
    var part = body.subtractAll([bore, counterbore]);
    for (piece in [block, chamfered, boss, body, bore, counterbore]) piece.close();
    return part;
  }

  /** 10 x 10 x 1.2 mm, six layers at 0.2 mm: the fast test. */
  public static function smallTab():Part
    return Part.box(10, 10, 1.2);
}
