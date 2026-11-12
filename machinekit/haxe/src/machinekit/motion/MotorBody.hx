package machinekit.motion;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;

/**
 * A square-flanged motor housing as servo and spindle motors are built: a flange of side `face` and thickness
 * `flange` with bolt holes, a housing with chamfered corners, a rear (encoder) cap, and a connector boss on the
 * housing's +Y side. Motor CAD frame: mounting face at z = 0, body towards -Z over `length`; the shaft is its own
 * part. Everything lies inside the box `face` × `face` × `length` behind the face, the motor's envelope. With a `relief`
 * diameter the housing and cap are relieved round each bolt axis, as a servo's corners are behind its flange.
 */
class MotorBody {
  public static function record(
    b:DocumentBuilder,
    face:Float,
    length:Float,
    flange:Float,
    holeDiameter:Float,
    holes:Array<{x:Float, y:Float}>,
    ?relief:Float
  ):Feature {
    if (!(face > 0)
      || !(length > flange) || !(flange > 0)) throw "Motor body needs a positive face, flange and a longer body";
    var housing = face * 0.9, cap = face * 0.84, capLength = Math.min(length * 0.22, face * 0.5);
    function prism(side:Float, chamfer:Float, z0:Float, z1:Float):Feature
      return b.extrude(b.polygon(chamfered(side, chamfer), new Plane(new Vector(0, 0, z0), Vector.X(), Vector.Z())), z1 - z0);
    // Power and feedback connector on the housing's +Y face, inside the flange's footprint.
    var boss = face * 0.18, bossZ = -(length - capLength) + face * 0.06;
    var body = b.label(b.unionAll([
      b.label(prism(face, face * 0.08, -flange, 0), "Flange"),
      b.label(prism(housing, housing * 0.16, -(length - capLength), -flange), "Housing"),
      b.label(prism(cap, cap * 0.2, -length, -(length - capLength)), "Rear cap"),
      b.label(b.boxBetween(-boss * 0.7, boss * 0.7, housing / 2 - 0.01, face / 2, bossZ, bossZ + boss), "Connector")
    ]), "Motor body");
    var tools:Array<Feature> = [], labels:Array<String> = [];
    for (k => hole in holes) {
      tools.push(b.cylinderAlong(new Vector(hole.x, hole.y, 0), Vector.Z(), holeDiameter / 2, -flange - 0.1, 0.1));
      labels.push('bolt ${k + 1}');
    }
    // Corner reliefs behind the flange: a cylinder the width of the screw head and its driver round each bolt axis, over
    // the whole body, so the heads of the flange screws can be reached from the back (`ServoMotor.reliefDiameter`).
    if (relief != null && relief > 0) for (k => hole in holes) {
      tools.push(b.cylinderAlong(new Vector(hole.x, hole.y, 0), Vector.Z(), relief / 2, -length - 0.1, -flange));
      labels.push('screw relief ${k + 1}');
    }
    return tools.length == 0 ? body : b.label(b.cutAll(body, tools, labels), "Bolt holes and reliefs");
  }

  /** A square of side `side` centred on the axis with 45° chamfers of leg `chamfer` at its corners. */
  static function chamfered(side:Float, chamfer:Float):Array<Vector> {
    var h = side / 2, c = Math.min(chamfer, h * 0.9);
    return [
      new Vector(h - c, -h),
      new Vector(h, -h + c),
      new Vector(h, h - c),
      new Vector(h - c, h),
      new Vector(-h + c, h),
      new Vector(-h, h - c),
      new Vector(-h, -h + c),
      new Vector(-h + c, -h)
    ];
  }
}
