package motionkit.path;

import motionkit.math.QuinticPolynomial;

/** Endpoint interpolation error plus a continuous remainder supplied by the primitive's exact geometry. */
class HermitePositionBound {
  public static function of(primitive:PosePrimitive,control:Array<Array<Float>>,from:Float,to:Float,
      remainder:Float):Float {
    var a=primitive.waypointAt(from).pose,b=primitive.waypointAt(to).pose;
    var da=primitive.derivativesAt(from),db=primitive.derivativesAt(to);
    var reference=QuinticPolynomial.bernstein(QuinticPolynomial.hermite(a.positionArray(),b.positionArray(),
      da.linear,db.linear,da.linearSecond,db.linearSecond,to-from));
    return QuinticPolynomial.differenceBound(control,reference)+remainder;
  }
  /** Quintic Hermite remainder: max |f^(6)| * span^6/(6!*64). */
  public static function circle(radius:Float,angle:Float):Float
    return Math.sqrt(3.0)*radius*Math.pow(Math.abs(angle),6)/46080.0;
}
