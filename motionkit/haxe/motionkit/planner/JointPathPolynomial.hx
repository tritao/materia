package motionkit.planner;

import motionkit.math.QuinticPolynomial;

/** The normalized quintic Hermite polynomial consumed by native path.cpp. */
class JointPathPolynomial {
  public static function coefficients(path:JointPathSamples,span:Int):Array<Array<Float>>
    return QuinticPolynomial.hermite(path.q[span],path.q[span+1],path.qPrime[span],path.qPrime[span+1],
      path.qDoublePrime[span],path.qDoublePrimeBefore[span+1],path.s[span+1]-path.s[span]);
  public static function at(power:Array<Array<Float>>,u:Float):Array<Float> return QuinticPolynomial.at(power,u);
  public static function bernstein(power:Array<Array<Float>>):Array<Array<Float>> return QuinticPolynomial.bernstein(power);
  public static function split(control:Array<Array<Float>>):Array<Array<Array<Float>>> return QuinticPolynomial.split(control);
}
