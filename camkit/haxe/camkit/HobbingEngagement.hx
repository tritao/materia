package camkit;

/** Exact continuous cylindrical-envelope bounds for two skew cutter/blank cylinders.
    Lengths in mm. Cutter tooth/gash gaps can only reduce this envelope.
    Requires the finite hob to cover the returned axial half-span at the selected shift. */
class HobbingEngagement {
  public final overrun:Float;
  public final axialHalfSpan:Float;
  public function new(blankRadius:Float, hobRadius:Float, centreDistance:Float, tilt:Float) {
    for (v in [blankRadius,hobRadius,centreDistance,tilt]) if (!Math.isFinite(v)) throw "Engagement geometry must be finite";
    if (blankRadius <= 0 || hobRadius <= 0 || centreDistance <= 0 || Math.abs(tilt) >= Math.PI/2)
      throw "Invalid hobbing engagement cylinders";
    var lo = Math.max(-hobRadius,centreDistance-blankRadius), hi = Math.min(hobRadius,centreDistance+blankRadius);
    if (lo >= hi) throw "Hob does not engage the blank";
    var cosine = Math.cos(tilt), tangent = Math.abs(Math.tan(tilt));
    // At radial penetration w, |y| <= sqrt(r²-(D-w)²) and |v| <= sqrt(R²-w²).
    // Global Z = -y*tan(A)-v/cos(A), global axial S = -y/cos(A)-v*tan(A).
    overrun = maximum(lo,hi,w -> Math.sqrt(Math.max(0,blankRadius*blankRadius-(centreDistance-w)*(centreDistance-w)))*tangent
      +Math.sqrt(Math.max(0,hobRadius*hobRadius-w*w))/cosine);
    axialHalfSpan = maximum(lo,hi,w -> Math.sqrt(Math.max(0,blankRadius*blankRadius-(centreDistance-w)*(centreDistance-w)))/cosine
      +Math.sqrt(Math.max(0,hobRadius*hobRadius-w*w))*tangent);
  }
  static function maximum(lo:Float,hi:Float,f:Float->Float):Float {
    // Both square-root terms are concave on this common domain; ternary search has one maximum.
    var a=lo,b=hi;
    for (_ in 0...100) { var u=a+(b-a)/3,v=b-(b-a)/3; if(f(u)<f(v))a=u;else b=v; }
    return Math.max(Math.max(f(lo),f(hi)),f((a+b)/2))+1e-9;
  }
}
