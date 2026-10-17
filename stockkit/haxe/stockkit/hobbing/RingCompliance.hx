package stockkit.hobbing;

/** Quasistatic in-plane n=2 thin-ring model. mm, N, Young's modulus N/mm².
    w=d*cos(2 theta), U=9*pi*E*I*d²/(2R³), I=width*wall³/12.
    Mandrel support is a separately stated generalized stiffness in parallel.
    Higher modes, local tooth bending, contact lift-off and dynamics are omitted. */
class RingCompliance {
  public final radius:Float;public final wall:Float;public final width:Float;
  public final youngs:Float;public final supportStiffness:Float;
  public final ringStiffness:Float;public final stiffness:Float;
  public final basis:String;
  public function new(radius:Float,wall:Float,width:Float,youngs:Float,supportStiffness:Float,basis:String) {
    for(v in [radius,wall,width,youngs])if(!Math.isFinite(v)||v<=0)throw "invalid ring geometry/material";
    if(wall>=radius/5||!Math.isFinite(supportStiffness)||supportStiffness<0||basis==null||basis.length==0)
      throw "thin ring needs explicit support stiffness and basis";
    this.radius=radius;this.wall=wall;this.width=width;this.youngs=youngs;
    this.supportStiffness=supportStiffness;this.basis=basis;
    ringStiffness=9*Math.PI*youngs*(width*wall*wall*wall/12)/(radius*radius*radius);
    stiffness=ringStiffness+supportStiffness;
  }
  public function deflection(force:Float):Float {
    if(!Math.isFinite(force)||force<0)throw "invalid radial ring load";return force/stiffness;
  }
  public function source(part:String,force:Float->Float):HobbingErrorSource {
    var source=HobbingErrorSource.wall(part,stiffness,force);
    return new HobbingErrorSource(source.id,part,'n=2 ring: R=$radius mm, wall=$wall mm, width=$width mm, E=$youngs N/mm²; '
      +'support $supportStiffness N/mm ($basis); local quasistatic recession only',source.apply);
  }
}
