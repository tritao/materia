package motionkit.robot;
import robotkit.spatial.Quat;
import motionkit.robot.RedundancySpline.SplineSample;

/** Analytic second-order quaternion composition for a moving orientation
 * centre, a spline swing vector, and periodic roll. Angular rates are spatial
 * (expressed in the same reference frame as the centre quaternion). */
class OrientationDifferential {
  public static function refine(centre:Quat,omega:Array<Float>,alpha:Array<Float>,
      x:SplineSample,y:SplineSample,roll:SplineSample):OrientationMotion {
    if(centre==null || omega==null || alpha==null || omega.length!=3 || alpha.length!=3 || x==null || y==null || roll==null)
      throw "Orientation derivatives require a centre, angular rates and spline states";
    for(v in omega.concat(alpha))if(!Math.isFinite(v))throw "Angular rates must be finite";
    var q=[centre.x,centre.y,centre.z,centre.w],w=[omega[0],omega[1],omega[2],0.0],a=[alpha[0],alpha[1],alpha[2],0.0];
    var dq=product(w,q),ddq=product(a,q),wwq=product(w,dq);
    var base=[for(i in 0...4)new ScalarJet(q[i],0.5*dq[i],0.5*ddq[i]+0.25*wwq[i])];
    var sx=ScalarJet.of(x),sy=ScalarJet.of(y),spin=ScalarJet.of(roll).scale(0.5);
    var u=sx.mul(sx).add(sy.mul(sy)),factor:ScalarJet,cosine:ScalarJet;
    if(u.value<1e-4){
      // Entire power series in squared tilt avoids a polar singularity at zero.
      factor=new ScalarJet(0.5).add(u.scale(-1.0/48)).add(u.mul(u).scale(1.0/3840)).add(u.mul(u).mul(u).scale(-1.0/645120));
      cosine=new ScalarJet(1).add(u.scale(-1.0/8)).add(u.mul(u).scale(1.0/384)).add(u.mul(u).mul(u).scale(-1.0/46080));
    }else{var radius=u.sqrt();factor=radius.scale(0.5).sin().mul(radius.inverse());cosine=radius.scale(0.5).cos();}
    var swing=[sy.mul(factor).scale(-1),sx.mul(factor),new ScalarJet(0),cosine];
    var twist=[new ScalarJet(0),new ScalarJet(0),spin.sin(),spin.cos()];
    var result=compose(compose(base,swing),twist);
    var value=[for(c in result)c.value],first=[for(c in result)c.first],second=[for(c in result)c.second];
    var conjugate=[-value[0],-value[1],-value[2],value[3]];
    var velocity=product(first,conjugate),acceleration=product(second,conjugate);
    for(v in value.concat(first).concat(second))if(!Math.isFinite(v))throw "Orientation derivatives are unrepresentable";
    return new OrientationMotion(new Quat(value[0],value[1],value[2],value[3]),
      [for(i in 0...3)2*velocity[i]],[for(i in 0...3)2*acceleration[i]]);
  }
  static function product(a:Array<Float>,b:Array<Float>):Array<Float> return [
    a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
    a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
    a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
    a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]];
  static function compose(a:Array<ScalarJet>,b:Array<ScalarJet>):Array<ScalarJet> return [
    a[3].mul(b[0]).add(a[0].mul(b[3])).add(a[1].mul(b[2])).add(a[2].mul(b[1]).scale(-1)),
    a[3].mul(b[1]).add(a[0].mul(b[2]).scale(-1)).add(a[1].mul(b[3])).add(a[2].mul(b[0])),
    a[3].mul(b[2]).add(a[0].mul(b[1])).add(a[1].mul(b[0]).scale(-1)).add(a[2].mul(b[3])),
    a[3].mul(b[3]).add(a[0].mul(b[0]).scale(-1)).add(a[1].mul(b[1]).scale(-1)).add(a[2].mul(b[2]).scale(-1))];
}
class OrientationMotion {
  public final rotation:Quat;
  public final velocity:Array<Float>;
  public final acceleration:Array<Float>;
  public function new(rotation:Quat,velocity:Array<Float>,acceleration:Array<Float>){this.rotation=rotation;this.velocity=velocity;this.acceleration=acceleration;}
}
private class ScalarJet {
  public final value:Float;public final first:Float;public final second:Float;
  public function new(value:Float,first:Float=0,second:Float=0){this.value=value;this.first=first;this.second=second;}
  public static function of(s:SplineSample):ScalarJet {
    if(!Math.isFinite(s.value) || !Math.isFinite(s.first) || !Math.isFinite(s.second))throw "Spline states must be finite";
    return new ScalarJet(s.value,s.first,s.second);
  }
  public function add(b:ScalarJet):ScalarJet return new ScalarJet(value+b.value,first+b.first,second+b.second);
  public function scale(b:Float):ScalarJet return new ScalarJet(value*b,first*b,second*b);
  public function mul(b:ScalarJet):ScalarJet return new ScalarJet(value*b.value,first*b.value+value*b.first,second*b.value+2*first*b.first+value*b.second);
  public function inverse():ScalarJet return new ScalarJet(1/value,-first/(value*value),2*first*first/(value*value*value)-second/(value*value));
  public function sqrt():ScalarJet {var r=Math.sqrt(value);return new ScalarJet(r,first/(2*r),second/(2*r)-first*first/(4*r*r*r));}
  public function sin():ScalarJet return new ScalarJet(Math.sin(value),Math.cos(value)*first,Math.cos(value)*second-Math.sin(value)*first*first);
  public function cos():ScalarJet return new ScalarJet(Math.cos(value),-Math.sin(value)*first,-Math.sin(value)*second-Math.cos(value)*first*first);
}
