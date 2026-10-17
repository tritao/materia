package stockkit.hobbing;
import StockKitNative;

/** mm/radians. X is the work/hob centre distance, Y shift along the hob axis,
    Z the hob head height. A is signed tilt from -Y towards +Z. B and C are
    right-hand, unwrapped spindle and table angles. */
class HobbingAxes {
  public final x:Float; public final y:Float; public final z:Float;
  public final a:Float; public final b:Float; public final c:Float;
  public final offset:Null<HobbingPoseOffset>;
  public function new(x:Float,y:Float,z:Float,a:Float,b:Float,c:Float,?offset:HobbingPoseOffset) {
    for(v in [x,y,z,a,b,c]) if(!Math.isFinite(v)) throw "hobbing axes must be finite";
    this.offset=offset;
    this.x=x;this.y=y;this.z=z;this.a=a;this.b=b;this.c=c;
  }
  /** Hob +X faces the work centre; hob +Z points along the tilted -Y axis.
      The final inverse C rotation expresses the cut in the workpiece frame. */
  public function pose():sk_rake_pose {
    var ca=Math.cos(a),sa=Math.sin(a),cb=Math.cos(b),sb=Math.sin(b),cc=Math.cos(c),sc=Math.sin(c);
    var r=[-cb,-sa*sb,-ca*sb,sb,-sa*cb,-ca*cb,0.0,-ca,sa]; // columns
    var p=new sk_rake_pose();p.set_struct_size(sk_rake_pose.size());
    for(j in 0...3) {
      p.set_rotation(j,cc*r[3*j]+sc*r[3*j+1]);
      p.set_rotation(3+j,-sc*r[3*j]+cc*r[3*j+1]);
      p.set_rotation(6+j,r[3*j+2]);
    }
    p.set_translation(0,cc*x-sc*y*ca);
    p.set_translation(1,-sc*x-cc*y*ca);
    p.set_translation(2,z+y*sa);
    if(offset!=null){
      var tool=[offset.toolX,offset.toolY,offset.toolZ],work=[offset.workX,offset.workY,offset.workZ];
      for(i in 0...3){var delta=-work[i];for(j in 0...3)delta+=p.get_rotation(3*i+j)*tool[j];
        if(!Math.isFinite(delta))throw "non-finite hobbing pose error";
        p.set_translation(i,p.get_translation(i)+delta);}
    }
    return p;
  }
}
