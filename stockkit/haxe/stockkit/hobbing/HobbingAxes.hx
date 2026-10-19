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
  public function poseValues():Array<Float> {
    var ca=Math.cos(a),sa=Math.sin(a),cb=Math.cos(b),sb=Math.sin(b),cc=Math.cos(c),sc=Math.sin(c);
    var r=[-cb,-sa*sb,-ca*sb,sb,-sa*cb,-ca*cb,0.0,-ca,sa]; // columns
    var values=[for(i in 0...12)0.0];
    for(j in 0...3) {
      values[j]=cc*r[3*j]+sc*r[3*j+1];
      values[3+j]=-sc*r[3*j]+cc*r[3*j+1];
      values[6+j]=r[3*j+2];
    }
    values[9]=cc*x-sc*y*ca;values[10]=-sc*x-cc*y*ca;values[11]=z+y*sa;
    if(offset!=null){
      var tool=[offset.toolX,offset.toolY,offset.toolZ],work=[offset.workX,offset.workY,offset.workZ];
      for(i in 0...3){var delta=-work[i];for(j in 0...3)delta+=values[3*i+j]*tool[j];
        if(!Math.isFinite(delta))throw "non-finite hobbing pose error";
        values[9+i]+=delta;}
      if(offset.workTiltX!=0||offset.workTiltY!=0){
        // Inverse of Rx(tiltX)*Ry(tiltY), about the bearing centre.
        var cx=Math.cos(offset.workTiltX),sx=Math.sin(offset.workTiltX),
          cy=Math.cos(offset.workTiltY),sy=Math.sin(offset.workTiltY);
        for(v in [offset.workTiltX,offset.workTiltY,offset.workPivotZ])if(!Math.isFinite(v))throw "non-finite table tilt";
        for(j in 0...4){var i=j==3?9:j,xx=values[i],yy=values[j==3?10:3+j],
          zz=values[j==3?11:6+j]-(j==3?offset.workPivotZ:0);
          var ry=cx*yy+sx*zz,rz=-sx*yy+cx*zz;
          values[i]=cy*xx-sy*rz;values[j==3?10:3+j]=ry;
          values[j==3?11:6+j]=sy*xx+cy*rz+(j==3?offset.workPivotZ:0);
        }
      }
    }
    return values;
  }
  /** Materialize only accepted samples; midpoint error checks stay in Haxe. */
  public static function nativePose(values:Array<Float>):sk_rake_pose {
    var p=new sk_rake_pose();p.set_struct_size(sk_rake_pose.size());
    for(j in 0...9)p.set_rotation(j,values[j]);
    for(i in 0...3)p.set_translation(i,values[9+i]);
    return p;
  }
  public function pose():sk_rake_pose return nativePose(poseValues());
}
