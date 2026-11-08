package machinekit.hobbing;

import machinekit.component.BoltedFace;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;

/** Short ground bars clamp to the cast inserts before fitting their supports.
 * Flush recessed heads lie below the support foot: deliberately covered for
 * normal service, accessible after support removal for rare bar replacement.
 * The same joint declaration supplies the insert's source drilling. */
class HobberPadBolts {
  public static function thread(d:HobberDriveAxis,support:String):String return support=="fixed"?"M4":"M3";
  public static function face(d:HobberDriveAxis,support:String):BoltedFace {
    var b=HobberDrives.barBounds(d,support),n=d.axis.normal,a=d.axis.direction,ni=0;
    for(i in 0...3)if(Math.abs(n[i])>0.5)ni=i;
    var centre=[for(i in 0...3)(b[i]+b[i+3])/2];centre[ni]=n[ni]>0?b[ni]:b[ni+3];
    var frame=BoltedFace.frameOf({x:centre[0],y:centre[1],z:centre[2]},{x:n[0],y:n[1],z:n[2]},{x:a[0],y:a[1],z:a[2]});
    var points:Array<machinekit.component.BoltedFace.BoltHole>=[];
    if(support=="fixed") {
      var rows:Array<Float>=[];
      for(h in HobberDrives.seatRouteHoles(d,support))if(h.kind=="tapped") {
        var p=F.transformPoint(F.inverse(frame),h.position[0],h.position[1],h.position[2]);if(!rows.contains(p.x))rows.push(p.x);
      }
      rows.sort((x,y)->x<y?-1:x>y?1:0);var gap=0.0,middle=0.0;
      for(i in 1...rows.length)if(rows[i]-rows[i-1]>gap){gap=rows[i]-rows[i-1];middle=(rows[i]+rows[i-1])/2;}
      for(x in [middle-6,middle+6])for(y in [-12.0,12.0])points.push({x:x,y:y});
    } else for(y in [-5.0,5.0])points.push({x:0.0,y:y});
    var spec=SocketHeadCapScrew.catalog().get(thread(d,support)),recess=spec.headHeight+0.5;
    return BoltedFace.clamped("insert",frame,points,spec.clearanceMedium,HobberDrives.barThickness(d)-recess,
      spec.size,{diameter:spec.headDiameter+2,depth:recess},false);
  }
  public static function points(d:HobberDriveAxis,support:String):Array<{x:Float,y:Float,z:Float}> {
    var f=face(d,support);return [for(p in f.holes)F.transformPoint(f.frame,p.x,p.y,0)];
  }
}
