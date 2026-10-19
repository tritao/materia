package machinekit.component;
import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
/** Convex pieces for a union of constant-section machined plates. Round edges use 0.05 mm / 0.5 rad collision tessellation. Every hole
    is taken from the actual cap mesh, not replaced by the union's convex hull. */
class ExtrusionCollision {
  public static function pieces(parts:Array<Part>):CollisionHullFacet {
    var hulls:Array<Array<Float>> = [];
    for(part in parts){
      var box=part.shape.bounds(),lo=box.get_min(),hi=box.get_max(),spans=[hi.get_x()-lo.get_x(),hi.get_y()-lo.get_y(),hi.get_z()-lo.get_z()],axis=0;
      for(i in 1...3)if(spans[i]<spans[axis])axis=i;
      var frame=axis==0?F.fromRotationMatrix(0,0,0,[0.0,0,1,0,1,0,-1,0,0]):axis==1?F.fromRotationMatrix(0,0,0,[1.0,0,0,0,0,1,0,-1,0]):F.identity();
      var local=PosedParts.moved(part,F.inverse(frame));
      try {
        var b=local.shape.bounds();var facet=CollisionHullFacet.fromExtrusion(local,b.get_min().get_z(),b.get_max().get_z(),0.05,0.5);
        for(h in facet.hulls){var points:Array<Float> = [];for(i in 0...Std.int(h.length/3)){var p=F.transformPoint(frame,h[i*3],h[i*3+1],h[i*3+2]);points.push(p.x);points.push(p.y);points.push(p.z);}hulls.push(points);}
      }catch(e:Dynamic){local.close();throw e;}local.close();
    }
    return new CollisionHullFacet(hulls);
  }
}
