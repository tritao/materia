package machinekit.hobbing;

import gearkit.HobDesign;
import gearkit.Point3;
import cadkit.modeling.Curve;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;

/** Detailed B-rep from the generating cutter, not an ornamental thread texture.
    One comb-shaped land is lofted, then repeated by the exact gash screw transform. */
class HobSolid {
  public static function retained(points:Array<Point3>,tolerance:Float):Array<Int> {
    var keep:Map<Int,Bool> = [];keep.set(0,true);keep.set(points.length-2,true);
    var stack=[{a:0,b:points.length-2}];
    while(stack.length>0){var segment=stack.pop(),a=points[segment.a],b=points[segment.b];
      var dx=b.x-a.x,dy=b.y-a.y,dz=b.z-a.z,den=dx*dx+dy*dy+dz*dz,worst=0.0,chosen=-1;
      for(i in segment.a+1...segment.b){var p=points[i];
        var t=den==0?0:Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy+(p.z-a.z)*dz)/den));
        var error=Math.sqrt(Math.pow(p.x-a.x-t*dx,2)+Math.pow(p.y-a.y-t*dy,2)+Math.pow(p.z-a.z-t*dz,2));
        if(error>worst){worst=error;chosen=i;}
      }
      if(worst>tolerance){keep.set(chosen,true);stack.push({a:segment.a,b:chosen});stack.push({a:chosen,b:segment.b});}
    }
    return [for(i in 0...points.length-1)if(keep.exists(i))i];
  }
  public static function build(hob:Hob):Part {
    var d=hob.design,length=hob.parameterNumber("height"),bore=hob.parameterNumber("bore");
    var land=2*Math.PI/d.gashes*hob.parameterNumber("landFraction"),relief=Math.tan(hob.parameterNumber("reliefAngle"));
    var edgeTolerance=hob.parameterNumber("edgeTolerance"),surfaceTolerance=hob.parameterNumber("surfaceTolerance");
    var radius=d.outsideDiameter/2,root=d.pitchDiameter/2-d.rootHeight,drop=radius*land*relief;
    if(drop>=0.9*(radius-root))throw "hob relief removes too much of the tooth land";
    var steps=Std.int(Math.ceil(land/(2*Math.acos(1-surfaceTolerance/radius))));steps=Std.int(Math.max(2,steps));
    if(steps>128)throw "hob surface tolerance needs more than 128 land sections";
    var raw=[for(i in 0...steps+1)d.relievedFaceRegion(0,0,0,land*i/steps,drop*i/steps)];
    var keep:Map<Int,Bool> = [];
    for(section in raw)for(index in retained(section,edgeTolerance))keep.set(index,true);
    var indices=[for(i in 0...raw[0].length-1)if(keep.exists(i))i];
    var first=Std.int(Math.floor(-length/(2*d.axialPitch)))-d.starts-2,last=Std.int(Math.ceil(length/(2*d.axialPitch)))+d.starts+2;
    if(indices.length*(last-first+1)*d.gashes>100000)throw "hob detail exceeds 100000 profile segments";
    var core=root-hob.parameterNumber("fluteExtraDepth");
    var inner=core-Math.min(0.5,(core-radius*Math.abs(Math.sin(d.rake)))/2);
    function trace(message:String):Void {if(Sys.getEnv("HOB_DETAIL_TRACE")=="1")Sys.stderr().writeString("HOB_SOLID "+Sys.time()+" "+message+"\n");}
    trace("profile points="+indices.length+" sections="+(steps+1)+" teeth="+(last-first+1));
    var sections:Array<Curve> = [];
    var parts:Array<Part> = [],base:Part=null,blank:Part=null,result:Part=null;
    try {
      for(i in 0...steps+1){
        var profile:Array<Vector> = [];
        for(tooth in first...last+1)for(index in indices){var p=raw[i][index];profile.push(new Vector(p.x,p.y,p.z+tooth*d.axialPitch+length/2));}
        var angle=Math.asin(radius*Math.sin(d.rake)/inner)-d.rake-land*i/steps;
        profile.push(new Vector(inner*Math.cos(angle),inner*Math.sin(angle),profile[profile.length-1].z));
        profile.push(new Vector(inner*Math.cos(angle),inner*Math.sin(angle),profile[0].z));
        sections.push(Curve.polyline(profile,true));
      }
      base=Part.loft(sections,false);trace("loft complete");for(section in sections)section.close();sections=[];
      blank=Part.cylinderSpan(radius,0,length);
      parts.push(Part.cylinderSpan(core,0,length));
      for(g in 0...d.gashes){var phase=2*Math.PI*g/d.gashes;
        var moved=PosedParts.moved(base,F.fromRotationMatrix(0,0,d.lead*g/d.gashes,[Math.cos(phase),-Math.sin(phase),0.0,Math.sin(phase),Math.cos(phase),0.0,0,0,1]));
        try {var clipped=moved.combine(blank,cadkit.modeling.Mode.Intersect);parts.push(clipped);trace("gash "+g+" clipped");}catch(e:Dynamic){moved.close();throw e;}moved.close();
      }
      base.close();base=null;blank.close();blank=null;
      result=Solids.union(parts);parts=[];trace("lands fused");
      var tools:Array<Part> = [];if(bore>0)tools.push(Part.cylinderSpan(bore/2,-0.1,length+0.1));
      var width=hob.parameterNumber("keywayWidth"),depth=hob.parameterNumber("keywayDepth");
      if(width>0){var y0=Math.sqrt(bore*bore/4-width*width/4)-0.01;
        var box=Part.box(width,bore/2+depth-y0,length+0.2),placed=box.translated(new Vector(0,(y0+bore/2+depth)/2,-0.1));box.close();tools.push(placed);}
      var cut=Solids.cut(result,tools);result=null;return Solids.named(cut,"hob");
    }catch(error:Dynamic){
      for(section in sections)section.close();for(part in parts)if(!part.shape.isClosed())part.close();
      if(base!=null&&!base.shape.isClosed())base.close();if(blank!=null&&!blank.shape.isClosed())blank.close();if(result!=null&&!result.shape.isClosed())result.close();throw error;
    }
  }
}
