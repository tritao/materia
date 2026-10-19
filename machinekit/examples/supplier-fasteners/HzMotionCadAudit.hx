import cadkit.Shape;
class HzMotionCadAudit {
 static function main() {
  var root=Sys.getEnv("MATERIA_ROOT");
  if(root==null)throw "Set MATERIA_ROOT to repository root";
  var records:Array<Dynamic> = haxe.Json.parse(sys.io.File.getContent(root+"/tools/suppliers/hzmotion-cad.json")).assets;
  var out:Array<Dynamic> = [];
  for(r in records){
   var path=root+"/build/suppliers/hzmotion/cad/assets/"+Std.string(r.sha256)+".step";
   var digest=haxe.crypto.Sha256.make(sys.io.File.getBytes(path)),hex=new StringBuf();
   for(i in 0...digest.length)hex.add(StringTools.hex(digest.get(i),2).toLowerCase());
   if(hex.toString()!=r.sha256)throw "Pinned STEP hash mismatch";
   var s=Shape.importStep(path);
   var b=s.bounds(),lo=b.get_min(),hi=b.get_max();
   var circles:Array<Dynamic> = [],planes:Array<Dynamic> = [],solids:Array<Dynamic> = [];
   for(i in 0...s.subshapeCount(CadKit.ShapeKind.Solid)){var solid=s.subshape(CadKit.ShapeKind.Solid,i),b=solid.bounds(),lo=b.get_min(),hi=b.get_max();solids.push({index:i,volume:solid.volume(),min:[lo.get_x(),lo.get_y(),lo.get_z()],max:[hi.get_x(),hi.get_y(),hi.get_z()]});solid.close();}
   for(i in 0...s.subshapeCount(CadKit.ShapeKind.Face)) {
    var f=s.subshape(CadKit.ShapeKind.Face,i);
    if(f.surfaceKind()==CadKit.SurfaceKind.Cylinder){var a=f.faceAxis(),o=a.get_origin(),d=a.get_direction(),bb=f.bounds(),mn=bb.get_min(),mx=bb.get_max();circles.push({r:a.get_radius(),o:[o.get_x(),o.get_y(),o.get_z()],d:[d.get_x(),d.get_y(),d.get_z()],min:[mn.get_x(),mn.get_y(),mn.get_z()],max:[mx.get_x(),mx.get_y(),mx.get_z()]});}
    if(f.surfaceKind()==CadKit.SurfaceKind.Plane && f.faceArea()>20){var c=f.center(),n=f.faceNormal();planes.push({area:f.faceArea(),center:[c.get_x(),c.get_y(),c.get_z()],normal:[n.get_x(),n.get_y(),n.get_z()]});}
    f.close();
   }
   out.push({solids:solids,planes:planes,cylinders:circles,model:r.model,sha256:r.sha256,volumeMm3:s.volume(),min:[lo.get_x(),lo.get_y(),lo.get_z()],max:[hi.get_x(),hi.get_y(),hi.get_z()]});
   if(r.model=="M60D-BK12-C") {
    var motor=false,base=false,pilot=false;
    for(p in planes){var n:Array<Float> = p.normal,c:Array<Float> = p.center;
     if(Math.abs(n[2]-1)<0.001&&Math.abs(c[2]-84.5)<0.001&&p.area>1000)motor=true;
     if(Math.abs(n[1]+1)<0.001&&Math.abs(c[1]+25)<0.001&&p.area>2000)base=true;}
    for(c in circles){var mn:Array<Float> = c.min,mx:Array<Float> = c.max;if(Math.abs(c.r-25)<0.001&&Math.abs(mn[2]-78.5)<0.001&&Math.abs(mx[2]-84)<0.001)pilot=true;}
    if(!motor||!base||!pilot)throw "BK12 planar datums/pilot differ from reviewed source";
    for(hole in [{x:25.0,z:60.5,r:3.3},{x:23.0,z:20.5,r:3.3},{x:23.0,z:7.5,r:3.3}])for(side in [-1,1]){
     var found=false;
     for(c in circles){var o:Array<Float> = c.o,d:Array<Float> = c.d;
      if(Math.abs(c.r-hole.r)<0.001&&Math.abs(o[0]-side*hole.x)<0.001&&Math.abs(o[2]-hole.z)<0.001&&Math.abs(d[1])>0.999)found=true;}
     if(!found)throw "BK12 base mounting hole differs from PDF p12 P/P1/E/C1/C2";
    }
   }
   if(r.model=="BF12"||r.model=="BF15") {
    var h=r.model=="BF12"?25.0:28.0,z=r.model=="BF12"?-10.0:10.0;
    var x=r.model=="BF12"?23.0:27.0,bore=r.model=="BF12"?5.0:7.5;
    function has(radius:Float,px:Float,pz:Float,vertical:Bool):Bool {
     for(c in circles){var o:Array<Float> = c.o,d:Array<Float> = c.d;
      if(Math.abs(c.r-radius)<0.001 && (vertical?Math.abs(d[1])>0.999&&Math.abs(o[0]-px)<0.001&&Math.abs(o[2]-pz)<0.001:Math.abs(d[2])>0.999&&Math.abs(o[0])<0.001&&Math.abs(o[1])<0.001))return true;}
     return false;
    }
    if(!has(bore,0,0,false)||!has(3.3,x,z,true)||!has(3.3,-x,z,true)||Math.abs(lo.get_y()+h)>0.001)throw "Support interface discrepancy";
   }
   s.close();
  }
  sys.io.File.saveContent(root+"/build/suppliers/hzmotion/cad/import-audit.json",haxe.Json.stringify(out,null,"  "));
  Sys.println("Imported "+out.length+" HZMotion STEP files");
 }
}
