import gearkit.*;
import machinekit.hobbing.*;
import machinekit.component.*;

class HobGeometryTests {
  static var checks=0;
  static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
  static function probeVolume(shape:cadkit.modeling.Part,x:Float,y:Float,z:Float):Float {
    var raw=cadkit.modeling.Part.sphere(0.01),probe=raw.translated(new cadkit.modeling.Vector(x,y,z));raw.close();
    try {var common=shape.combine(probe,cadkit.modeling.Mode.Intersect);var volume=common.volume();common.close();probe.close();return volume;}
    catch(e:Dynamic){probe.close();throw e;}
  }
  public static function run():Void {
    var reference=Sys.args().indexOf("reference")>=0,short=Sys.args().indexOf("short")>=0;
    var spec=reference?new GearSpec(0.3,200,8,20*Math.PI/180):new GearSpec(1,40,4,20*Math.PI/180);
    var starts=Sys.args().indexOf("two-start")>=0?2:1,rake=Sys.args().indexOf("rake")>=0?0.12:0.0;
    var design=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),starts,32,reference?12:4,rake);
    var hob=Hob.fromDesign(design,spec,short?8:40,16,true);
    if(Sys.args().indexOf("keyed")>=0)hob=new Hob(hob.values().copy().setNumber("keywayWidth",5));
    for(g in 0...design.gashes)for(thread in 0...design.starts)for(turn in [-1,0,1]){
      var edge=design.rakeFaceRegion(g,thread,turn),section=design.relievedFaceRegion(g,thread,turn,0,0);
      for(i in 0...edge.length)check(edge[i].x==section[i].x&&edge[i].y==section[i].y&&edge[i].z==section[i].z,"CAD front section differs from cutting edge");
    }
    var started=Sys.time();Sys.println("HOB_DETAIL_BUILD_START reference="+reference+" short="+short);Sys.stdout().flush();
    var input=Sys.getEnv("HOB_DETAIL_INPUT_STEP");
    var shape=input==null?hob.geometry(Preview):new cadkit.modeling.Part(cadkit.Shape.importStep(input)),edgeError=0.0;
    try {
      var envelope=hob.geometry(Envelope),volume=shape.volume(),bounds=shape.shape.bounds();
      check(shape.solidCount()==1,"detailed hob must be one connected solid");
      check(volume>0&&volume<envelope.volume(),"teeth and gashes must remove stock from the blank");envelope.close();
      check(Math.abs(bounds.get_min().get_z())<1e-5&&Math.abs(bounds.get_max().get_z()-hob.parameterNumber("height"))<1e-5,"hob end faces must be clipped at mounting datums");
      check(probeVolume(shape,0,0,hob.parameterNumber("height")/2)<1e-10,"actual axial bore is open");
      if(hob.parameterNumber("keywayWidth")>0){
        check(probeVolume(shape,0,hob.parameterNumber("bore")/2+hob.parameterNumber("keywayDepth")/2,hob.parameterNumber("height")/2)<1e-10,"actual keyway clears the key crown");
        check(probeVolume(shape,hob.parameterNumber("keywayWidth")/2+1,hob.parameterNumber("bore")/2+0.2,hob.parameterNumber("height")/2)>4.18e-6,"keyway leaves adjacent hub material");
      }
      var land=2*Math.PI/design.gashes*hob.parameterNumber("landFraction"),back=design.relievedFaceRegion(0,0,0,land/2,0),tip=back[0];
      for(p in back)if(p.x*p.x+p.y*p.y>tip.x*tip.x+tip.y*tip.y)tip=p;
      check(probeVolume(shape,tip.x,tip.y,tip.z+hob.parameterNumber("height")/2)<1e-10,"back of land actually clears the unrelieved cutting surface");
      if(short){
        var edges:Array<cadkit.Shape> = [];
        for(i in 0...shape.shape.subshapeCount(CadKit.ShapeKind.Edge))edges.push(shape.shape.subshape(CadKit.ShapeKind.Edge,i));
        var boundary=cadkit.Shape.compound(edges);for(edge in edges)edge.close();
        try {
          for(g in 0...2){var expected=design.rakeFaceRegion(g);
            for(sample in 0...8){var p=expected[Std.int(sample*(expected.length-1)/8)];
              var line=cadkit.modeling.Curve.line(new cadkit.modeling.Vector(p.x,p.y,p.z+hob.parameterNumber("height")/2),new cadkit.modeling.Vector(p.x,p.y,p.z+hob.parameterNumber("height")/2+1));
              var vertex=line.shape.subshape(CadKit.ShapeKind.Vertex,0);line.close();var distance=vertex.distance(boundary);vertex.close();edgeError=Math.max(edgeError,distance);
              check(distance<=hob.parameterNumber("edgeTolerance")*1.1+1e-6,'CAD edge differs from cutting edge by $distance mm');
            }
          }
        }catch(e:Dynamic){boundary.close();throw e;}boundary.close();
      }
      var report={reference:reference,short:short,seconds:Sys.time()-started,volumeMm3:volume,maxSampledCadEdgeErrorMm:edgeError,assertions:checks,importedStep:input};
      Sys.println("HOB_DETAIL_PASS "+haxe.Json.stringify(report));Sys.stdout().flush();
      var out=Sys.getEnv("HOB_DETAIL_EVIDENCE");
      if(out!=null){sys.io.File.saveContent(out+"/result.json",haxe.Json.stringify(report));
        if(input==null)shape.shape.exportStep(out+"/hob.step");
        if(Sys.getEnv("HOB_DETAIL_EXPORT_MESH")=="1"){
        Sys.println("HOB_DETAIL_MESH_START");Sys.stdout().flush();
        var rendered=machinekit.assembly.AssemblyPreview.part("hob","Detailed conjugate-rack hob",shape,hob.materialId,0.01);shape=null;
        sys.io.File.saveBytes(out+"/hob.mtrg",materia.project.SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:[rendered]}));}
      }
      Sys.println("HOB_DETAIL_COMPLETE seconds="+(Sys.time()-started));
    }catch(e:Dynamic){if(shape!=null)shape.close();throw e;}
    if(shape!=null)shape.close();
  }
}
