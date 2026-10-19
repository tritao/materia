import cadkit.Shape;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import collisionkit.native.NativeCollisionWorld;
import collisionkit.CollisionPose;
import collisionkit.CollisionPair;
import collisionkit.CollisionGeometry;
import machinekit.robotics.CobotArm;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;

/** Independent backend calibration and exact-CAD audit of one reported cable/arm clash. */
class TendingElectricCollisionAudit {
  static function value(v:Dynamic):Float return Std.parseFloat(Std.string(v));
  static function field(v:Dynamic,k:String):Float return value(Reflect.field(v,k));
  static function frame(v:Dynamic):AssemblyFrame return {x:field(v,"x"),y:field(v,"y"),z:field(v,"z"),
    qx:field(v,"qx"),qy:field(v,"qy"),qz:field(v,"qz"),qw:field(v,"qw")};
  static function place(shape:Shape,f:AssemblyFrame):Shape {
    var m=AssemblyFrames.toRotationMatrix(f);
    return new Location(new Plane(new Vector(f.x,f.y,f.z),new Vector(m[0],m[3],m[6]),new Vector(m[2],m[5],m[8]))).apply(shape);
  }
  static function calibration():Array<{translationMm:Float,meshDistanceMm:Float,solidDistanceMm:Float,cadDistanceMm:Float,cadOverlapMm3:Float}> {
    var cube=Part.box(10,10,10),t=cube.shape.tessellate(0.05),vertices:Array<Float> = [],indices:Array<Int> = [];
    for(i in 0...t.vertexCount*3) vertices.push(t.vertices.getDouble(i*8));
    for(i in 0...t.indexCount) indices.push(t.indices.getInt32(i*4));
    var points:Array<Float> = [];for(x in [-5.0,5.0]) for(y in [-5.0,5.0]) for(z in [0.0,10.0]) {points.push(x);points.push(y);points.push(z);}
    var world=new NativeCollisionWorld(),a=world.addBody(0),b=world.addBody(1);
    var ao=world.add(a,CollisionPose.identity(),Mesh(vertices,indices)),bo=world.add(b,CollisionPose.identity(),Convex(points));
    var solidBody=world.addBody(2),solidObject=world.add(solidBody,CollisionPose.identity(),Convex(points));
    var out=[];
    for(dx in [20.0,10.4,10.0,8.0]) {
      world.setBodyPose(b,CollisionPose.translation(dx,0,0));
      var distance=world.pairDistances([new CollisionPair(ao,bo,a,b)])[0].distance;
      var solidDistance=world.pairDistances([new CollisionPair(solidObject,bo,solidBody,b)])[0].distance;
      var other=cube.shape.translate(new Vector(dx,0,0).native()),cad=cube.shape.distance(other),common=cube.shape.common(other);
      var volume=common.subshapeCount(CadKit.ShapeKind.Solid)>0 ? common.volume() : 0;common.close();other.close();
      var expected=Math.max(0,dx-10),expectedVolume=Math.max(0,10-dx)*100;
      if(Math.abs(cad-expected)>0.001 || Math.abs(volume-expectedVolume)>0.001) throw "CAD calibration disagrees with analytic boxes";
      if(dx>=10 && Math.abs(distance-expected)>0.002) throw "Mesh/convex distance calibration failed";
      if(dx<10 && distance>0.002) throw "Mesh/convex query missed intersecting surfaces";
      if(Math.abs(solidDistance-(dx-10))>0.002) throw "Solid convex signed-distance calibration failed";
      out.push({translationMm:dx,meshDistanceMm:distance,solidDistanceMm:solidDistance,cadDistanceMm:cad,cadOverlapMm3:volume});
    }
    cube.close();world.dispose();return out;
  }
  public static function main():Void {
    var directory=DhGripperPreview.directory()+"/../machine-tending/build/electric-access";
    var calibrated=calibration();Sys.println("Mesh and solid-convex calibration passed; mesh zero is ambiguous, solid penetration is signed");
    var reportFile=Sys.args().length>0 ? Sys.args()[0] : directory+"/report.json";
    var reportText=sys.io.File.getContent(reportFile);
    var report:Dynamic=haxe.Json.parse(reportText);
    var rows:Array<Dynamic> = cast Reflect.field(report,"results");
    var chosen:Dynamic=null,toolName="",armName="",reportedDistance=0.0;
    for(row in rows) {
      if(chosen!=null) break;
      if(Std.string(Reflect.field(row,"profile"))!="tending-top-entry") continue;
      var q:Array<Dynamic> = cast Reflect.field(row,"q");if(q.length!=6) continue;
      var hits:Array<Dynamic> = cast Reflect.field(row,"hits");
      for(hit in hits) {
        var first=Std.string(Reflect.field(hit,"first")),second=Std.string(Reflect.field(hit,"second"));
        var tool=StringTools.startsWith(first,"tool/") ? first : second;
        var arm=StringTools.startsWith(first,"arm/") ? first : second;
        if(tool.indexOf("electrical-cable")<0 || !StringTools.startsWith(arm,"arm/link")) continue;
        chosen=row;toolName=tool;armName=arm;reportedDistance=field(hit,"distanceMm");break;
      }
    }
    if(chosen==null) throw "No reported cable/link pair with a solved joint pose to audit";
    var q:Array<Dynamic> = cast Reflect.field(chosen,"q");
    var source=new CobotArm(Reach1300),model=new AssemblyModel("mm");source.addTo(model,"");
    var state=model.initialState("cable-cad-audit");for(i in 0...6) state.setJoint("j"+(i+1),value(q[i]));
    var placement=frame(Reflect.field(report,"armPlacement")),wrist=AssemblyFrames.compose(placement,state.worldPose("toolFlange"));
    var tool=SceneArtifact.decode(sys.io.File.getBytes(DhGripperPreview.directory()+"/build/dual/dual-electric-tool.mtrg"));
    var ts=new AssemblyState(tool.assemblyDefinition,tool.assemblyState);
    // Check the revised static service geometry against actual robot solids at this
    // solved pose. This is an independent CAD check, not a convex-hull verdict.
    var serviceChecks:Array<{tool:String,arm:String,distanceMm:Float,overlapMm3:Float}> = [];
    var serviceFiles=["a-electrical-cable","b-electrical-cable","support-0-base","support-1-base",
      "support-0-cap","support-1-cap","return-0-cap","return-1-cap"];
    var armSolids:Array<{id:String,shape:Shape}> = [];
    for(entry in source.components()) {
      if(!StringTools.startsWith(entry.id,"link") && !StringTools.startsWith(entry.id,"joint") && entry.id!="toolFlange") continue;
      var component=entry.component.geometry();
      armSolids.push({id:entry.id,shape:place(component.shape,AssemblyFrames.compose(placement,state.worldPose(entry.id)))});
      component.close();
    }
    for(id in serviceFiles) {
      var file=StringTools.endsWith(id,"electrical-cable") ? "/build/electrical-cable-local.step" : "/build/dual/"+id+".step";
      var local=Shape.importStep(DhGripperPreview.directory()+file);
      var service=place(local,AssemblyFrames.compose(wrist,ts.worldPose(id)));local.close();
      for(entry in armSolids) {
        var armSolid=entry.shape;
        var gap=service.distance(armSolid),volume=0.0;
        if(gap<0.01) {
          var shared=service.common(armSolid);
          if(shared.subshapeCount(CadKit.ShapeKind.Solid)>0) volume=shared.volume();shared.close();
        }
        serviceChecks.push({tool:id,arm:entry.id,distanceMm:gap,overlapMm3:volume});
      }
      service.close();
    }
    for(entry in armSolids) entry.shape.close();
    sys.io.File.saveContent(directory+"/service-cad-audit.json",haxe.Json.stringify({
      scope:"One solved robot pose; actual static service cable/support CAD. No flexible harness or motion clearance certification.",
      profile:Reflect.field(chosen,"profile"),station:Reflect.field(chosen,"station"),side:Reflect.field(chosen,"side"),
      sourceReportSha256:haxe.crypto.Sha256.encode(reportText),
      pairs:serviceChecks},null,"  "));
    var cableLocal=Shape.importStep(DhGripperPreview.directory()+"/build/electrical-cable-local.step");
    var cableCheck=new Part(cableLocal.cloneShape());
    if(!cableCheck.valid() || cableCheck.solidCount()!=1) throw "Cable CAD audit needs one valid solid";cableCheck.close();
    var cable=place(cableLocal,AssemblyFrames.compose(wrist,ts.worldPose(toolName.substr(5))));cableLocal.close();
    var linkPart:Part=null;for(entry in source.components()) if(entry.id==armName.substr(4)) linkPart=entry.component.geometry();
    if(linkPart==null) throw "Reported arm part has no CAD geometry";
    if(!linkPart.valid() || linkPart.solidCount()!=1) throw "Arm CAD audit needs one valid solid";
    var link=place(linkPart.shape,AssemblyFrames.compose(placement,state.worldPose(armName.substr(4))));linkPart.close();
    var exactDistance=cable.distance(link),common=cable.common(link);
    var overlap=common.subshapeCount(CadKit.ShapeKind.Solid)>0 ? common.volume() : 0;common.close();
    cable.exportStep(directory+"/audit-cable-world.step");link.exportStep(directory+"/audit-arm-link-world.step");cable.close();link.close();
    var conclusion=overlap>1 ? "CAD confirms physical interference" : exactDistance<0.01 ? "CAD confirms touch or very small interference; no cable clearance" : exactDistance<5 ? "No solid overlap; CAD confirms less than 5 mm clearance" : "Conservative hull flag; actual CAD clears 5 mm";
    sys.io.File.saveContent(directory+"/collision-audit.json",haxe.Json.stringify({calibration:calibrated,sourceReport:reportFile,sourceReportSha256:haxe.crypto.Sha256.encode(reportText),
      profile:Reflect.field(chosen,"profile"),station:Reflect.field(chosen,"station"),side:Reflect.field(chosen,"side"),
      tool:toolName,arm:armName,reportedMeshHullDistanceMm:reportedDistance,
      exactCadDistanceMm:exactDistance,exactCadOverlapMm3:overlap,conclusion:conclusion},null,"  "));
    Sys.println('$conclusion: $toolName / $armName; CAD gap $exactDistance mm, overlap $overlap mm3; hull result $reportedDistance mm');
  }
}
