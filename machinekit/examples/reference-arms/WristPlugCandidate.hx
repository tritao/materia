import cadkit.Shape;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import collisionkit.native.NativeCollisionWorld;
import collisionkit.CollisionPose;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPair;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;

typedef WristPlugStudy = {
  var id:String;
  var description:String;
  var externalBodyStartZMm:Float;
  var preferredClockDeg:Float;
  var offsetsMm:Array<Float>;
  var clocksDeg:Array<Float>;
  var separationBoundMm:Float;
  var inspectMatingZone:Bool;
}

/** Supplier geometry inspection; candidate placement never qualifies an OEM mating frame. */
class WristPlugCandidate {
  static function vector(v:CadKit.Vec3):Array<Float> return [v.get_x(),v.get_y(),v.get_z()];
  public static function pose(shape:Shape,f:AssemblyFrame):Shape {
    var m=AssemblyFrames.toRotationMatrix(f);
    return new Location(new Plane(new Vector(f.x,f.y,f.z),new Vector(m[0],m[3],m[6]),new Vector(m[2],m[5],m[8]))).apply(shape);
  }
  static function rendered(id:String,name:String,shape:Shape,color:Array<Float>):SceneArtifactPart {
    var m=shape.tessellate(0.1);
    return {id:id,name:name,red:color[0],green:color[1],blue:color[2],physicalMassKg:0,physicalMassDeclared:true,
      vertexCount:m.vertexCount,indexCount:m.indexCount,vertices:m.vertices,normals:m.normals,indices:m.indices,
      edgeSegments:m.edgeSegments,edgeIds:m.edgeIds,faceRanges:[for(r in m.faceRanges) {faceIndex:r.faceIndex,firstIndex:r.firstIndex,indexCount:r.indexCount}]};
  }
  static function writeScene(file:String,parts:Array<SceneArtifactPart>):Void {
    var model=new AssemblyModel("mm");for(p in parts) {model.add(p.id);model.ground(p.id);}
    var definition=model.definition("unqualified-wrist-plug-layout");for(d in definition.definitions) d.collisionHulls=[];
    sys.io.File.saveBytes(file,SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:parts,assemblyDefinition:definition,
      assemblyState:model.initialState("unqualified-wrist-plug-layout").record(),recipeDiagnostics:["COMMERCIAL CANDIDATE ONLY: seating, key clocking, electrical mapping and cable suitability unqualified. No tool or flexible cable modeled."]}));
  }
  static function collisionMesh(mesh:cadkit.Mesh,scale:Float):CollisionGeometry {
    var vertices:Array<Float> = [],indices:Array<Int> = [];
    for(i in 0...mesh.vertexCount*3) vertices.push(mesh.vertices.getDouble(i*8)*scale);
    for(i in 0...mesh.indexCount) indices.push(mesh.indices.getInt32(i*4));return Mesh(vertices,indices);
  }
  static function cp(f:AssemblyFrame):CollisionPose return new CollisionPose(f.x,f.y,f.z,f.qx,f.qy,f.qz,f.qw);
  public static function layout(root:String,directory:String,plug:Shape,study:Null<WristPlugStudy> = null):Void {
    if(study==null) study={id:"binder-7734290000200050500",description:"Binder straight M12 candidate",externalBodyStartZMm:15.8,preferredClockDeg:0,
      offsetsMm:[-8.0,0.0,8.0],clocksDeg:[0.0,90.0,180.0,270.0],separationBoundMm:0,inspectMatingZone:true};
    var config:WristPlugStudy=study;
    var robot=new ReferenceRobot(root+"/fairino5_v6"),profile=robot.wristInterfaces[0];
    var binding=ReferenceWristInterfaces.requireObservedGeometry(profile,"rs485");
    var frame=binding.observedFrame;
    var nominal:AssemblyFrame={x:frame.x*1000,y:frame.y*1000,z:frame.z*1000,qx:frame.qx,qy:frame.qy,qz:frame.qz,qw:frame.qw};
    var file=root+"/"+binding.sourceStep;
    if(ReferenceWristInterfaces.digest(sys.io.File.getBytes(file))!=binding.sourceStepSha256) throw "Changed robot STEP";
    var step=Shape.importStep(file),terminal=step.subshape(CadKit.ShapeKind.Solid,36);
    var shift=new Vector(820,-50,-102),wrist=terminal.translate(shift.native());terminal.close();
    // Exclude the complete mating/coupling region; its overlap is not evidence of a housing clash.
    var envelope=plug.bounds(),low=envelope.get_min(),high=envelope.get_max();
    if(high.get_z()<=config.externalBodyStartZMm) throw "Exterior cutoff excludes the whole connector";
    var clip=Part.box(high.get_x()-low.get_x()+2,high.get_y()-low.get_y()+2,high.get_z()-config.externalBodyStartZMm+1);
    var clipMoved=clip.shape.translate(new Vector((low.get_x()+high.get_x())/2,(low.get_y()+high.get_y())/2,config.externalBodyStartZMm).native());
    var body=plug.common(clipMoved);clipMoved.close();clip.close();
    var bodyBounds=body.bounds();
    if(body.subshapeCount(CadKit.ShapeKind.Solid)!=1) throw "Exterior region must remain one solid";
    // The axial cutoff must preserve the complete sideways elbow/strain-relief extent.
    var envelopeMesh=plug.tessellate(0.1),bodyLow=bodyBounds.get_min(),bodyHigh=bodyBounds.get_max();
    for(i in 0...envelopeMesh.vertexCount) {
      var x=envelopeMesh.vertices.getDouble(i*24),y=envelopeMesh.vertices.getDouble(i*24+8),z=envelopeMesh.vertices.getDouble(i*24+16);
      if(z<config.externalBodyStartZMm) continue;
      if(x<bodyLow.get_x()-0.001 || x>bodyHigh.get_x()+0.001 || y<bodyLow.get_y()-0.001 || y>bodyHigh.get_y()+0.001 || z>bodyHigh.get_z()+0.001)
        throw "Exterior clip truncated the connector envelope";
    }
    if(Sys.args().indexOf("--exterior-check-only")>=0) {
      sys.io.File.saveContent(directory+"/exterior-envelope-check.json",haxe.Json.stringify({candidate:config.id,
        cutoffZMm:config.externalBodyStartZMm,sourceLow:vector(low),sourceHigh:vector(high),
        exteriorLow:vector(bodyLow),exteriorHigh:vector(bodyHigh),allExteriorSourceVerticesContained:true},null,"  "));
      body.close();wrist.close();step.close();
      Sys.println("Exterior envelope guard passed for "+config.id+"; sideways elbow and strain relief retained");return;
    }
    var cad:Array<Dynamic> = [],nominalPlug:Shape=null;
    for(offset in config.offsetsMm) {
      var angle=config.preferredClockDeg*Math.PI/180;
      var at=AssemblyFrames.compose(nominal,{x:0,y:0,z:offset,qx:0,qy:0,qz:Math.sin(angle/2),qw:Math.cos(angle/2)});
      var exterior=pose(body,at),complete=pose(plug,at),checks:Array<Dynamic> = [];
      for(i in 0...step.subshapeCount(CadKit.ShapeKind.Solid)) {
        var original=step.subshape(CadKit.ShapeKind.Solid,i),obstacle=original.translate(shift.native());original.close();
        var eb=exterior.bounds(),ob=obstacle.bounds(),el=eb.get_min(),eh=eb.get_max(),ol=ob.get_min(),oh=ob.get_max();
        var dx=Math.max(0,Math.max(el.get_x()-oh.get_x(),ol.get_x()-eh.get_x()));
        var dy=Math.max(0,Math.max(el.get_y()-oh.get_y(),ol.get_y()-eh.get_y()));
        var dz=Math.max(0,Math.max(el.get_z()-oh.get_z(),ol.get_z()-eh.get_z()));
        var bound=Math.sqrt(dx*dx+dy*dy+dz*dz),exact=config.separationBoundMm<=0 || bound<config.separationBoundMm;
        var gap=exact ? exterior.distance(obstacle) : bound,volume=0.0;
        if(exact && gap<0.0001) {var shared=exterior.common(obstacle);if(shared.subshapeCount(CadKit.ShapeKind.Solid)>0) volume=shared.volume();shared.close();}
        checks.push({supplierSolid:i,externalBodyGapMm:gap,externalBodyOverlapMm3:volume,
          distanceKind:exact ? "exact-BRep-distance" : "conservative-AABB-separation-lower-bound"});obstacle.close();
      }
      var wholeGap:Null<Float> = null,wholeVolume:Null<Float> = null;
      if(config.inspectMatingZone) {
        wholeGap=complete.distance(wrist);var wholeCommon=complete.common(wrist);
        wholeVolume=wholeCommon.subshapeCount(CadKit.ShapeKind.Solid)>0 ? wholeCommon.volume() : 0;wholeCommon.close();
      }
      cad.push({assumedLeadingFaceOffsetMm:offset,clockDeg:config.preferredClockDeg,externalBodySourceZMinMm:config.externalBodyStartZMm,pairs:checks,
        entirePlugToWristGapMm:wholeGap,entirePlugToWristOverlapMm3:wholeVolume,
        note:config.inspectMatingZone ? "Whole-plug overlap includes unqualified mating region; it is not a functional fit test." : "Mating zone intentionally unevaluated; seating/key interface is unqualified."});
      writeScene(directory+"/fr5-candidate-offset-"+Std.int(offset)+".mtrg",[rendered("fr5-wrist","Supplier FR5 terminal wrist",wrist,[0.55,0.57,0.6]),
        rendered("binder-candidate","UNQUALIFIED "+config.description+"; assumed offset "+offset+" mm",complete,[0.15,0.5,0.8])]);
      if(offset==0) {nominalPlug=complete.cloneShape();complete.exportStep(directory+"/candidate-nominal-wrist-local.step");}
      complete.close();exterior.close();
    }
    var combined=Shape.compound([wrist,nominalPlug]);combined.exportStep(directory+"/fr5-and-candidate-nominal.step");combined.close();nominalPlug.close();
    var world=new NativeCollisionWorld(),objects:Array<{id:String,body:Int,object:Int}> = [];
    for(part in robot.scene.parts) {
      var vertices:Array<Float> = [],indices:Array<Int> = [];
      for(i in 0...part.vertexCount*3) vertices.push(part.vertices.getDouble(i*8));
      for(i in 0...part.indexCount) indices.push(part.indices.getInt32(i*4));
      var rb=world.addBody(objects.length);objects.push({id:part.id,body:rb,object:world.add(rb,CollisionPose.identity(),Mesh(vertices,indices))});
    }
    var candidateBody=world.addBody(objects.length),candidateObject=world.add(candidateBody,CollisionPose.identity(),collisionMesh(body.tessellate(0.1),0.001));
    var definition:materia.assembly.AssemblyDefinition=robot.scene.assemblyDefinition;
    var state=new AssemblyState(definition),joints=[for(j in robot.manifest.joints) if(j.type!="fixed") j];
    var screens:Array<Dynamic> = [];
    for(probeIndex in 0...robot.manifest.probes.length) {
      var probe=robot.manifest.probes[probeIndex];for(i in 0...joints.length) state.setJoint(joints[i].id,probe.q[i]);
      for(object in objects) world.setBodyPose(object.body,cp(state.worldPose(object.id)));
      for(offset in config.offsetsMm) for(clock in config.clocksDeg) {
        var angle=clock*Math.PI/180;
        var local=AssemblyFrames.compose(frame,{x:0,y:0,z:offset*0.001,qx:0,qy:0,qz:Math.sin(angle/2),qw:Math.cos(angle/2)});
        world.setBodyPose(candidateBody,cp(AssemblyFrames.compose(robot.linkPose(state,binding.parentLink),local)));
        var pairs=[for(o in objects) new CollisionPair(candidateObject,o.object,candidateBody,o.body)];
        var distances=world.pairDistances(pairs),results:Array<Dynamic> = [];
        for(i in 0...distances.length) {
          var distance=distances[i].distance;if(!Math.isFinite(distance)) throw "Nonfinite mesh distance";
          results.push({link:objects[i].id,surfaceDistanceMm:distance*1000});
        }
        screens.push({sourceProbe:probeIndex,q:probe.q,assumedLeadingFaceOffsetMm:offset,clockDeg:clock,pairs:results});
      }
    }
    world.dispose();body.close();wrist.close();step.close();
    sys.io.File.saveContent(directory+"/layout-report.json",haxe.Json.stringify({status:"Unqualified commercial candidate envelope study",candidate:config.id,preferredClockDeg:config.preferredClockDeg,
      sourceStepSha256:binding.sourceStepSha256,parentUrdfSha256:robot.manifest.urdfSha256,
      placement:"Plug CAD leading Z=0 plane referenced to observed socket insert; sensitivity offsets are assumptions, not measured seating depths. Clock zero follows wrist-up, not key clocking.",
      exactCadScope:"External geometry above the recorded normalized cutoff against all 37 static supplier solids. Separated boxes provide conservative distance lower bounds; nearby pairs use exact BRep distance/overlap. See distanceKind per pair.",
      separationBoundThresholdMm:config.separationBoundMm,matingZoneEvaluated:config.inspectMatingZone,
      cadCases:cad,meshScope:"External body triangle surfaces against imported ROS visual triangle surfaces over all prepared FK probes and four assumed clockings; positive surface distance does not rule out containment. No swept motion or flexible harness certificate.",
      meshCases:screens,flexibleCableModeled:false,toolModeled:false,matingFrameQualified:false},null,"  "));
    Sys.println("Saved candidate/wrist scenes, static supplier-CAD separation checks and "+screens.length+" representative-pose mesh screens; mating remains unqualified");
  }
  public static function inspectFile(file:String,directory:String,name:String = "Binder five-pin M12-A straight plug candidate"):Shape {
    sys.FileSystem.createDirectory(directory);
    var source=Shape.importStep(file),bounds=source.bounds();
    var cylinders:Array<Dynamic> = [],planes:Array<Dynamic> = [];
    for(i in 0...source.subshapeCount(CadKit.ShapeKind.Face)) {
      var face=source.subshape(CadKit.ShapeKind.Face,i),fb=face.bounds();
      if(face.surfaceKind()==CadKit.SurfaceKind.Cylinder) {
        var axis=face.faceAxis();cylinders.push({face:i,radius:axis.get_radius(),origin:vector(axis.get_origin()),
          direction:vector(axis.get_direction()),low:vector(fb.get_min()),high:vector(fb.get_max())});
      }
      if(face.surfaceKind()==CadKit.SurfaceKind.Plane) planes.push({face:i,centre:vector(face.center()),normal:vector(face.faceNormal()),area:face.faceArea(),low:vector(fb.get_min()),high:vector(fb.get_max())});
      face.close();
    }
    var mesh=source.tessellate(0.1);
    sys.io.File.saveBytes(directory+"/plug.vertices",mesh.vertices);sys.io.File.saveBytes(directory+"/plug.indices",mesh.indices);
    var part:SceneArtifactPart={id:"binder-candidate",name:name,red:0.2,green:0.45,blue:0.7,
      physicalMassKg:0,physicalMassDeclared:true,vertexCount:mesh.vertexCount,indexCount:mesh.indexCount,
      vertices:mesh.vertices,normals:mesh.normals,indices:mesh.indices,edgeSegments:mesh.edgeSegments,edgeIds:mesh.edgeIds,
      faceRanges:[for(r in mesh.faceRanges) {faceIndex:r.faceIndex,firstIndex:r.firstIndex,indexCount:r.indexCount}]};
    var model=new AssemblyModel("mm");model.add(part.id);model.ground(part.id);
    var definition=model.definition("binder-plug-candidate");for(d in definition.definitions) d.collisionHulls=[];
    sys.io.File.saveBytes(directory+"/supplier-plug.mtrg",SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:[part],assemblyDefinition:definition,
      assemblyState:model.initialState("binder-plug-candidate").record()}));
    sys.io.File.saveContent(directory+"/supplier-geometry.json",haxe.Json.stringify({sha256:ReferenceWristInterfaces.digest(sys.io.File.getBytes(file)),
      low:vector(bounds.get_min()),high:vector(bounds.get_max()),solids:source.subshapeCount(CadKit.ShapeKind.Solid),faces:source.subshapeCount(CadKit.ShapeKind.Face),
      cylinders:cylinders,planes:planes},null,"  "));
    return source;
  }
  public static function main():Void {
    var root=ReferenceAssetLocation.DIRECTORY;
    var file=root+"/cad/binder-m12-candidate/binder-7734290000200050500.step";
    var directory=root+"/wrist-plug-candidate";
    if(ReferenceWristInterfaces.digest(sys.io.File.getBytes(file))!="a732683ed40958c19c86017b76da9bcf731ece1b4655a8ea8e82e2a6e9f377c1") throw "Changed Binder STEP candidate";
    var source=inspectFile(file,directory);
    if(source.subshapeCount(CadKit.ShapeKind.Solid)!=1 || source.subshapeCount(CadKit.ShapeKind.Face)!=308) throw "Changed Binder candidate topology";
    layout(root,directory,source);source.close();
    Sys.println("Imported Binder candidate supplier STEP and saved exact surface observations");
  }
}
