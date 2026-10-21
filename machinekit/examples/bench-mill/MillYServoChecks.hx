import machinekit.milling.*;
import machinekit.assembly.*;
import machinekit.component.*;
import machinekit.motion.HiwinHgh15ca;
import cadkit.modeling.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;

/** Selected Y drive in the real mill frame and XY stage, including the existing X return drive. */
class MillYServoChecks {
 public static function preview():haxe.io.Bytes return MillProject.small();

 static function check(ok:Bool,message:String):Void {if(!ok)throw message;}
 static function checkDetailedBlockSection():Void {
  var block=new HiwinHgh15ca().geometry(),raw=machinekit.motion.Hgr15RailGeometry.build(200,[]);
  var rail=raw.translated(new Vector(0,0,-100));raw.close();
  var common=block.intersect(rail),clearance=common.volume();common.close();
  check(clearance<1e-5,"Detailed carriage clears the undrilled continuous running profile");
  var misplaced=block.translated(new Vector(0,-1,0));common=misplaced.intersect(rail);
  var penetration=common.volume();common.close();misplaced.close();rail.close();block.close();
  check(penetration>1,"Running-profile check detects a 1 mm seating error");
 }
 public static function main():Void {
  checkDetailedBlockSection();
  if(Sys.args().indexOf("--preview-only")>=0){
   var bytes=preview(),decoded=SceneArtifact.decode(bytes),found=false;
   for(part in decoded.parts)if(part.id=="xyStage/screwY"){
    if(part.threads==null||part.threads.length!=1)throw "Y screw lost cosmetic thread metadata";
    var thread=part.threads[0];
    if(Math.abs(thread.origin[2]-51)>1e-5||thread.length<=0)throw "Thread texture includes a machined end";
    found=true;
   }
   if(!found)throw "Missing selected Y screw";
   var detailed=0;
   for(part in decoded.parts)for(axis in ["X","Y"])for(i in 0...4)if(part.id=='xyStage/block$axis$i')detailed++;
   if(detailed!=8)throw "Mill preview lost detailed carriage blocks";
   var root=Sys.getEnv("MATERIA_ROOT");
   sys.io.File.saveBytes(root+"/machinekit/examples/bench-mill/build/y-servo/mill-y-servo.mtrg",bytes);
   Sys.println("Selected mill preview and cosmetic screw thread passed");return;
  }

  var reference=new BenchMill(false,false),f=MillFramePreview.layout(reference),guide=new HiwinHgh15ca(),z=MillFramePreview.zLayout(reference,f,guide);
  var l=new MillXYStageLayout(f,z.spindleY,BenchMill.RAIL,guide,MillScrewDrive.m60dBk12()),frame=new MillFrame(f,z,l),stage=new MillXYStage(l);
  stage.validate();
  var cache=new PosedParts(),rows:Array<Dynamic> = [],poses:Map<String,AssemblyFrame> = stage.solvedPoses();
  var framePoses:Map<String,AssemblyFrame> = frame.solvedPoses();
  function overlap(a:MachineComponent,pa:AssemblyFrame,b:MachineComponent,pb:AssemblyFrame,label:String):Void {
   var v=cache.overlap(a,pa,b,pb);rows.push({pair:label,intersectionMm3:v});check(v<1e-5,label+": "+v+" mm3");
  }
  for(axis in ["X","Y"])for(i in 0...4)for(j in i+1...4){
   var a='block$axis$i',b='block$axis$j';
   overlap(stage.component(a),poses.get(a),stage.component(b),poses.get(b),'Detailed $a/$b');
  }
  var drive=["motorMountY","motorY","motorShaftY","screwY","couplingY","floatingBearingY","fixedPedestalY","floatingPedestalY"];
  for(id in drive)for(other in frame.components())overlap(stage.component(id),poses.get(id),other.component,framePoses.get(other.id),id+"/frame/"+other.id);
  for(pair in [["motorMountY","motorY"],["motorMountY","screwY"],["motorMountY","couplingY"],["motorMountY","fixedPedestalY"],
   ["floatingBearingY","screwY"],["floatingBearingY","floatingPedestalY"],["couplingY","screwY"],["couplingY","motorShaftY"]])
   overlap(stage.component(pair[0]),poses.get(pair[0]),stage.component(pair[1]),poses.get(pair[1]),pair[0]+"/"+pair[1]);
  Sys.println("Selected Y drive/frame and installed interfaces passed");
  // Straight key approaches, with the rear hatch removed for motor access.
  // Nominal receiving threads are intentional engagement, not clearance failures.
  var access:Array<{id:String,part:Part}> = [];
  for(station in [24.0,40])access.push({id:"coupling key "+station,part:Part.cylinderAlong(2.5,new Vector(0,l.yMotorFace-station,l.yScrewZ+12.6),Vector.Z(),30)});
  for(m in stage.components())if(StringTools.startsWith(m.id,"mountYBolt")||StringTools.startsWith(m.id,"bearingBoltYfloating")||StringTools.startsWith(m.id,"seatYAnchor")){
   var p:AssemblyFrame=poses.get(m.id);access.push({id:m.id,part:Part.cylinderSpan(3.0,p.z+6.1,p.z+36.1,p.x,p.y)});
  }
  var mounting:machinekit.motion.ServoMotor.ServoMounting=l.yMotor.mounting();
  for(hole in mounting.holes)access.push({id:"motor key "+hole.x+"/"+hole.y,part:Part.cylinderAlong(2.5,
   new Vector(hole.x,l.yMotorFace+mounting.flangeThickness+5.1,l.yScrewZ+hole.y),Vector.Y(),30)});
  for(tool in access){
   var toolBox=PosedParts.boxOf(tool.part);
   for(m in frame.components()){
    if(PosedParts.apart(toolBox,cache.box(m.component,framePoses.get(m.id))))continue;
    var fixed=cache.posed(m.component,framePoses.get(m.id)),common=tool.part.intersect(fixed),v=common.volume();common.close();fixed.close();
    rows.push({pair:"access "+tool.id+"/frame/"+m.id,intersectionMm3:v});check(v<1e-5,"Access blocked "+tool.id+"/"+m.id+": "+v);
   }
   // Each approach must also clear adjacent installed drive members.
   for(id in ["motorMountY","motorY","floatingBearingY"]){
    if(PosedParts.apart(toolBox,cache.box(stage.component(id),poses.get(id))))continue;
    var fixed=cache.posed(stage.component(id),poses.get(id)),common=tool.part.intersect(fixed),v=common.volume();common.close();fixed.close();
    check(v<1e-5,"Access blocked "+tool.id+"/"+id+": "+v);
   }
   tool.part.close();
  }
  Sys.println("Thirty mm fastener key approaches passed");
  var model=new AssemblyModel("mm");stage.addTo(model,"");var state=new AssemblyState(model.definition("mill-y-servo"));
  for(travel in [-l.yHalfTravel,0,l.yHalfTravel]){
   state.setJoint("y",travel);state.forwardKinematics();
   for(m in stage.components())if(!MillXYStageIntegrity.yFixed(m.id))for(id in drive)
    if(id!="screwY"||m.id!="nutY")overlap(m.component,state.worldPose(m.id),stage.component(id),state.worldPose(id),'Y=$travel ${m.id}/$id');
   check(Math.abs(state.joint("screwY-turn")+travel*2*Math.PI/5)<1e-6,"Signed 5 mm lead preserved");
   check(Math.abs(state.worldPose("motorY").y-l.yMotorFace)<1e-6,"Motor remains fixed");
   check(Math.abs(state.worldPose("saddle").y-(l.centreY-travel))<1e-6,"Configured Y travel preserved");
  }
  cache.close();
  Sys.println("Sampled drive motion passed; running full-stroke audit");
  MillXYStageIntegrity.check(stage,frame).throwIfErrors();
  Sys.println("Full-stroke XY/XYZ and motor extraction passed");
  var restored=MachineAssembly.decode(stage.encode(),MachineKitComponents.standard());restored.validate();
  check(restored.components().filter(m->m.id=="motorY")[0].component.designation==l.yMotor.designation,"Selected motor survives save");
  var root=Sys.getEnv("MATERIA_ROOT"),dir=root+"/machinekit/examples/bench-mill/build/y-servo";
  sys.FileSystem.createDirectory(dir);
  sys.io.File.saveContent(dir+"/checks.json",haxe.Json.stringify({status:"nominal CAD integration; not strength or preload qualification",clearances:rows,motor:l.yMotor.designation,travelMm:2*l.yHalfTravel,railLengthMm:f.yRailLength,baseDepthMm:f.baseDepth,frontY:f.front,
   fullStrokeAudit:"passed conservative XY/XYZ sweep and motor extraction",toolApproachMm:30,yOvertravelMm:l.yOvertravel,bayRoofZ:l.yBayTop,shaftPassageMm:[22,22],yCompliance:"unspecified; kinematic lead only",motorFaceY:l.yMotorFace,screwLengthMm:l.yDriveScrew.totalLength,clipState:"nominal DIN471 installed envelope; raw supplier clip retained separately",pending:["supplier coupling SKU","effective motor thread engagement","joint loads and preload","retaining ring elasticity and exact installed form"]},null,"  "));
  sys.io.File.saveContent(dir+"/xy-stage.assembly.json",stage.encode());
  sys.io.File.saveBytes(dir+"/mill-y-servo.mtrg",preview());
  Sys.println("Selected mill Y servo geometry, travel and persistence checks passed");
 }
}
