import machinekit.assembly.*;
import machinekit.component.*;
import machinekit.motion.*;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;

/** Worked Y-drive stack, not installed in the mill. Uses an explicitly ordered
 * HZMotion screw-end profile; the existing Vallder screw is not modified.
 */
class HzMotionAxis extends MachineAssembly {
 public final mount=new HzMotionComponent("M60D-BK12-C");
 public final motor=LeadshineServo.model("ELM2H-0400LA60F");
 public final screw=new HzMotionDrawingScrew(400);
 public final nut=new VallderSfu1605Nut();
 public final coupling=new ShaftCoupling(14,10,25,32);
 public function new(supplierFloating:Bool=false) {
  super();
  addComponent("mount",mount);
  addComponent("motor",motor);addMate("motor-seat","fixed","mount","motor","motor","mountFace");
  var tip=mount.connector("screwShoulder").frame.z-HzMotionDrawingScrew.FIXED_END;
  addComponent("screw",screw,F.translation(0,25,tip));
  addMemberConnector("mount","screwSeat",Solids.axial(0,25,tip));
  addMateOnAxis("screw-turn","continuous","mount","screwSeat","screw","input",{x:0,y:1,z:0});
  attach("motorShaft",motor.shaft(),F.translation(0,25,0),"screw");
  attach("coupling",coupling,F.translation(0,25,16),"screw");
  var floating=tip+screw.totalLength-11;
  // BF12's 6000 bearing is 8 mm wide, centered in its 20 mm housing.
  attach("floating",new HzMotionComponent("BF12",null,"unspecified",supplierFloating?"supplier-step":"drawing"),F.translation(0,0,floating-6),"mount");
  addComponent("nut",nut,F.translation(0,25,120));
  addMemberConnector("mount","nutTravel",F.translation(0,25,120));
  addMemberConnector("nut","travelSeat",F.identity());
  addMateOnAxis("travel","prismatic","mount","nutTravel","nut","travelSeat",{x:0,y:0,z:1},0,
   {lower:0,upper:200,velocity:null,effort:null});
  addCoupling("screw-lead","travel","screw-turn",2*Math.PI/screw.thread.signedLead());
  var i=0;
  for(p in motor.boltPattern())attach('motorBolt${++i}',SocketHeadCapScrew.metric("M5",12),
   F.fromRotationMatrix(p.x,25+p.y,-motor.spec.flangeThickness,[1,0,0,0,-1,0,0,0,-1]),"mount");
 }
 function attach(id:String,part:MachineComponent,pose:AssemblyFrame,parent:String):Void {
  var parentPose=solvedPoses().get(parent);
  addComponent(id,part,pose);
  addMemberConnector(parent,'to-$id',F.compose(F.inverse(parentPose),pose));
  addMemberConnector(id,"attachment",F.identity());
  addMate('attach-$id',"fixed",parent,'to-$id',id,"attachment");
 }
 static function assert(ok:Bool,label:String):Void {if(!ok)throw label;}
 public static function checkAndWrite(catalogRows:Array<Dynamic>):Void {
  var axis=new HzMotionAxis();axis.validate();
  var report:Array<Dynamic> = [];
  var poses:Map<String,AssemblyFrame> = axis.solvedPoses();
  var cache=new PosedParts();
  var pairs=[["mount","motor"],["mount","motorShaft"],["mount","coupling"],["mount","screw"],
   ["screw","coupling"],["motorShaft","coupling"],["motor","motorShaft"],["floating","screw"]];
  for(pair in pairs) {
   var a=axis.components().filter(m->m.id==pair[0])[0],b=axis.components().filter(m->m.id==pair[1])[0];
   var v=cache.overlap(a.component,poses.get(a.id),b.component,poses.get(b.id));
   report.push({a:a.id,b:b.id,intersectionMm3:v});assert(v<1e-5,'Drive stack clash ${a.id}/${b.id}: $v');
  }
  cache.close();
  assert(axis.screw.designation!=new BallScrew(400).designation,"Required machining profile has a distinct BOM identity");
  var saved=axis.encode(),rebuilt=MachineAssembly.decode(saved,MachineKitComponents.standard());
  var before=axis.billOfMaterials().lines(),after=rebuilt.billOfMaterials().lines();
  before.sort((a,b)->Reflect.compare(a.partNumber,b.partNumber));after.sort((a,b)->Reflect.compare(a.partNumber,b.partNumber));
  assert(haxe.Json.stringify(before)==haxe.Json.stringify(after),"Complete drive BOM survives saved reconstruction");
  assert(new LeadScrew(BallScrew.race(),400).componentType()!=null,"Default base screw recipe remains available");
  assert(new LeadScrew(BallScrew.race(),400,"CUSTOM-END").componentType()==null,"Code-only base screw identity cannot silently reconstruct as generic");
  var rebuiltModel=new AssemblyModel("mm");rebuilt.addTo(rebuiltModel,"");
  var reopened=new AssemblyState(rebuiltModel.definition("saved-y-drive"));
  for(travel in [0.0,100,200]) {
   reopened.setJoint("travel",travel);reopened.forwardKinematics();
   assert(Math.abs(reopened.joint("screw-turn")+travel*2*Math.PI/5)<1e-7,"Saved screw lead retained");
   var p=reopened.worldPose("nut");assert(Math.abs(p.z-120-travel)<1e-6,"Nut follows travel");
   assert(p.z>axis.mount.connector("screwShoulder").frame.z&&p.z+axis.nut.bodyLength<34+axis.screw.totalLength-11,"Nut stays on race");
  }
  var tip=axis.mount.connector("screwShoulder").frame.z-HzMotionDrawingScrew.FIXED_END;
  assert(Math.abs(tip-34)<1e-6,"HZMotion prescribed end locates coupling tip");
  assert(tip-axis.motor.rating.shaftLength==4,"Four mm shaft-end gap");
  var couplingPose:AssemblyFrame=poses.get("coupling"),shaftPose:AssemblyFrame=poses.get("motorShaft");
  var couplingStart=couplingPose.z,motorEnd=shaftPose.z+axis.motor.rating.shaftLength;
  var motorEngagement=Math.min(motorEnd,couplingStart+axis.coupling.length/2)-couplingStart;
  var screwEngagement=couplingStart+axis.coupling.length-Math.max(tip,couplingStart+axis.coupling.length/2);
  assert(Math.abs(motorEngagement-14)<1e-6&&Math.abs(screwEngagement-14)<1e-6,"Actual shafts engage their respective coupling halves");
  for(hole in axis.coupling.setScrews) {
   var station=couplingStart+hole.z;
   assert(station<couplingStart+axis.coupling.length/2?station<motorEnd:station>tip,"Each clamp acts on a shaft, not the end gap");
  }
  var root=Sys.getEnv("MATERIA_ROOT");if(root==null)throw "Set MATERIA_ROOT to the repository root";
  var dir=root+"/build/suppliers/hzmotion";
  if(!sys.FileSystem.exists(dir))sys.FileSystem.createDirectory(dir);
  sys.io.File.saveContent(dir+"/y-drive.assembly.json",saved);
  sys.io.File.saveContent(dir+"/y-drive.bom.json",haxe.Json.stringify(axis.billOfMaterials().lines(),null,"  "));
  sys.io.File.saveContent(dir+"/checks.json",haxe.Json.stringify({status:"nominal drawing-based geometry checks only; procurement and manufacturing not qualified",
   catalog:catalogRows,driveClearances:report,travelChecksMm:[0,100,200],shaftEndGapMm:4,motorEngagementMm:14,screwEngagementMm:14,
   screwRequiredProfile:{source:"Motor_Bracket.pdf p10",fixedEndMm:51,bearingJournalMm:12,couplingSeatMm:10,couplingSeatLengthMm:15,lockThread:"M12x1",threadLengthMm:12},
   vallderCompatibility:{dropIn:false,fixedEndMm:54,reason:"Different prescribed end stack; no Vallder model was changed"},
   unresolved:["Drive stack retains drawing geometry: motor-mount STEP discrepancies and BF12 retaining-clip interference remain unresolved", "Webs, bearing internals, pilot recess and external fillets are simplified", "M5 engagement is nominal 5.4 mm; bracket effective thread depth is unpublished", "Coupling D25 L32 14/10 is a custom reference geometry, not a selected supplier SKU", "Supplier locknut/collars and bearing preload are included metadata, not independently articulated parts", "First Y stack has no mill frame attachment or guide/carriage structure", "BF12 mass and adapter masses are unknown; no axis sizing result uses guessed masses"]},null,"  "));
 }
}
