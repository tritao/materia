import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames as F;

/** Native checks of the study's continuous mount profiles against real router
 * members. Profiles omit production drilling: overlap is a blocked candidate,
 * not a claim that an approved detailed mount exists. Frozen CAD is untouched. */
class RouterMountClearanceTests {
 static function blank(b:Dynamic):Part {
  var raw=Part.box(b.maxX-b.minX,b.maxY-b.minY,b.maxZ-b.minZ);
  var result=raw.translated(new Vector((b.maxX+b.minX)/2,(b.maxY+b.minY)/2,(b.maxZ+b.minZ)/2));raw.close();return result;
 }
 public static function main():Void {
  var input=Sys.getEnv("ROUTER_MOUNT_SCREEN"),output=Sys.getEnv("ROUTER_MOUNT_NATIVE");
  if(input==null||output==null)throw "Set absolute ROUTER_MOUNT_SCREEN and ROUTER_MOUNT_NATIVE paths";
  var plan:Dynamic=haxe.Json.parse(sys.io.File.getContent(input)),rows:Array<Dynamic>=[];
  var beds:Array<Dynamic>=plan.beds;
  for(bed in beds) {
   var router=new SheetRouter(new SheetRouterSpec(bed.bedMm));
   var state=new AssemblyState(router.definition());
   var flags:Array<Dynamic>=bed.nativeChecksRequired;
   var candidates:Map<String,Part>=[],members:Map<String,Part>=[];
   try {
    for(flag in flags) {
     var key=flag.side+"/"+flag.candidate;
     if(!candidates.exists(key))candidates.set(key,blank(Reflect.field(Reflect.field(bed.candidateBoxes,flag.side),flag.candidate)));
     if(!members.exists(flag.existingMember)) {
      var body=router.component(flag.existingMember).geometry();
      if(!body.valid())throw 'Invalid existing solid ${flag.existingMember}';
      members.set(flag.existingMember,body);
     }
     var minimum=1e100,maximum=0.0,governing:Dynamic=null,checked=0;
     for(x in [0.0,800.0,1600.0])for(y in [0.0,bed.bedMm/2.0,bed.bedMm*1.0])for(z in [0.0,200.0]) {
      state.setJoint("x",x);state.setJoint("y",y);state.setJoint("z",z);state.forwardKinematics();
      var placed=machinekit.assembly.PosedParts.moved(members.get(flag.existingMember),state.worldPose(flag.existingMember));
      var target=candidates.get(key),common=placed.intersect(target);
      var volume=common.volume(),gap=placed.distance(target);common.close();placed.close();checked++;
      if(!Math.isFinite(volume)||!Math.isFinite(gap))throw "Non-finite native clearance";
      if(volume>maximum) {maximum=volume;governing={x:x,y:y,z:z};}minimum=Math.min(minimum,gap);
     }
     var result={bedMm:bed.bedMm,side:flag.side,candidate:flag.candidate,existingMember:flag.existingMember,
      poses:checked,minimumGapMm:minimum,maximumIntersectionMm3:maximum,governingPoseMm:governing,
      disposition:maximum>1e-6?"BLOCKED: continuous profile intersects native member; redesign proposal required":
       (minimum<1e-5?"CONTACT: service/assembly allowance unresolved":"CLEAR: native solid separates from continuous profile")};
     rows.push(result);Sys.println(haxe.Json.stringify(result));Sys.stdout().flush();
    }
   }catch(e:Dynamic){for(p in candidates)p.close();for(p in members)p.close();throw e;}
   for(p in candidates)p.close();for(p in members)p.close();
  }
  sys.io.File.saveContent(output,haxe.Json.stringify({status:"Native profile/member check; detailed mount release remains open",
   sourceScreenSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(input)),rows:rows,
   limitations:["Continuous candidate profiles omit detailed drilling and joints","Frozen router geometry unchanged",
    "Service wrench and cassette withdrawal paths remain unqualified","No structural solve or dynamic acceptance"]},null,"  "));
 }
}
