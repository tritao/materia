import materia.casting.CastingInput;
import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.structural.MountHoleDerivation;
import machinekit.structural.InsertMountPattern.MountHoleErrorAllocation;
import machinekit.structural.MountedElement;
import machinekit.structural.MountServiceScrews;
import materia.assembly.AssemblyFrames as F;

/** Review fixture, not a production mount or a frozen-router geometry change. */
class LinearMountContactTests {
 static var count=0;
 static function check(ok:Bool,message:String):Void {count++;if(!ok)throw message;}
 public static function main():Void {
  var rack=machinekit.transmission.GearRack.catalogue("29-15-055"),element=MountedElement.of(rack);
  var washer=machinekit.standard.FlatWasher.metric("M6");
  var width=28.0,height=20.0,length=rack.spec.lengthMm+40;
  // Receiver +Z is the side-bolt axis. The rack back plane is independent.
  var holeHeight=-rack.spec.pitchHeightMm+rack.spec.holeHeightMm;
  var pose=F.fromRotationMatrix(holeHeight,-rack.spec.lengthMm/2,height+washer.thickness+rack.spec.faceWidthMm,
   [0,-1,0,0,0,1,-1,0,0]);
  var stock={x:-width/2,y:-length/2,z:0.0,width:width,depth:length,height:height};
  var allocation:MountHoleErrorAllocation={placementOffsetMm:Unresolved("placementOffsetMm"),placementAngleRad:Unresolved("placementAngleRad"),
   assemblyAdjustmentMm:Unresolved("assemblyAdjustmentMm"),cureBowMm:Unresolved("cureBowMm"),machiningErrorMm:Unresolved("machiningErrorMm"),pitchAccumulationMm:Unresolved("pitchAccumulationMm"),requiredMarginMm:Unresolved("requiredMarginMm")};
  var derived=MountHoleDerivation.derive(stock,C45Bright,rack,"rack",element.fingerprint("rack"),pose,
   element.connectors.map(c->c.name),element.thread,element.bore.diameter,element.bore.reference,
   [TransferAfterCasting],allocation);
  var facet=machinekit.structural.InsertHoles.facet(derived.holes,width,length,height);
  var member=machinekit.structural.LinearMountSteel.receiver(width,length,height,derived);
  var parts:Array<Part>=[];
  function own(p:Part):Part {parts.push(p);return p;}
  var output=Sys.getEnv("LINEAR_MOUNT_EXPORT");
  if(output!=null && !sys.FileSystem.exists(output))sys.FileSystem.createDirectory(output);
  var bolts:Array<Dynamic>=[],contacts:Array<Dynamic>=[];
  var minimumClearance=1e30;
  try {
   var receiver=own(member.geometry());
   var registry=machinekit.component.MachineKitComponents.standard();
   var recipe=machinekit.structural.LinearMountSteel.recipeType();
   var rebuilt=registry.byId(recipe.id).create(member.values());
   check(registry.category(recipe.id)!=null && registry.kind(recipe.id)!=null,"shared receiver recipe is classified");
   check(recipe.key(rebuilt.values())==recipe.key(member.values()),"shared receiver recipe round trips its source drilling");
   check(Math.abs(rebuilt.massProperties().mass-member.massProperties().mass)<1e-12,"rebuild preserves computed steel mass");
   check(!Std.isOfType(member,machinekit.structural.CastInsert),"welded-host receiver is not a cast-in insert");
   var changed=false;try member.setMaterial("steel") catch(_:Dynamic)changed=true;
   check(changed,"C45 thread policy cannot be retained after switching material");
   var assembly=new machinekit.assembly.MachineAssembly();assembly.addComponent("receiver",member,F.identity());
   var restored=machinekit.assembly.MachineAssembly.decode(assembly.encode());
   check(restored.encode()==assembly.encode(),"receiver and source drilling round trip through assembly encoding");
   var state=new cadkit.modeling.AssemblyState(assembly.definition());
   var fem=machinekit.structural.fem.AssemblyFemExport.bodies(assembly,state,".",_ -> false);
   check(fem.parts.length==1 && Math.abs(fem.totalMassKg-member.massProperties().mass)<1e-12,
    "shared receiver mass and inertia enter existing FEM export exactly once");
   if(output!=null)sys.io.File.saveContent(output+'/fem-mass.json',haxe.Json.stringify(fem,null,"  "));
   var model:machinekit.structural.LinearMountCompliance.MountComplianceRecord={schema:1,host:"pad",element:"rack",
    frame:F.translation(0,0,10),terms:[{name:"receiver-bending",members:["receiver"],xMPerN:1e-8,yMPerN:2e-8,zMPerN:3e-8,
     reference:"deliberate test values; not router design inputs",assumed:true}]};
   var supported=new machinekit.assembly.MachineAssembly();
   var compliant=new machinekit.structural.LinearMountSteel(member.values().setToken("compliance",haxe.Json.stringify(model)));
   supported.addComponent("receiver",compliant,F.translation(10,20,30));
   supported.addComponent("pad",new machinekit.structural.LinearMountSteel(new ComponentValues()
    .setNumber("width",width).setNumber("length",length).setNumber("thickness",8)),F.translation(10,20,22));
   supported.addComponent("rack",rack,F.compose(F.translation(10,20,30),pose));
   var saved=machinekit.assembly.MachineAssembly.decode(supported.encode());
   check(saved.encode()==supported.encode(),"compliance recipe survives saved assembly round trip");
   var supportedFem=machinekit.structural.fem.AssemblyFemExport.bodies(saved,
    new cadkit.modeling.AssemblyState(saved.definition()),".",_ -> false);
   check(supportedFem.mounts.length==1 && supportedFem.mounts[0].hostId=="pad"
    && supportedFem.mounts[0].elementId=="rack","FEM discovers mount interface and resolves its real bodies automatically");
   check(Math.abs(supportedFem.mounts[0].frameMm.z-40)<1e-9,"exported mount frame follows receiver pose");
   check(Math.abs(supportedFem.mounts[0].stiffnessNPerMLocal.y-5e7)<1e-6,"export preserves directional flexibility in SI");
   var bodyMass=0.0;for(body in supportedFem.parts)bodyMass+=body.massKg;
   check(Math.abs(supportedFem.totalMassKg-bodyMass)<1e-12 && supportedFem.parts.length==3,
    "compliance export creates neither duplicate bodies nor additional lumped mass");
   model.terms[0].members=["missing"];
   var broken=new machinekit.assembly.MachineAssembly();
   broken.addComponent("receiver",new machinekit.structural.LinearMountSteel(member.values()
    .setToken("compliance",haxe.Json.stringify(model))));
   broken.addComponent("pad",member);broken.addComponent("rack",rack,pose);
   var refused=false;try machinekit.structural.fem.AssemblyFemExport.bodies(broken,
    new cadkit.modeling.AssemblyState(broken.definition()),".",_ -> false) catch(_:Dynamic)refused=true;
   check(refused,"FEM rejects a missing load-path member instead of exporting a hidden rigid support");
   if(output!=null)sys.io.File.saveContent(output+'/fem-compliance.json',haxe.Json.stringify(supportedFem,null,"  "));

   var raw=own(rack.geometry()),placed=own(PosedParts.moved(raw,pose));
   check(receiver.valid() && placed.valid(),"receiver and supplier rack are valid native solids");
   check(Math.abs(placed.distance(receiver)-washer.thickness)<1e-6,"rack stand-off equals actual washer thickness");
   var spacers:Array<Part>=[];
   for(i in 0...derived.holes.length) {
    var hole=derived.holes[i],localSeat=F.compose(element.connectors[i].frame,F.translation(0,-element.bore.counterboreDepth,0));
    var seat=F.compose(pose,localSeat);
    var screw=MountServiceScrews.select("rack/"+hole.id,element,seat,facet.holes[i],hole.frame,[20,25,30,35]);
    check(screw.length==25,"real M6x25 fits counterbore, grip, thread, tip gap and back wall");
    var screwPose=F.compose(seat,F.inverse(screw.connector("head").frame));
    var screwRaw=own(screw.geometry()),screwBody=own(PosedParts.moved(screwRaw,screwPose));
    var washerRaw=own(washer.geometry()),spacer=own(PosedParts.moved(washerRaw,F.translation(hole.frame.x,hole.frame.y,height)));
    spacers.push(spacer);
    var toRack=spacer.distance(placed),toSteel=spacer.distance(receiver);
    check(toRack<1e-6 && toSteel<1e-6,"real washer closes both sides of the rack load path");
    bolts.push({connector:element.connectors[i].name,size:"M6",lengthMm:screw.length,
     counterboreBearing:seat,washerThicknessMm:washer.thickness});
    contacts.push({connector:element.connectors[i].name,toRackMm:toRack,toSteelMm:toSteel});
    if(output!=null) {
     screwBody.shape.exportStep(output+'/bolt${i+1}.step');
     spacer.shape.exportStep(output+'/washer${i+1}.step');
    }
   }
   var pinion=new machinekit.transmission.HelicalPinion(new ComponentValues().setToken("hand","left"));
   var gearRaw=own(pinion.geometry(Envelope));
   for(i in 0...9) {
    var station=25+i*(rack.spec.lengthMm-50)/8;
    var gear=own(PosedParts.moved(gearRaw,F.translation(holeHeight-pinion.pitchDiameter()/2,
     -rack.spec.lengthMm/2+station,height+washer.thickness-(pinion.n("faceWidth")-rack.spec.faceWidthMm)/2)));
    var gap=gear.distance(receiver);
    for(spacer in spacers)gap=Math.min(gap,gear.distance(spacer));
    check(gap>=0.5,"pinion conservative envelope clears receiver and support hardware");
    minimumClearance=Math.min(minimumClearance,gap);
    if(output!=null && i==4)gear.shape.exportStep(output+'/pinion-envelope.step');
   }
   // Thin backing route: real through clearance, washer and nut; no blind tap.
   var rail=machinekit.motion.LinearRail.metric("HGH25CA",400,machinekit.motion.LinearRail.NOMINAL);
   var railElement=MountedElement.of(rail),backingThickness=8.0;
   var railPose=machinekit.assembly.AxisBuilder.orient(0,-200,backingThickness+rail.spec.railHeight,[0.0,0,1],[0.0,1,0]);
   var backing=new machinekit.structural.SeatedPlate(40,440,backingThickness,"thin backing fixture",[
    {name:"rail",frame:F.identity(),holes:rail.mountingConnectors().map(c->{
     var p=F.compose(railPose,c.frame);return {x:p.x,y:p.y,diameter:7.0};})}],null,"steel");
   var backingBody=own(backing.geometry()),railBody=own(PosedParts.moved(own(rail.geometry()),railPose));
   check(backingBody.distance(railBody)<1e-6,"thin backing retains the rail datum and real base contact");
   for(c in railElement.connectors) {
    var railSeat=F.compose(railPose,F.compose(c.frame,F.translation(0,-railElement.bore.counterboreDepth,0)));
    var stack=machinekit.structural.MountThroughBolts.select(c.name,railElement,railSeat,
     machinekit.component.Solids.axial(railSeat.x,railSeat.y,0),[20,25,30,35,40]);
    var w=own(PosedParts.moved(own(stack.washer.geometry()),stack.washerPose));
    var n=own(PosedParts.moved(own(stack.nut.geometry()),stack.nutPose));
    check(w.distance(backingBody)<1e-6 && w.distance(n)<1e-6,"nut and washer close the backing load path");
    check(stack.projectionMm>=2*stack.screw.thread.pitch-1e-6,"two pitches project beyond the fully engaged nut");
    check(stack.screw.length-stack.screw.threadLength<=stack.gripMm+stack.washer.thickness+1e-6,
     "unthreaded shank stays outside the nut");
   }
   if(output!=null) {
    receiver.shape.exportStep(output+'/receiver.step');placed.shape.exportStep(output+'/rack.step');
    sys.io.File.saveContent(output+'/contact.json',haxe.Json.stringify({reviewOnly:true,
     scope:"One sourced 500 mm section; no host brackets, adjustment bar, joints, replication, modes or fatigue qualified",
     rackDesignation:rack.spec.designation,sourceFingerprint:element.fingerprint("rack"),receiverMm:stock,
     receiverMaterial:"C45; engineering plate dimensions",receiverMassKg:receiver.volume()*7850e-9,
     minimumPinionClearanceMm:minimumClearance,bolts:bolts,contacts:contacts},null,"  "));
   }
  } catch(error:Dynamic) {for(p in parts)p.close();throw error;}
  for(p in parts)p.close();
  Sys.println('LinearMountContactTests: $count assertions passed; minimum pinion/receiver clearance $minimumClearance mm');
 }
}
