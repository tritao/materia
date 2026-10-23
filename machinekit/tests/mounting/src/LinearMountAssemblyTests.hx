import machinekit.structural.*;
import machinekit.component.*;
import materia.assembly.AssemblyFrames as F;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Review assembly only; frozen router remains untouched. Shared stock, source
 * holes, real hardware and assumed replication all retain their own identities. */
class LinearMountAssemblyTests {
 static var checks=0;
 static function check(ok:Bool,text:String):Void {checks++;if(!ok)throw text;}
 public static function main():Void {
  var rack=machinekit.transmission.GearRack.catalogue("29-15-055"),source=MountedElement.of(rack);
  var washer=machinekit.standard.FlatWasher.metric("M6");
  var height=22.0,thickness=20.0,length=540.0;
  var radius=50/Math.PI,back=58-radius-rack.spec.pitchHeightMm;
  var centre=back+rack.spec.holeHeightMm,foot=centre-height/2-12;
  var rackPose=F.fromRotationMatrix(rack.spec.pitchHeightMm-rack.spec.holeHeightMm,250,
   thickness+washer.thickness+rack.spec.faceWidthMm,[0,1,0,0,0,-1,-1,0,0]);
  var stock={x:-height/2,y:-length/2,z:0.0,width:height,depth:length,height:thickness};
  var u=materia.casting.CastingInput.Unresolved("Study placement/transfer accuracy not measured");
  var allocation:machinekit.structural.InsertMountPattern.MountHoleErrorAllocation={placementOffsetMm:u,placementAngleRad:u,
   assemblyAdjustmentMm:u,cureBowMm:u,machiningErrorMm:u,pitchAccumulationMm:u,requiredMarginMm:u};
  var derived=MountHoleDerivation.derive(stock,C45Bright,rack,"rack",source.fingerprint("rack"),rackPose,
   source.connectors.map(c->c.name),source.thread,source.bore.diameter,source.bore.reference,[TransferAfterCasting],allocation);
  var stations=derived.holes.map(h->h.frame.y),anchors:Array<Float>=[];
  stations.sort(Reflect.compare);for(i in 0...stations.length-1)anchors.push((stations[i]+stations[i+1])/2);
  var holes=derived.holes.copy();
  for(i in 0...anchors.length) {
   var h=InsertHoles.records(InsertHoles.through(7,foot))[0];h.id='anchor${i+1}';
   h.frame=F.fromRotationMatrix(-height/2,anchors[i],31.1,[0,0,-1,0,1,0,1,0,0]);
   h.counterbore={diameter:11,depth:6};holes.push(h);
  }
  var thread4=machinekit.standard.FastenerThreads.coarse(4).mating();
  for(i in 0...stations.length) {
   var h=InsertHoles.records(InsertHoles.tappedThrough(InsertHoles.metricThread(thread4),foot))[0];h.id='push${i+1}';
   h.frame=F.fromRotationMatrix(-height/2-foot,stations[i],31.1,[0,0,1,0,1,0,-1,0,0]);holes.push(h);
  }
  var member=new LinearMountSteel(LinearMountSteel.receiver(height,length,thickness,derived).values()
   .setNumber("flangeWidth",44.9).setNumber("flangeThickness",foot).setToken("holes",InsertHoles.fromNeutral(holes)));
  var assembly=new machinekit.assembly.MachineAssembly();
  assembly.addComponent("receiver",member);assembly.addComponent("rack",rack,rackPose);
  // One 140 mm replicated datum covers the rail and outward rack support.
  var rail=machinekit.motion.LinearRail.metric("HGH25CA",400,machinekit.motion.LinearRail.NOMINAL);
  var railPose=machinekit.assembly.AxisBuilder.orient(-centre+12+rail.spec.railHeight,-200,-38.9,[1.0,0,0],[0.0,1,0]);
  var padHoles:Array<materia.casting.InsertHole>=[],layerHoles:Array<materia.casting.InsertHole>=[];
  function clearance(id:String,x:Float,y:Float,d:Float):Void {
   for(pair in [{depth:8.0,rows:padHoles},{depth:4.0,rows:layerHoles}]) {
    var h=InsertHoles.records(InsertHoles.through(d,pair.depth))[0];h.id=id;h.frame.x=x;h.frame.y=y;pair.rows.push(h);
   }
  }
  for(i in 0...anchors.length)clearance('anchor${i+1}',50,-anchors[i],7);
  for(i in 0...stations.length)clearance('push-access${i+1}',50,-stations[i],3.5);
  for(c in rail.mountingConnectors()) {var p=F.compose(railPose,c.frame);clearance('rail-'+c.name,-20,-p.y,7);}
  var padPose=F.fromRotationMatrix(-centre,0,-18.9,[0,0,1,0,-1,0,1,0,0]);
  var layerPose=F.compose(padPose,F.translation(0,0,8));
  var pad=new LinearMountSteel(new ComponentValues().setNumber("width",140).setNumber("length",length).setNumber("thickness",8)
   .setToken("stock","weldable-mild-steel").setToken("role","host-pad").setToken("holes",InsertHoles.fromNeutral(padHoles)));
  var layer=new LinearMountReplication(new ComponentValues().setNumber("width",140).setNumber("length",length).setNumber("thickness",4)
   .setToken("holes",InsertHoles.fromNeutral(layerHoles)));
  assembly.addComponent("pad",pad,padPose);assembly.addComponent("replication",layer,layerPose);assembly.addComponent("rail",rail,railPose);
  var parts:Array<Part>=[];
  function own(p:Part):Part {parts.push(p);return p;}
  function placed(c:MachineComponent,p:materia.assembly.AssemblyRecord.AssemblyFrame):Part return own(machinekit.assembly.PosedParts.moved(own(c.geometry()),p));
  function clear(a:Part,b:Part,text:String):Void {var common=own(a.intersect(b));check(common.volume()<1e-7,text);}
  var output=Sys.getEnv("LINEAR_MOUNT_EXPORT");
  if(output!=null && !sys.FileSystem.exists(output))sys.FileSystem.createDirectory(output);
  var selected:Array<Dynamic>=[];
  try {
   var receiverBody=placed(member,F.identity()),rackBody=placed(rack,rackPose),padBody=placed(pad,padPose),layerBody=placed(layer,layerPose);
   var railBody=placed(rail,railPose);
   check(receiverBody.valid() && receiverBody.solidCount()==1,"receiver, integral adjustment bar and foot form one solid");
   check(receiverBody.distance(layerBody)<1e-6 && layerBody.distance(padBody)<1e-6,"rack support bears on the actual shared replicated datum");
   check(railBody.distance(layerBody)<1e-6,"master rail uses the same datum");
   clear(receiverBody,rackBody,"receiver clears the rack apart from stand-off washers");
   var sourceFacet=InsertHoles.facet(holes,height,length,thickness);
   for(i in 0...derived.holes.length) {
    var h=derived.holes[i],seat=F.compose(rackPose,F.compose(source.connectors[i].frame,F.translation(0,-source.bore.counterboreDepth,0)));
    var bolt=MountServiceScrews.select(h.id,source,seat,sourceFacet.holes[i],h.frame,[20,25,30]);
    var boltPose=F.compose(seat,F.inverse(bolt.connector("head").frame));
    var spacerPose=F.translation(h.frame.x,h.frame.y,thickness),spacerBody=placed(washer,spacerPose);
    check(spacerBody.distance(receiverBody)<1e-6 && spacerBody.distance(rackBody)<1e-6,"rack stand-off washer closes both physical contacts");
    assembly.addComponent('rack-bolt${i+1}',bolt,boltPose);assembly.addComponent('rack-spacer${i+1}',washer,spacerPose);
   }
   var ids=[for(i in 0...anchors.length)'hole-anchor${i+1}'];
   var footElement=new MountedElement(member,ids.map(id->member.connector(id)),
    {diameter:7,counterboreDiameter:11,counterboreDepth:6,reference:"Engineering foot clearance/counterbore; source geometry in receiver recipe"},source.thread);
   for(i in 0...anchors.length) {
    var connector=member.connector(ids[i]),seat=F.compose(connector.frame,F.translation(0,-6,0));
    var bolt=MountThroughBolts.select(ids[i],footElement,seat,F.alongY(-centre,anchors[i],31.1,1,0,0),[20,25,30]);
    var boltBody=placed(bolt.screw,bolt.screwPose),nutBody=placed(bolt.nut,bolt.nutPose),washerBody=placed(bolt.washer,bolt.washerPose);
    clear(boltBody,padBody,"foot bolt clears the welded-pad drilling");clear(boltBody,receiverBody,"foot bolt seats in its real counterbore");
    check(nutBody.distance(washerBody)<1e-6 && washerBody.distance(padBody)<1e-6,"outboard nut bears on the welded pad without an inaccessible tube nut");
    assembly.addComponent('anchor-bolt${i+1}',bolt.screw,bolt.screwPose);assembly.addComponent('anchor-nut${i+1}',bolt.nut,bolt.nutPose);
    assembly.addComponent('anchor-washer${i+1}',bolt.washer,bolt.washerPose);
    selected.push({connector:ids[i],lengthMm:bolt.screw.length,gripMm:bolt.gripMm,projectionMm:bolt.projectionMm});
   }
   for(i in 0...stations.length) {
    var screw=machinekit.standard.SocketSetScrew.forThread(thread4.mating(),12);
    var pose=F.fromRotationMatrix(-20,stations[i],31.1,[0,0,-1,0,1,0,1,0,0]);
    var tip=F.compose(pose,screw.connector("tip").frame),body=placed(screw,pose);
    check(Math.abs(tip.x+8)<1e-8,"adjuster tip sets the actual rack back rather than a fixed lip");
    check(body.distance(rackBody)<1e-6,"real adjuster reaches the rack back");
    assembly.addComponent('adjuster${i+1}',screw,pose);
    var key=own(Part.prism(cadkit.modeling.Polygon.regular(6,1.9),0,80));
    var keyPose=F.fromRotationMatrix(-20,stations[i],31.1,[0,0,-1,0,1,0,1,0,0]);
    var keyBody=own(machinekit.assembly.PosedParts.moved(key,keyPose));
    clear(keyBody,padBody,"adjuster key passes the real outboard pad opening");clear(keyBody,layerBody,"adjuster key passes the replicated datum opening");
   }
   var pinion=new machinekit.transmission.HelicalPinion(new ComponentValues().setToken("hand","left"));
   var pinionRaw=own(pinion.geometry(Envelope));
   for(i in 0...9) {
    var gear=own(machinekit.assembly.PosedParts.moved(pinionRaw,F.translation(rack.spec.pitchHeightMm-rack.spec.holeHeightMm+radius,-225+i*56.25,21.1)));
    check(gear.distance(receiverBody)>=1.0,"pinion envelope clears complete receiver/foot");
   }
   var block=machinekit.motion.LinearRailBlock.metric("HGH25CA",machinekit.motion.LinearRailBlock.NOMINAL);
   for(i in 0...9) {
    var pose=machinekit.assembly.AxisBuilder.orient(-centre+12+rail.spec.railHeight,-200+i*50,-38.9,[1.0,0,0],[0.0,1,0]);
    var body=placed(block,pose);check(body.distance(receiverBody)>1,"guide block sweep clears outboard foot");
   }
   var reload=machinekit.assembly.MachineAssembly.decode(assembly.encode());
   check(reload.encode()==assembly.encode(),"physical support and all catalogue hardware survive saved assembly reload");
   var fem=machinekit.structural.fem.AssemblyFemExport.bodies(reload,new cadkit.modeling.AssemblyState(reload.definition()),".",_ -> false);
   var mass=0.0;for(m in assembly.components())mass+=m.component.massProperties().mass;
   check(Math.abs(fem.totalMassKg-mass)<1e-10,"all physical mount members enter FEM mass once");
   if(output!=null) {
    receiverBody.shape.exportStep(output+"/receiver.step");padBody.shape.exportStep(output+"/pad.step");
    layerBody.shape.exportStep(output+"/replication.step");
    sys.io.File.saveContent(output+"/assembly.json",assembly.encode());sys.io.File.saveContent(output+"/mass.json",haxe.Json.stringify(fem,null,"  "));
    sys.io.File.saveContent(output+"/geometry.json",haxe.Json.stringify({reviewOnly:true,sourceRack:rack.spec.designation,lengthMm:length,
     footMm:foot,receiverHeightMm:height,receiverThicknessMm:thickness,flangeWidthMm:44.9,anchors:selected,
     datumMm:{carrier:12,rackBack:back,pinionCentre:58},
     limitations:["Local 500 mm rack/400 mm rail fixture; not a complete router clearance sweep","Assumed replication product/creep unresolved",
      "Welded pad/tube load and fatigue qualification pending","Nominal guide block used; supplier sweep still required","No new SR-D3 pass"]},null,"  "));
   }
  } catch(e:Dynamic) {for(p in parts)p.close();throw e;}
  for(p in parts)p.close();Sys.println('LinearMountAssemblyTests: $checks assertions passed');
 }
}
