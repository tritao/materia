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
  var allocation:MountHoleErrorAllocation={placementOffsetMm:null,placementAngleRad:null,
   assemblyAdjustmentMm:null,cureBowMm:null,machiningErrorMm:null,pitchAccumulationMm:null,requiredMarginMm:null};
  var derived=MountHoleDerivation.derive(stock,C45Bright,rack,"rack",element.fingerprint("rack"),pose,
   element.connectors.map(c->c.name),element.thread,element.bore.diameter,element.bore.reference,
   [TransferAfterCasting],allocation);
  var facet=machinekit.structural.InsertHoles.facet(derived.holes,width,length,height);
  var parts:Array<Part>=[];
  function own(p:Part):Part {parts.push(p);return p;}
  var output=Sys.getEnv("LINEAR_MOUNT_EXPORT");
  if(output!=null && !sys.FileSystem.exists(output))sys.FileSystem.createDirectory(output);
  var bolts:Array<Dynamic>=[],contacts:Array<Dynamic>=[];
  var minimumClearance=1e30;
  try {
   var receiver=own(Solids.cut(Part.box(width,length,height),facet.drillingTools()));
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
