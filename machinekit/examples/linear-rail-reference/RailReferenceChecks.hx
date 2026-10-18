import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import machinekit.motion.HiwinHgh15ca;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

class RailReferenceChecks {
 static function check(ok:Bool,message:String):Void {if(!ok)throw message;}
 static function near(a:Float,b:Float,message:String):Void check(Math.abs(a-b)<0.02,message+': $a != $b');
 static function bounds(p:Part):Dynamic {
  var b=p.shape.bounds();return {min:[b.get_min().get_x(),b.get_min().get_y(),b.get_min().get_z()],
   max:[b.get_max().get_x(),b.get_max().get_y(),b.get_max().get_z()],volume:p.volume(),solids:p.solidCount()};
 }
 static function main():Void {
  var root=Sys.getCwd();
  while(!sys.FileSystem.exists(root+"/machinekit/examples/linear-rail-reference")) {
   var parent=haxe.io.Path.directory(root);if(parent==root || parent=="")throw "Repository not found";root=parent;
  }
  var output=root+"/machinekit/examples/linear-rail-reference/build/",cache=output+"supplier/";
  var railPath=cache+"HGR15R.step";
  check(HiwinHgh15ca.fileDigest(railPath)=="2897086393c72de3ca2a1af658da9e471bfbe38e234605e89dbb8497283e4351","Pinned rail STEP");
  var rejected=false;
  sys.io.File.saveContent(output+"tampered.step","ISO-10303-21; unreviewed replacement");
  try {new HiwinHgh15ca(output+"tampered.step");} catch(_:Dynamic) rejected=true;
  sys.FileSystem.deleteFile(output+"tampered.step");check(rejected,"Reject unreviewed supplier bytes");
  var block=new HiwinHgh15ca(cache+"HGH15CA.step"),raw=new Part(Shape.importStep(railPath));
  var rail=raw.translated(new Vector(0,0,75));raw.close();
  var oldBlock=LinearRailBlock.metric("HGR15"),oldRail=LinearRail.metric("HGR15",150);
  var preview=block.geometry(Preview), envelope=block.geometry(Envelope);
  var genericBlock=oldBlock.geometry(),genericRail=oldRail.geometry(Envelope);
  var generated=oldRail.geometry();
  var matched=machinekit.motion.Hgr15RailGeometry.build(150,[15,75,135]);
  var missing=rail.subtract(matched), extra=matched.subtract(rail);
  check(missing.volume()<0.02 && extra.volume()<0.02,"Generated running section and drilling reproduce pinned supplier rail");
  var profileDifference=missing.volume()+extra.volume(); missing.close(); extra.close(); matched.close();
  for(length in [150.0,237.0,440.0,600.0]) {
   var component=LinearRail.metric("HGR15",length), part=component.geometry(), box=component.geometry(Envelope);
   var outside=part.subtract(box);check(outside.volume()<0.001,"Rail preview inside envelope");outside.close();box.close();
   for(z in [31.0,length/2,length-31]) {
    var carriage=preview.translated(new Vector(0,0,z)), common=part.intersect(carriage);
    check(common.volume()<0.001,"Generated rail clears supplier block across travel");common.close();carriage.close();
   }
   for(z in component.holePositions) {
    var bore=Part.cylinderAlongY(2.2,-16,1,0,z), counter=Part.cylinderAlongY(3.7,-5.29,1,0,z);
    var a=part.intersect(bore), b=part.intersect(counter);
    check(a.volume()<0.001 && b.volume()<0.001,"Generated holes follow mount connectors");
    a.close();b.close();bore.close();counter.close();
   }
   part.close();
  }
  var b=rail.shape.bounds();near(b.get_min().get_z(),0,"Supplier rail start");near(b.get_max().get_z(),150,"Supplier rail length");
  near(b.get_min().get_y(),-15,"Supplier rail mounting face");
  for(c in oldBlock.connectors()) {
   var current=block.connector(c.name);check(current.role==c.role,"Connector roles preserved");
   near(current.frame.x,c.frame.x,"Connector X");near(current.frame.y,c.frame.y,"Connector Y");near(current.frame.z,c.frame.z,"Connector Z");
   near(current.frame.qx,c.frame.qx,"Connector orientation X");near(current.frame.qy,c.frame.qy,"Connector orientation Y");
   near(current.frame.qz,c.frame.qz,"Connector orientation Z");near(current.frame.qw,c.frame.qw,"Connector orientation W");
  }
  check(block.designation!=oldBlock.designation,"Distinct supplier/BOM/cache identity");
  var envelopeCommon=envelope.intersect(preview);near(envelopeCommon.volume(),preview.volume(),"Envelope contains the supplier exterior");envelopeCommon.close();
  var mounted=preview.translated(new Vector(0,0,75));
  var seated=rail.intersect(mounted);near(seated.volume(),0,"Supplier block seats on supplier rail without penetration");seated.close();
  var nominalInterference=genericRail.intersect(mounted),genericOverlap=nominalInterference.volume();nominalInterference.close();
  // Test all four tapped mounting holes, independently of the inherited connector coordinates.
  for(c in block.connectors()) if(StringTools.startsWith(c.name,"mount")) {
   var tool=Part.cylinderSpan(1.4,8,14);
   var bore=tool.placed(new Location(new Plane(new Vector(c.frame.x,0,c.frame.z),Vector.X(),Vector.Y())));tool.close();
   var common=preview.intersect(bore);near(common.volume(),0,"Mount connector aligns with supplier tapped bore");common.close();bore.close();
  }
  // This 150 mm supplier reference has holes at 15/75/135, unlike the current fixed-20 mm start policy.
  for(z in [15.0,75.0,135.0]) {
   var tool=Part.cylinderSpan(2,-16,1);
   var bore=tool.placed(new Location(new Plane(new Vector(0,0,z),Vector.X(),Vector.Y())));tool.close();
   var common=rail.intersect(bore);near(common.volume(),0,"Supplier rail through-hole axis");common.close();bore.close();
  }
  var old=new MachineAssembly();old.addComponent("rail",new BorrowedRail(genericRail,"NOMINAL-RECTANGULAR-RAIL-150"));old.addComponent("carriage",oldBlock,AssemblyFrames.translation(0,0,75));
  var supplier=new MachineAssembly();supplier.addComponent("rail",new BorrowedRail(rail));supplier.addComponent("carriage",block,AssemblyFrames.translation(0,0,75));
  var hybrid=new MachineAssembly();hybrid.addComponent("rail",oldRail);hybrid.addComponent("carriage",block,AssemblyFrames.translation(0,0,75));
  var comparison=new MachineAssembly();comparison.include("nominal",old,AssemblyFrames.translation(-55,0,0));comparison.include("generated",hybrid);comparison.include("supplier",supplier,AssemblyFrames.translation(55,0,0));
  check(genericOverlap>1,"Comparison detects the nominal rail/profile mismatch");
  sys.io.File.saveContent(output+"comparison.json",haxe.Json.stringify({
   supplierBlock:bounds(preview),nominalBlock:bounds(genericBlock),supplierRail:bounds(rail),nominalRail:bounds(genericRail),
   nominalRailHoleCentres:oldRail.holePositions,supplierRailHoleCentres:[15,75,135],
   nominalRailWithSupplierBlockOverlapMm3:genericOverlap,
   generatedRail:bounds(generated),generatedSupplierSymmetricDifferenceMm3:profileDifference,
   checkedLengths:[150,237,440,600],generatedPairNoPenetration:true,
   mountInterfacesPreserved:true,supplierPairNoPenetration:true,
   notes:["Reference STEP exterior is not a load/preload or mass qualification.","Supplier block reaches 10.7 mm below the rail-top datum; generic block omits this volume.",
    "Generated HGR15 preview uses the checked HGR15R running section; nominal envelopes remain conservative."]},null,"  "));
  sys.io.File.saveBytes(output+"comparison.mtrg",SceneArtifact.encode(AssemblyPreview.scene(comparison,"HGR15-HGH15CA-reference")));
  var owned=block.geometry(),volume=owned.volume();block.close();
  near(owned.volume(),volume,"Returned CAD remains owned after component closes");owned.close();
  rejected=false;try {block.geometry();} catch(_:Dynamic) rejected=true;check(rejected,"Reject geometry after component closes");
  generated.close();mounted.close();preview.close();envelope.close();genericBlock.close();genericRail.close();rail.close();
  trace("Supplier rail equivalence, four cut lengths, carriage fit, bores, connectors, envelopes and comparison export passed");
 }
}
private class BorrowedRail extends MachineComponent {
 final part:Part;
 public function new(part:Part, designation:String="HIWIN-HGR15R-STEP-150") {super(designation,"Matis-hosted HIWIN HGR15R 150 mm reference","steel",true);this.part=part;}
 override public function hasGeometry():Bool return true;
 override public function geometry(detail:ComponentDetail=Preview):Part return new Part(part.shape.cloneShape());
}
