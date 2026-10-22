import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentValues;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

/**
 * One rail and one carriage recipe, each drawn three ways: the HGR15/HGR25 measured sections and the HGH15CA/HGH25CA
 * supplier exteriors are checked against the pinned HIWIN CAD, and against the nominal drawings they replace.
 */
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
  var output=root+"/machinekit/examples/linear-rail-reference/build/";
  if(!sys.FileSystem.exists(output))sys.FileSystem.createDirectory(output);
  var cache=LinearRailBlock.referenceDirectory()+"/";
  // Recipes and part identities need no supplier files; only a supplier drawing reads them.
  var configured=Sys.getEnv("MATERIA_HG_REFERENCE");
  Sys.putEnv("MATERIA_HG_REFERENCE",output+"deliberately-unavailable");
  var registry=machinekit.component.MachineKitComponents.standard();
  var unprepared=registry.byId("machinekit.motion.linear-rail-block").create(new ComponentValues().setToken("profile","HGH25CA").setToken("geometry","supplier"));
  check(unprepared!=null && unprepared.designation==LinearRailBlock.metric("HGH25CA").designation,"Supplier drawing keeps the carriage's part number without its CAD");
  var missing=false;try {unprepared.geometry();} catch(_:Dynamic) missing=true;check(missing,"Supplier drawing reports missing CAD");
  // A file whose digest differs from its review is rejected.
  var tampered=output+"tampered/";if(!sys.FileSystem.exists(tampered))sys.FileSystem.createDirectory(tampered);
  sys.io.File.saveContent(tampered+"HGH15CA.step","ISO-10303-21; unreviewed replacement");
  Sys.putEnv("MATERIA_HG_REFERENCE",tampered);
  var rejected=false;try {LinearRailBlock.metric("HGH15CA",LinearRailBlock.SUPPLIER).geometry();} catch(_:Dynamic) rejected=true;
  sys.FileSystem.deleteFile(tampered+"HGH15CA.step");sys.FileSystem.deleteDirectory(tampered);check(rejected,"Reject unreviewed supplier bytes");
  Sys.putEnv("MATERIA_HG_REFERENCE",configured==null || configured=="" ? cache : configured);
  var report:Array<Dynamic> = [];
  for(size in [{profile:"HGH15CA",rail:"HGR15R",tap:1.4},{profile:"HGH25CA",rail:"HGR25R",tap:2.5}]) {
   var block=LinearRailBlock.metric(size.profile,LinearRailBlock.SUPPLIER),nominalBlock=LinearRailBlock.metric(size.profile);
   check(block.designation==nominalBlock.designation,"Drawing leaves the carriage part number "+size.profile);
   near(block.massProperties().mass,nominalBlock.massProperties().mass,"Drawing leaves the datasheet mass "+size.profile);
   var recreated=block.componentType().create(block.values()),preview=block.geometry(),restored=recreated.geometry(),envelope=block.geometry(Envelope);
   near(preview.volume(),restored.volume(),"Supplier carriage recipe round-trip "+size.profile);restored.close();
   var inside=envelope.intersect(preview);near(inside.volume(),preview.volume(),"Envelope contains the supplier exterior "+size.profile);inside.close();
   // The measured section, drilled at the 150 mm reference's 15/75/135 holes, reproduces the supplier rail.
   var raw=new Part(Shape.importStep(cache+size.rail+".step")),supplierRail=raw.translated(new Vector(0,0,75));raw.close();
   var spec=nominalBlock.spec,bore=spec.railBore;
   var matched=machinekit.component.Solids.cut(LinearRail.measuredSection(size.rail,150),[for(z in [15.0,75,135]) for(tool in [
    Part.cylinderAlongY(bore.through/2,-spec.railHeight-1,1,0,z),Part.cylinderAlongY(bore.counterbore/2,-bore.depth,1,0,z)]) tool]);
   var lost=supplierRail.subtract(matched),added=matched.subtract(supplierRail),difference=lost.volume()+added.volume();
   check(difference<0.04,"Measured section and table bores reproduce the pinned "+size.rail+": "+difference);lost.close();added.close();matched.close();
   for(length in [150.0,237.0,440.0,600.0]) {
    var rail=LinearRail.metric(size.profile,length),part=rail.geometry(),box=rail.geometry(Envelope);
    check(rail.drawing==LinearRail.MEASURED,"HG rails default to their measured section");
    var outside=part.subtract(box);check(outside.volume()<0.001,"Rail drawing inside envelope");outside.close();box.close();
    for(z in [spec.blockLength/2+1,length/2,length-spec.blockLength/2-1]) {
     var carriage=preview.translated(new Vector(0,0,z)),common=part.intersect(carriage);
     check(common.volume()<0.001,"Measured rail clears the supplier carriage across travel "+size.profile);common.close();carriage.close();
    }
    for(z in rail.holePositions) {
     var through=Part.cylinderAlongY(bore.through/2-0.3,-spec.railHeight-1,1,0,z),counter=Part.cylinderAlongY(bore.counterbore/2-0.3,-bore.depth+0.01,1,0,z);
     var a=part.intersect(through),b=part.intersect(counter);
     check(a.volume()<0.001 && b.volume()<0.001,"Rail bores follow its mount connectors");a.close();b.close();through.close();counter.close();
    }
    part.close();
   }
   // All four tapped holes, independently of the connector coordinates.
   for(c in block.connectors()) if(StringTools.startsWith(c.name,"mount")) {
    var tool=Part.cylinderSpan(size.tap,c.frame.y-5,c.frame.y+1);
    var hole=tool.placed(new Location(new Plane(new Vector(c.frame.x,0,c.frame.z),Vector.X(),Vector.Y())));tool.close();
    var common=preview.intersect(hole);check(common.volume()<0.001,"Mount connector aligns with supplier tapped hole "+size.profile);common.close();hole.close();
   }
   var nominalRail=LinearRail.metric(size.profile,150,LinearRail.NOMINAL),nominalEnvelope=nominalRail.geometry(Envelope),mounted=preview.translated(new Vector(0,0,75));
   var seated=supplierRail.intersect(mounted);near(seated.volume(),0,"Supplier carriage seats on supplier rail "+size.profile);seated.close();
   var mismatch=nominalEnvelope.intersect(mounted),overlap=mismatch.volume();mismatch.close();
   check(overlap>1,"Comparison detects the rectangular rail envelope inside the supplier carriage");
   var nominalPreview=nominalBlock.geometry();
   report.push({profile:size.profile,rail:size.rail,supplierBlock:bounds(preview),nominalBlock:bounds(nominalPreview),
    supplierRail:bounds(supplierRail),measuredSupplierSymmetricDifferenceMm3:difference,rectangularRailWithSupplierBlockOverlapMm3:overlap});
   if(size.profile=="HGH15CA") {
    var nominal=new MachineAssembly();nominal.addComponent("rail",nominalRail);nominal.addComponent("carriage",nominalBlock,AssemblyFrames.translation(0,0,75));
    var supplier=new MachineAssembly();supplier.addComponent("rail",new BorrowedRail(supplierRail));supplier.addComponent("carriage",block,AssemblyFrames.translation(0,0,75));
    var measured=new MachineAssembly();measured.addComponent("rail",LinearRail.metric(size.profile,150));measured.addComponent("carriage",block,AssemblyFrames.translation(0,0,75));
    var comparison=new MachineAssembly();comparison.include("nominal",nominal,AssemblyFrames.translation(-55,0,0));comparison.include("measured",measured);
    comparison.include("supplier",supplier,AssemblyFrames.translation(55,0,0));
    sys.io.File.saveBytes(output+"comparison.mtrg",SceneArtifact.encode(AssemblyPreview.scene(comparison,"HGR15-HGH15CA-reference")));
   }
   nominalPreview.close();nominalEnvelope.close();mounted.close();supplierRail.close();preview.close();envelope.close();
  }
  sys.io.File.saveContent(output+"comparison.json",haxe.Json.stringify({sizes:report,checkedLengths:[150,237,440,600],
   notes:["Reference STEP exteriors are not load, preload or mass qualifications; masses and ratings come from the catalogue rows.",
    "Supplier carriages reach below the rail-top datum; the nominal carriage outline omits that volume."]},null,"  "));
  trace("One rail and one carriage recipe: measured sections, supplier exteriors, bores, tapped holes, envelopes and comparison export passed");
 }
}
private class BorrowedRail extends MachineComponent {
 final part:Part;
 public function new(part:Part) {super("HIWIN-HGR15R-STEP-150","Pinned HIWIN HGR15R 150 mm reference","steel",true);this.part=part;}
 override public function hasGeometry():Bool return true;
 override public function geometry(detail:ComponentDetail=Preview):Part return new Part(part.shape.cloneShape());
}
