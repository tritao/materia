import machinekit.assembly.*;
import machinekit.component.*;
import machinekit.motion.*;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import cadkit.Shape;
import materia.assembly.AssemblyFrames as F;
import materia.project.SceneArtifact;

class SupportedSfu20AxisChecks {
 static var checks=0;
 static function check(ok:Bool,label:String):Void {checks++;if(!ok)throw label;}
 static function near(a:Float,b:Float,label:String):Void check(Math.abs(a-b)<1e-6,label+': $a != $b');
 static function rejects(fn:Void->Void,label:String):Void {var failed=false;try fn()catch(_:Dynamic)failed=true;check(failed,label);}
 static function bom(axis:MachineAssembly):String {var lines=axis.billOfMaterials().lines();lines.sort((a,b)->Reflect.compare(a.partNumber,b.partNumber));return haxe.Json.stringify(lines);}
 static function main():Void {
  var root=Sys.getEnv("MATERIA_ROOT"),output=root+"/build/suppliers/sfu20-axis/";if(!sys.FileSystem.exists(output))sys.FileSystem.createDirectory(output);
  rejects(()->new Sfu20AxisLayout(500),"Reject carriage beyond rail");
  rejects(()->new Sfu20AxisLayout(300,400),"Reject rail/support overlap");
  rejects(()->new Sfu20AxisLayout(300,600,450,100),"Reject carrier/guide interference");
  rejects(()->new Sfu20AxisLayout(Math.NaN),"Reject nonfinite travel");
  var merged=new Bom();merged.addComponent(new Sfu20AxisPart("base",new Sfu20AxisLayout(200)));merged.addComponent(new Sfu20AxisPart("base",new Sfu20AxisLayout(300)));
  check(merged.lines().length==1 && merged.lines()[0].quantity==2,"Travel range does not split identical fabrication BOM items");
  var axis=new SupportedSfu20Axis();axis.validate();
  var registry=MachineKitComponents.standard(),saved=axis.encode(),loaded=MachineAssembly.decode(saved,registry);
  check(bom(axis)==bom(loaded),"Complete engineering BOM survives reload");
  var state=new AssemblyState(loaded.definition());
  for(travel in [0.0,150,300]) {
   axis.setTravel(state,travel);var poses=loaded.solvedPoses(state),carriage=poses.get("carriage"),housing=poses.get("housing"),nut=poses.get("nut");
   if(carriage==null || housing==null || nut==null)throw "Missing moving member";
   near(carriage.z,200+travel,"Carriage travel");near(housing.z,180+travel,"DSG travel");near(nut.z,160+travel,"Nut travel");
   near(state.joint("screwTurn"),-travel*2*Math.PI/5,"Screw lead");
  }
  rejects(()->axis.setTravel(state,301),"Enforce travel bound");
  var zero=new AssemblyState(loaded.definition()),poses=loaded.solvedPoses(zero);
  var fixed=poses.get("fixed"),floating=poses.get("floating");if(fixed==null || floating==null)throw "Missing supports";
  var shoulder=F.compose(fixed,axis.fixed.connector("raceShoulder").frame);
  var nutSeat=F.compose(fixed,axis.fixed.connector("retainingNutSeat").frame);
  near(shoulder.z,100,"Race shoulder seats on BK15 rear sleeve");near(nutSeat.z,66,"Retaining nut bears on included front sleeve");
  var front=F.compose(floating,axis.floating.connector("bearingFront").frame),back=F.compose(floating,axis.floating.connector("bearingBack").frame);
  near(front.z,627,"BF15 bearing begins at floating journal shoulder");near(back.z,636,"BF15 bearing ends at ring groove");
  near(axis.screw.grooveStart+40,636,"Standard shaft groove matches bearing end");
  near(axis.screw.journalStart+40,72,"Plain fixed journal starts before bearings");
  near(86-10.5,75.5,"Front bearing seat on plain journal");near(86+7.5,93.5,"Rear bearing seat on plain journal");
  near(axis.screw.couplingDiameter,12,"Coupling shaft diameter");
  check(axis.screw.thread.designation==axis.nut.thread.designation,"Matching SFU2005 race/lead");
  for(h in axis.hardware)check(h.engagement>0 && h.engagement<=h.depth,"Receiving thread engagement "+h.id);
  var cache=new PosedParts();
  for(entry in loaded.components()){var p=cache.local(entry.component);check(p.valid() && p.volume()>0,"Valid component "+entry.id);}
  cache.close();Sys.println("SFU20 seats, hardware and saved assembly checked; sweeping all component pairs");
  var clearances=axis.installationClearances();
  var conflicts=[for(row in clearances)if(row.intersectionMm3>0.001)row];
  sys.io.File.saveContent(output+"clearances.json",haxe.Json.stringify({count:clearances.length,exemptPairs:axis.threadedPairs,conflicts:conflicts,samples:[0.0,0.625,1.25,1.875,2.5,3.125,3.75,4.375,150,299.375,300]},null,"  "));
  check(conflicts.length==0,"SFU20 installation clashes: "+haxe.Json.stringify(conflicts));
  var data:Dynamic=haxe.Json.parse(sys.io.File.getContent(root+"/build/suppliers/vallder/recipes.json")),bindings:Array<Dynamic> = data.recipes,plan=new SourcingPlan();
  var selections=[{id:"fixed",sku:"1012003"},{id:"floating",sku:"1013003"},{id:"railLeft",sku:"1058002"},{id:"blockLeft",sku:"1055002"},{id:"screw",sku:"1033003"},{id:"nut",sku:"1035003"},{id:"housing",sku:"1037002"}];
  var byId:Map<String,MachineComponent> = [];for(e in loaded.components())byId.set(e.id,e.component);
  var library=new SupplierRecipeLibrary(haxe.Json.stringify(data),registry);
  for(selection in selections) {
   var source:Dynamic=null;for(row in bindings)if(row.sku==selection.sku)source=row;if(source==null)throw "Missing Vallder binding "+selection.sku;
   var requested=new ComponentValues(),required:Array<String> = source.required_inputs;
   for(name in required)requested.setNumber(name,name=="length" ? (selection.id=="screw" ? 600 : 450) : name=="raceRootDiameter" ? 16 : name=="baseThreadDepth" ? 12 : 10);
   var bound=library.create("vallder",selection.sku,requested),component=byId.get(selection.id);if(component==null)throw "Missing sourced component";
   check(bound.bom.partNumber==component.bom.partNumber,"Exact source recipe identity "+selection.sku);
   var offer:SupplierOffer={id:"vallder:"+selection.sku,partNumber:component.bom.partNumber,supplierId:"vallder",supplierSku:selection.sku,
    productUrl:source.source_url,observedAt:"2026-10-19T00:00:00Z",currency:"EUR",currencyMinorUnit:2,unitPriceMinor:null,taxBasis:"unspecified",inStock:false,compatibilityVerified:false};
   plan.add(offer);plan.select(offer.partNumber,offer.id);
  }
  check(bom(loaded)==bom(axis),"Sourcing preserves engineering BOM");
  var scene=AssemblyPreview.scene(loaded,"SFU2005-BK15-BF15-DSG20-HGR20-axis"),definition=scene.assemblyDefinition;
  if(definition==null)throw "Missing preview definition";for(j in definition.joints)if(j.id=="travel")j.driven=true;
  scene.project={dynamicParts:[],motions:[],initialView:{yaw:Math.PI-0.9652516631899266,pitch:0.5639426413606289}};
  sys.io.File.saveBytes(output+"sfu20-axis.mtrg",SceneArtifact.encode(scene));
  sys.io.File.saveContent(output+"sfu20-axis.assembly.json",saved);
  sys.io.File.saveContent(output+"sfu20-axis.bom.json",haxe.Json.stringify(loaded.billOfMaterials().lines(),null,"  "));
  sys.io.File.saveContent(output+"sfu20-axis.sourcing.json",haxe.Json.stringify(plan.lines(loaded.billOfMaterials()),null,"  "));
  var exportCache=new PosedParts(),parts:Array<Part> = [],shapes:Array<Shape> = [];
  try {
   for(entry in loaded.components()){var pose=poses.get(entry.id);if(pose==null)throw "Missing STEP pose";var p=exportCache.posed(entry.component,pose);parts.push(p);shapes.push(p.shape);}
   var compound=Shape.compound(shapes);compound.exportStep(output+"sfu20-axis.step");compound.close();
   for(p in parts)p.close();exportCache.close();
  }catch(e:Dynamic){for(p in parts)p.close();exportCache.close();throw e;}
  sys.io.File.saveContent(output+"checks.json",haxe.Json.stringify({checks:checks,clearancePairs:clearances.length,parts:loaded.components().length,hardware:axis.hardware,
   assumptions:["DSG blind threads 12/10 mm","16 mm race root","M15x1 SW22 retaining-nut height 6 mm; retention/preload not qualified"],
   supportHashes:[Vallder15SupportSeats.BK_HASH,Vallder15SupportSeats.BF_HASH]},null,"  "));
  trace(checks+" SFU20 axis checks passed; "+clearances.length+" pair samples clear");
 }
}
