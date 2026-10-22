import machinekit.assembly.*;
import machinekit.component.*;
import machinekit.motion.*;
import cadkit.modeling.AssemblyState;
import materia.project.SceneArtifact;

/** Integration of shared recipes with optional Vallder offers and a complete installed axis. */
class VallderCatalogAxisChecks {
 static var checks=0;
 static function check(ok:Bool,label:String):Void {checks++;if(!ok)throw label;}
 static function bom(axis:MachineAssembly):String {
  var lines=axis.billOfMaterials().lines();lines.sort((a,b)->Reflect.compare(a.partNumber,b.partNumber));return haxe.Json.stringify(lines);
 }
 static function main():Void {
  var root=Sys.getEnv("MATERIA_ROOT"),directory=root+"/build/suppliers/vallder/",registry=MachineKitComponents.standard();
  var text=sys.io.File.getContent(directory+"recipes.json"),bindings=new SupplierRecipeLibrary(text,registry),data:Dynamic=haxe.Json.parse(text),rows:Array<Dynamic> = data.recipes;
  var bound=0;
  for(row in rows)if(["machinekit.motion.linear-rail","machinekit.motion.linear-rail-block","machinekit.motion.vallder-screw-support","machinekit.motion.sfu-nut","machinekit.motion.dsg-nut-housing","machinekit.motion.sfu-drawing-screw"].indexOf(row.type_id)>=0) {
   var requested=new ComponentValues(),required:Array<String> = row.required_inputs;
   for(name in required)requested.setNumber(name,name=="length" ? 400 : name=="raceRootDiameter" ? 13 : 10);
   var component=bindings.create("vallder",row.sku,requested);
   var type=component.componentType();
   check(type!=null && type.id==row.type_id,"Registered supplier binding "+row.sku);bound++;
  }
  check(bound==43,"Expected 43 new supplier bindings");
  var layout=new ScrewAxisLayout(250,500,400,160);
  var selected=new ScrewAxisComponents(new RbkMotorMount(),NemaStepper.model("23HS22-2804S"),new VallderSfu1605Screw(500),
   new SfuNut("SFU1605",13),new VallderBf12Support(),LinearRail.metric("HGH15CA",400,LinearRail.MEASURED),LinearRailBlock.metric("HGH15CA",LinearRailBlock.SUPPLIER),false);
  var axis=new TwinRailScrewAxis(layout,selected);axis.validate();
  var saved=axis.encode(),loaded=MachineAssembly.decode(saved,registry),before=bom(axis);
  check(before==bom(loaded),"Catalog axis BOM survives persistence");
  var state=new AssemblyState(loaded.definition());
  for(travel in [0.0,125,250]) {
   axis.setTravel(state,travel);var poses=loaded.solvedPoses(state),plate=poses.get("bracket");
   if(plate==null)throw "Missing travel bracket";
   check(Math.abs(plate.z-140-travel)<1e-6,"Catalog axis travel");
   check(Math.abs(state.joint("screwTurn")+travel*2*Math.PI/5)<1e-6,"Catalog axis screw lead");
  }
  var clearances=axis.installationClearances();
  check(clearances.length==168,"Complete required-clearance sweep");
  for(row in clearances)check(row.intersectionMm3<0.001,"Catalog axis clash "+row.a+"/"+row.b);
  var plan=new SourcingPlan();
  for(component in ([selected.rail,selected.block]:Array<MachineComponent>)) {
   var sku=component==selected.rail ? "1058001" : "1055001",source:Dynamic=null;
   for(row in rows)if(row.sku==sku)source=row;
   if(source==null)throw "Missing supplier binding";
   var offer:SupplierOffer={id:"vallder:"+sku,partNumber:component.bom.partNumber,supplierId:"vallder",supplierSku:sku,
    productUrl:source.source_url,observedAt:"2026-10-19T00:00:00Z",currency:"EUR",currencyMinorUnit:2,unitPriceMinor:null,
    taxBasis:"unspecified",inStock:false,compatibilityVerified:false};
   plan.add(offer);plan.select(offer.partNumber,offer.id);
  }
  var sourcing=plan.lines(axis.billOfMaterials());
  check(before==bom(axis),"Optional supplier selection preserves engineering BOM");
  var count=0;for(line in sourcing)if(line.selected!=null){count++;check(line.status=="review_required","No invented stock/price/compatibility");}
  check(count==2,"Vallder offers selected for shared rail/block recipes");
  var output=root+"/build/suppliers/configurable-axis/";
  sys.io.File.saveContent(output+"vallder-catalog.assembly.json",saved);
  sys.io.File.saveContent(output+"vallder-catalog.bom.json",haxe.Json.stringify(axis.billOfMaterials().lines(),null,"  "));
  sys.io.File.saveContent(output+"vallder-catalog.sourcing.json",haxe.Json.stringify(sourcing,null,"  "));
  sys.io.File.saveContent(output+"vallder-catalog.checks.json",haxe.Json.stringify({checks:checks,boundRecipes:bound,clearances:clearances},null,"  "));
  var scene=AssemblyPreview.scene(loaded,"vallder-catalog-axis"),definition=scene.assemblyDefinition;
  if(definition==null)throw "Missing axis definition";
  for(joint in definition.joints)if(joint.id=="travel")joint.driven=true;
  scene.presentation=materia.presentation.Presentations.assembled(scene.assemblyDefinition,Math.PI-0.9652516631899266,0.5639426413606289,"supplier-fasteners");
  sys.io.File.saveBytes(output+"vallder-catalog.mtrg",SceneArtifact.encode(scene));
  trace(checks+" catalog recipe, saved axis, sourcing and installation checks passed");
 }
}
