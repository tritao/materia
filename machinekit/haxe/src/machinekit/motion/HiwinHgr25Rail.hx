package machinekit.motion;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Cut-length HGR25R, extruded from an undrilled section of the pinned supplier rail.
    Mounting pattern follows the shared catalogue; load/preload ratings remain assumed. */
class HiwinHgr25Rail extends LinearRail {
 static var recipe:Null<ComponentType>;
 public static function supplierRecipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.motion.supplier-hgr25-rail",[P.length("length",680),P.text("cadSha256","")],v->new HiwinHgr25Rail(v.number("length"),v.token("cadSha256")),true);return recipe;
 }
 public function new(length:Float,expectedDigest:String="") {
  if(expectedDigest!=""&&expectedDigest!=SupplierRailAssets.digest("HGR25R"))throw "Saved supplier rail revision is not installed";
  super(LinearRailSystem.catalog().get("HGR25"),length,false,"-HIWIN-HGR25R-STEP-"+SupplierRailAssets.digest("HGR25R").substr(0,12));
 }
 override public function componentType():Null<ComponentType> return supplierRecipeType();
 override public function values():ComponentValues return new ComponentValues().setNumber("length",length).setToken("cadSha256",SupplierRailAssets.digest("HGR25R")).setToken("material",materialSpec());
 /** Drawn from its own profile, whose shape sets its mass. */
 override public function massGeometry():Null<cadkit.modeling.Part> return null;

 override public function geometry(detail:ComponentDetail=Preview):Part return detail==Envelope?super.geometry(detail):profile(length,holePositions);
 public static function profile(length:Float,holes:Array<Float>):Part {
  var s=Shape.importStep(SupplierRailAssets.path("HGR25R")),b=s.bounds();
  if(Math.abs(b.get_min().get_x()+11.5)>0.02||Math.abs(b.get_min().get_y()+22)>0.02||Math.abs(b.get_max().get_y())>0.02){s.close();throw "HGR25R reference frame changed";}
  var face=s.section(new Vector(0,0,-45).native(),Vector.Z().native());s.close();
  var grown=face.extrude(new Vector(0,0,length).native());face.close();
  var p=new Part(grown),result=p.translated(new Vector(0,0,45));p.close();
  var tools:Array<Part> = [];
  for(z in holes){if(z<5.5||z>length-5.5){result.close();throw "HGR25 counterbore outside cut length";}tools.push(Part.cylinderAlongY(3.5,-23,1,0,z));tools.push(Part.cylinderAlongY(5.5,-9,1,0,z));}
  return Solids.cut(result,tools);
 }
}
