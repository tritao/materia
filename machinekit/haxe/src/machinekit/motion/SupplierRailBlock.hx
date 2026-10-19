package machinekit.motion;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Imported HGH carriage with the shared rail/top/mount interfaces and a portable pinned recipe.
    Geometry is supplier-reference CAD; load/preload and homogeneous steel density remain assumed. */
class SupplierRailBlock extends LinearRailBlock {
 static var recipe:Null<ComponentType>;
 final asset:String;
 final explicitPath:Null<String>;
 var source:Null<Shape>;
 var closed=false;
 public static function supplierRecipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.motion.supplier-hgh-block",[
   P.choice("profile",["HGR15","HGR25"],"HGR15"),P.text("cadSha256","")],v->new SupplierRailBlock(v.token("profile"),null,v.token("cadSha256")),true);
  return recipe;
 }
 public function new(profile:String,?path:String,expectedDigest:String="") {
  if(profile!="HGR15"&&profile!="HGR25")throw "Supplier carriage supports HGR15 and HGR25 only";
  asset=profile=="HGR15"?"HGH15CA":"HGH25CA";explicitPath=path;
  if(expectedDigest!=""&&expectedDigest!=SupplierRailAssets.digest(asset))throw "Saved supplier carriage revision is not installed";
  super(LinearRailSystem.catalog().get(profile),false,"-HIWIN-"+asset+"-STEP-"+SupplierRailAssets.digest(asset).substr(0,12));
  // Explicit imports preserve the inspection API's immediate rejection of unreviewed files.
  if(path!=null)load();
 }
 function load():Shape {
  if(closed)throw "Supplier carriage is closed";
  if(source==null){var imported=Shape.importStep(SupplierRailAssets.path(asset,explicitPath)),b=imported.bounds();
   var lo=b.get_min(),hi=b.get_max(),lower=asset=="HGH15CA"?-10.7:-16.5;
   if(Math.abs(lo.get_x()+spec.blockWidth/2)>0.02||Math.abs(hi.get_x()-spec.blockWidth/2)>0.02||Math.abs(lo.get_y()-lower)>0.02||Math.abs(hi.get_y()-(spec.blockHeight-spec.railHeight))>0.02||Math.abs(lo.get_z()+spec.blockLength/2)>0.02||hi.get_z()>spec.blockLength/2+10){imported.close();throw "Supplier carriage frame differs from reviewed interfaces";}
   source=imported;
  }
  return source;
 }
 override public function geometry(detail:ComponentDetail=Preview):Part {
  var s=load();if(detail!=Envelope)return new Part(s.cloneShape());
  var b=s.bounds(),lo=b.get_min(),hi=b.get_max();return Part.prism([new Vector(lo.get_x(),lo.get_y()),new Vector(hi.get_x(),lo.get_y()),new Vector(hi.get_x(),hi.get_y()),new Vector(lo.get_x(),hi.get_y())],lo.get_z(),hi.get_z());
 }
 override public function componentType():Null<ComponentType> return supplierRecipeType();
 override public function values():ComponentValues return new ComponentValues().setToken("profile",spec.designation).setToken("cadSha256",SupplierRailAssets.digest(asset)).setToken("material",materialSpec());
 public function close():Void {if(!closed){closed=true;if(source!=null)source.close();source=null;}}
}
