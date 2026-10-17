package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Mineral-cast machine bed. Reference row is Assumed; dimensions are simplified envelopes. */
class MineralCastBed extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("width",650,"mm"),
    PrecisionPart.quantity("depth",550,"mm"),
    PrecisionPart.quantity("height",120,"mm"),
    PrecisionPart.quantity("columnX",0,"mm"),
    PrecisionPart.quantity("columnY",180,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.mineral-cast-bed",parameters(),v -> new MineralCastBed(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Mineral-cast machine bed","MINERAL-CAST-BED",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("MINERAL-CAST-BED","Mineral-cast machine bed (reference envelope)","mineral cast",v,false,[]);
    if(n("columnY")>=n("depth")/2)throw "column mounting datum outside bed";
    addConnector("column",Mount,Solids.axial(n("columnX"),n("columnY"),n("height")));
    addConnector("table",Mount,Solids.axial(0,-100,n("height")));
  }
  override public function componentType():Null<ComponentType>return recipeType();


}
