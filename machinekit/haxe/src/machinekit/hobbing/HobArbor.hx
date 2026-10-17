package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hob arbor, HSK-E25 gauge interface. Reference row is Assumed; dimensions are simplified envelopes. */
class HobArbor extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("outside",16,"mm"),
    PrecisionPart.quantity("bore",0,"mm"),
    PrecisionPart.quantity("height",80,"mm"),
    PrecisionPart.quantity("hobSeatLength",40,"mm"),
    PrecisionPart.quantity("radialRunout",0.002,"mm"),
    PrecisionPart.quantity("axialRunout",0.002,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.hob-arbor",parameters(),v -> new HobArbor(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Hob arbor, HSK-E25 gauge interface","HOB-ARBOR-HSK-E25",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("HOB-ARBOR-HSK-E25","Hob arbor, HSK-E25 gauge interface (reference envelope)","steel",v,true,[{name:"radialRunout",unit:"mm"},{name:"axialRunout",unit:"mm"}]);
    if(n("hobSeatLength")<=0||n("hobSeatLength")>=n("height"))throw "hob seat exceeds arbor";
    addConnector("gaugeLine",Mount,Solids.axial(0,0,0));
    addConnector("hobCentre",Axis,Solids.axial(0,0,n("height")/2));
    addConnector("support",Axis,Solids.axial(0,0,n("height")));
  }
  public function interfaceName():String return "HSK-E25";
  override public function componentType():Null<ComponentType>return recipeType();


}
