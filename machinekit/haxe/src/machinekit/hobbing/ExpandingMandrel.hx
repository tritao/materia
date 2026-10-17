package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hydraulic expanding mandrel. Reference row is Assumed; dimensions are simplified envelopes. */
class ExpandingMandrel extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("outside",59,"mm"),
    PrecisionPart.quantity("bore",12,"mm"),
    PrecisionPart.quantity("height",50,"mm"),
    PrecisionPart.quantity("workSeat",30,"mm"),
    PrecisionPart.quantity("expansion",0.05,"mm"),
    PrecisionPart.quantity("clampingPressure",10000000,"Pa"),
    PrecisionPart.quantity("maxPressure",15000000,"Pa"),
    PrecisionPart.quantity("eccentricity",0.004,"mm"),
    PrecisionPart.quantity("clampingDistortion",0.002,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.expanding-mandrel",parameters(),v -> new ExpandingMandrel(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Hydraulic expanding mandrel","HYDRAULIC-MANDREL-59",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("HYDRAULIC-MANDREL-59","Hydraulic expanding mandrel (reference envelope)","steel",v,true,[{name:"eccentricity",unit:"mm"},{name:"clampingDistortion",unit:"mm"}]);
    if(n("clampingPressure")<=0||n("clampingPressure")>n("maxPressure")||n("workSeat")>=n("height")||n("expansion")<=0)throw "invalid mandrel ratings";
    addConnector("work",Mount,Solids.axial(0,0,n("workSeat")));
  }
  override public function componentType():Null<ComponentType>return recipeType();
  public var clampingPressure(get,never):Float;
  function get_clampingPressure():Float return n("clampingPressure");
  public var eccentricity(get,never):Float;
  function get_eccentricity():Float return n("eccentricity");
  public var clampingDistortion(get,never):Float;
  function get_clampingDistortion():Float return n("clampingDistortion");

}
