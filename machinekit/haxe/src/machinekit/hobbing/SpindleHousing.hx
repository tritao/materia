package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hob spindle housing. Reference row is Assumed; dimensions are simplified envelopes. */
class SpindleHousing extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("outside",90,"mm"),
    PrecisionPart.quantity("bore",62,"mm"),
    PrecisionPart.quantity("height",180,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.spindle-housing",parameters(),v -> new SpindleHousing(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Hob spindle housing","HOB-SPINDLE-HOUSING",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("HOB-SPINDLE-HOUSING","Hob spindle housing (reference envelope)","aluminium 6061",v,true,[]);

  }
  override public function componentType():Null<ComponentType>return recipeType();

  public function interfaceName():String return "HSK-E25";
}
