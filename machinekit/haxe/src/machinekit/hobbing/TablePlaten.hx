package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Rotary table platen. Reference row is Assumed; dimensions are simplified envelopes. */
class TablePlaten extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("outside",160,"mm"),
    PrecisionPart.quantity("bore",30,"mm"),
    PrecisionPart.quantity("height",20,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.table-platen",parameters(),v -> new TablePlaten(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Rotary table platen","TABLE-PLATEN-160",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("TABLE-PLATEN-160","Rotary table platen (reference envelope)","steel",v,true,[]);

  }
  override public function componentType():Null<ComponentType>return recipeType();


}
