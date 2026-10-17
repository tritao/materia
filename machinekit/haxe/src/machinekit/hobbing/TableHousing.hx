package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Rotary table housing. Reference row is Assumed; dimensions are simplified envelopes. */
class TableHousing extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("outside",180,"mm"),
    PrecisionPart.quantity("bore",80,"mm"),
    PrecisionPart.quantity("height",80,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.table-housing",parameters(),v -> new TableHousing(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Rotary table housing","TABLE-HOUSING-180",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("TABLE-HOUSING-180","Rotary table housing (reference envelope)","cast iron",v,true,[]);

  }
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    return Solids.cut(super.geometry(detail),[
      cadkit.modeling.Part.cylinderSpan(75,-0.1,45),
      cadkit.modeling.Part.cylinderSpan(60,45,61)]);
  override public function componentType():Null<ComponentType>return recipeType();


}
