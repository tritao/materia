package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Swivel-head housing. Reference row is Assumed; dimensions are simplified envelopes. */
class SwivelHousing extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("outside",180,"mm"),
    PrecisionPart.quantity("bore",100,"mm"),
    PrecisionPart.quantity("height",80,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.swivel-housing",parameters(),v -> new SwivelHousing(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Swivel-head housing","SWIVEL-HOUSING",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("SWIVEL-HOUSING","Swivel-head housing (reference envelope)","cast iron",v,true,[]);

  }
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    return Solids.cut(super.geometry(detail),[
      cadkit.modeling.Part.cylinderSpan(60,-0.1,16),
      cadkit.modeling.Part.cylinderSpan(75,16,61)]);
  override public function componentType():Null<ComponentType>return recipeType();


}
