package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hob spindle housing. Reference row is Assumed; dimensions are simplified envelopes. */
class SpindleHousing extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.quantity("outside",90,"mm"),
    P.quantity("bore",62,"mm"),
    P.quantity("height",180,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.spindle-housing",parameters(),v -> new SpindleHousing(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return EngineeringMetadata.assumedCatalog("Hob spindle housing","HOB-SPINDLE-HOUSING",parameters(),"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)");
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("HOB-SPINDLE-HOUSING","Hob spindle housing (reference envelope)","aluminium 6061",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    if(parameterNumber("bore")!=62||parameterNumber("height")!=180)throw "cartridge housing requires the 62 mm seats and 180 mm length";
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[]));

  }
  override public function componentType():Null<ComponentType>return recipeType();

  public function interfaceName():String return "ASSUMED-TF20";
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    return Solids.cut(cadkit.modeling.Part.cylinderSpan(parameterNumber("outside")/2,0,180),[
      cadkit.modeling.Part.cylinderSpan(31,-0.1,120),cadkit.modeling.Part.cylinderSpan(17,120,129),
      cadkit.modeling.Part.cylinderSpan(31,129,180.1)]);
}
