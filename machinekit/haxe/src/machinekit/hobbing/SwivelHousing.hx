package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Swivel-head housing. Reference row is Assumed; dimensions are simplified envelopes. */
class SwivelHousing extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.quantity("outside",180,"mm"),
    P.quantity("bore",100,"mm"),
    P.quantity("height",80,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.swivel-housing",parameters(),v -> new SwivelHousing(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return EngineeringMetadata.assumedCatalog("Swivel-head housing","SWIVEL-HOUSING",parameters(),"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)");
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("SWIVEL-HOUSING","Swivel-head housing (reference envelope)","cast iron",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[]));

  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    return Solids.cut(ComponentEnvelopes.annular(values()),[
      cadkit.modeling.Part.cylinderSpan(60,-0.1,16),
      cadkit.modeling.Part.cylinderSpan(75,16,61)]);
  override public function componentType():Null<ComponentType>return recipeType();


}
