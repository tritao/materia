package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Rotary table platen. Reference row is Assumed; dimensions are simplified envelopes. */
class TablePlaten extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.quantity("outside",160,"mm"),
    P.quantity("bore",30,"mm"),
    P.quantity("height",20,"mm"),P.quantity("stemDiameter",0,"mm"),P.quantity("stemBore",0,"mm"),P.quantity("stemHeight",0,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.table-platen",parameters(),v -> new TablePlaten(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return EngineeringMetadata.assumedCatalog("Rotary table platen","TABLE-PLATEN-160",parameters(),"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)");
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("TABLE-PLATEN-160","Rotary table platen (reference envelope)","steel",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[]));

  }
  override public function componentType():Null<ComponentType>return recipeType();


  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    {
    var disk=ComponentEnvelopes.annular(values());
    if(parameterNumber("stemHeight")==0)return disk;
    var outer=parameterNumber("stemDiameter"),inner=parameterNumber("stemBore"),height=parameterNumber("stemHeight");
    if(inner<=parameterNumber("bore")||outer<=inner||outer>parameterNumber("outside")||height<=0)throw "Invalid integral table hub";
    return Solids.union([disk,Solids.cut(cadkit.modeling.Part.cylinderSpan(outer/2,-height,0),[cadkit.modeling.Part.cylinderSpan(inner/2,-height-0.1,0.1)])]);
  }
}
