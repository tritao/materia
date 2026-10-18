package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hob arbor, HSK-E25 gauge interface. Reference row is Assumed; dimensions are simplified envelopes. */
class HobArbor extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.quantity("outside",16,"mm"),
    P.quantity("bore",0,"mm"),
    P.quantity("height",80,"mm"),
    P.quantity("hobSeatLength",40,"mm"),
    P.quantity("radialRunout",0.002,"mm"),
    P.quantity("axialRunout",0.002,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.hob-arbor",parameters(),v -> new HobArbor(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return EngineeringMetadata.assumedCatalog("Hob arbor, HSK-E25 gauge interface","HOB-ARBOR-HSK-E25",parameters(),"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)");
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("HOB-ARBOR-HSK-E25","Hob arbor, HSK-E25 gauge interface (reference envelope)","steel",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[{name:"radialRunout",unit:"mm"},{name:"axialRunout",unit:"mm"}]));
    if(parameterNumber("hobSeatLength")<=0||parameterNumber("hobSeatLength")>=parameterNumber("height"))throw "hob seat exceeds arbor";
    addConnector("gaugeLine",Mount,Solids.axial(0,0,0));
    addConnector("hobCentre",Axis,Solids.axial(0,0,parameterNumber("height")/2));
    addConnector("support",Axis,Solids.axial(0,0,parameterNumber("height")));
  }
  public function interfaceName():String return "HSK-E25";
  override public function componentType():Null<ComponentType>return recipeType();


  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    return ComponentEnvelopes.annular(values());
}
