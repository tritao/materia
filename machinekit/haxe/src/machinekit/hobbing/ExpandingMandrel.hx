package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hydraulic expanding mandrel. Reference row is Assumed; dimensions are simplified envelopes. */
class ExpandingMandrel extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.quantity("outside",59,"mm"),
    P.quantity("bore",12,"mm"),
    P.quantity("height",50,"mm"),P.length("baseDiameter",0),P.length("baseHeight",0),
    P.quantity("workSeat",30,"mm"),
    P.quantity("expansion",0.05,"mm"),
    P.quantity("sleeveWall",1.5,"mm"),
    P.quantity("sleeveLength",16,"mm"),
    P.quantity("sleeveStart",26,"mm"),
    P.quantity("sleeveYoungs",210000,"N/mm²"),
    P.quantity("sleevePoisson",0.3,""),
    P.quantity("oilCavity",0.5,"mm"),
    P.quantity("oilBulkModulus",1500,"N/mm²"),
    P.quantity("contactStiffnessFactor",1,""),
    P.quantity("radialFitGap",0.025,"mm"),
    P.quantity("clampingPressure",10000000,"Pa"),
    P.quantity("maxPressure",15000000,"Pa"),
    P.quantity("eccentricity",0.004,"mm"),
    P.quantity("clampingDistortion",0.002,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.expanding-mandrel",parameters(),v -> new ExpandingMandrel(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return EngineeringMetadata.assumedCatalog("Hydraulic expanding mandrel","HYDRAULIC-MANDREL-59",parameters(),"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)");
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("HYDRAULIC-MANDREL-59","Hydraulic expanding mandrel (reference envelope)","steel",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[{name:"eccentricity",unit:"mm"},{name:"clampingDistortion",unit:"mm"}]));
    if(parameterNumber("clampingPressure")<=0||parameterNumber("clampingPressure")>parameterNumber("maxPressure")||parameterNumber("workSeat")>=parameterNumber("height")||parameterNumber("expansion")<=0)throw "invalid mandrel ratings";
    for(key in ["sleeveWall","sleeveLength","sleeveYoungs","oilCavity","oilBulkModulus","contactStiffnessFactor"])
      if(parameterNumber(key)<=0)throw 'invalid mandrel support $key';
    if(parameterNumber("sleeveStart")+parameterNumber("sleeveLength")>parameterNumber("height")||parameterNumber("sleeveWall")>=parameterNumber("outside")/2||parameterNumber("radialFitGap")<0
      ||parameterNumber("sleevePoisson")<=0||parameterNumber("sleevePoisson")>=0.5)throw "invalid sleeve/contact design";
    addConnector("work",Mount,Solids.axial(0,0,parameterNumber("workSeat")));
  }
  override public function componentType():Null<ComponentType>return recipeType();
  public var clampingPressure(get,never):Float;
  function get_clampingPressure():Float return parameterNumber("clampingPressure");
  public var eccentricity(get,never):Float;
  function get_eccentricity():Float return parameterNumber("eccentricity");
  public var clampingDistortion(get,never):Float;
  function get_clampingDistortion():Float return parameterNumber("clampingDistortion");

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    {
      var stem=ComponentEnvelopes.annular(values());
      if(parameterNumber("baseHeight")==0)return stem;
      if(parameterNumber("baseDiameter")<parameterNumber("outside")||parameterNumber("baseHeight")>=parameterNumber("workSeat"))throw "invalid integral mandrel base";
      var base=Solids.cut(cadkit.modeling.Part.cylinderSpan(parameterNumber("baseDiameter")/2,0,parameterNumber("baseHeight")),
        [cadkit.modeling.Part.cylinderSpan(parameterNumber("bore")/2,-0.1,parameterNumber("baseHeight")+0.1)]);
      return Solids.union([stem,base]);
    }
}
