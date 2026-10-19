package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hob arbor, assumed taper-and-face interface. Reference row is Assumed; dimensions are simplified envelopes. */
class HobArbor extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.quantity("outside",16,"mm"),
    P.quantity("bore",0,"mm"),
    P.quantity("height",80,"mm"),
    P.quantity("hobSeatLength",40,"mm"),P.quantity("hobCentreOffset",0,"mm"),
    P.quantity("supportDiameter",16,"mm"),P.quantity("supportLength",0,"mm"),
    P.quantity("radialRunout",0.002,"mm"),
    P.quantity("axialRunout",0.002,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.hob-arbor",parameters(),v -> new HobArbor(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return EngineeringMetadata.assumedCatalog("Hob arbor, assumed taper-and-face interface","HOB-ARBOR-TF20",parameters(),"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)");
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("HOB-ARBOR-TF20","Hob arbor, assumed taper-and-face interface (reference envelope)","steel",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[{name:"radialRunout",unit:"mm"},{name:"axialRunout",unit:"mm"}]));
    if(parameterNumber("supportLength")>0&&(parameterNumber("bore")>=parameterNumber("supportDiameter")||parameterNumber("supportDiameter")>parameterNumber("outside")))throw "invalid support nose section";
    if(parameterNumber("supportDiameter")<=0||parameterNumber("supportLength")>parameterNumber("height")-parameterNumber("hobSeatLength"))throw "invalid arbor nose seat";
    if(parameterNumber("hobSeatLength")<=0||parameterNumber("hobSeatLength")>=parameterNumber("height"))throw "hob seat exceeds arbor";
    addConnector("gaugeLine",Mount,Solids.axial(0,0,0));
    var centre=parameterNumber("hobCentreOffset")==0?parameterNumber("height")/2:parameterNumber("hobCentreOffset");
    if(centre<parameterNumber("hobSeatLength")/2||centre+parameterNumber("hobSeatLength")/2>parameterNumber("height")-parameterNumber("supportLength"))throw "hob seat exceeds usable arbor span";
    addConnector("hobCentre",Axis,Solids.axial(0,0,centre));
    addConnector("support",Axis,Solids.axial(0,0,parameterNumber("height")));
  }
  public function interfaceName():String return "ASSUMED-TF20";
  override public function componentType():Null<ComponentType>return recipeType();


  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    {
      var length=parameterNumber("supportLength"),height=parameterNumber("height");

      var sections=[cadkit.modeling.Part.revolve([{r:0.0,z:-20.0},{r:8.0,z:-20.0},{r:10.0,z:0.0},{r:20.0,z:0.0},{r:20.0,z:4.0},{r:0.0,z:4.0}]),cadkit.modeling.Part.cylinderSpan(parameterNumber("outside")/2,0,height-length)];
      if(length>0)sections.push(cadkit.modeling.Part.cylinderSpan(parameterNumber("supportDiameter")/2,height-length,height));
      var body=Solids.union(sections);
      var tools=[cadkit.modeling.Part.cylinderSpan(3,-20.1,-8),cadkit.modeling.Part.cylinderSpan(2,0,4.1,14,0),cadkit.modeling.Part.cylinderSpan(2,0,4.1,-14,0)];
      if(parameterNumber("bore")>0)tools.push(cadkit.modeling.Part.cylinderSpan(parameterNumber("bore")/2,-20.1,height+0.1));
      return Solids.cut(body,tools);
    }
}
