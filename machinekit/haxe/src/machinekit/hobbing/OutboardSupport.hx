package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hob outboard support housing. Reference row is Assumed; dimensions are simplified envelopes. */
class OutboardSupport extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.quantity("width",60,"mm"),
    P.quantity("depth",60,"mm"),
    P.quantity("height",75,"mm"),
    P.quantity("seatDiameter",32,"mm"),
    P.quantity("shaftDiameter",16,"mm"),
    P.quantity("seatLength",0,"mm"),
    P.quantity("radialRunout",0.002,"mm"),
    P.quantity("radialStiffness",100000000,"N/m")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.outboard-support",parameters(),v -> new OutboardSupport(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return EngineeringMetadata.assumedCatalog("Hob outboard support housing","HOB-OUTBOARD-16",parameters(),"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)");
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("HOB-OUTBOARD-16","Hob outboard support housing (reference envelope)","steel",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateBlock(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[{name:"radialRunout",unit:"mm"}]));
    if(parameterNumber("seatLength")>=parameterNumber("height"))throw "invalid bearing seat length";
    if(parameterNumber("shaftDiameter")<=0||parameterNumber("seatDiameter")<=parameterNumber("shaftDiameter")||parameterNumber("seatDiameter")>=Math.min(parameterNumber("width"),parameterNumber("depth"))||parameterNumber("radialStiffness")<=0)throw "invalid support seat";
    addConnector("bearing",Axis,Solids.axial(0,0,parameterNumber("height")/2));
  }
  override public function componentType():Null<ComponentType>return recipeType();

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    return parameterNumber("seatLength")==0?Solids.cut(ComponentEnvelopes.block(values()),[cadkit.modeling.Part.cylinderSpan(parameterNumber("seatDiameter")/2,-0.1,parameterNumber("height")+0.1)]):Solids.cut(ComponentEnvelopes.block(values()),[cadkit.modeling.Part.cylinderSpan(parameterNumber("shaftDiameter")/2+0.1,-0.1,parameterNumber("height")+0.1),
      cadkit.modeling.Part.cylinderSpan(parameterNumber("seatDiameter")/2,(parameterNumber("height")-parameterNumber("seatLength"))/2,(parameterNumber("height")+parameterNumber("seatLength"))/2)]);
}
