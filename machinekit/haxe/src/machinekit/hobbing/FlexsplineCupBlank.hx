package machinekit.hobbing;

import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;

/** One-piece cup blank, open/toothed end at Z=0, closed end above it.
    Tooth band and bore use the existing work datum; dimensions are assumed. */
class FlexsplineCupBlank extends MachineComponent {
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.quantity("outside",62,"mm"),P.quantity("bore",58.05,"mm"),P.quantity("bodyOutside",59.25,"mm"),
    P.quantity("bandWidth",8,"mm"),P.quantity("height",40,"mm"),P.quantity("endThickness",2,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.flexspline-cup-blank",parameters(),v->new FlexsplineCupBlank(v));
    return recipe;
  }
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("FLEXSPLINE-CUP-BLANK","One-piece flexspline cup blank (assumed)","steel",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    if(parameterNumber("bore")<=0||parameterNumber("bodyOutside")<=parameterNumber("bore")||parameterNumber("outside")<parameterNumber("bodyOutside")
      ||parameterNumber("bandWidth")<=0||parameterNumber("endThickness")<=0||parameterNumber("height")<=parameterNumber("bandWidth")+parameterNumber("endThickness"))throw "invalid flexspline cup dimensions";
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addFacet(new EngineeringMetadata(v));addFacet(machinekit.motion.PrecisionErrors.fromValues(v,[]));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var tip=parameterNumber("outside")/2,body=parameterNumber("bodyOutside")/2,inner=parameterNumber("bore")/2,
      band=parameterNumber("bandWidth"),height=parameterNumber("height"),end=parameterNumber("endThickness");
    return Part.revolve([{r:inner,z:0.0},{r:tip,z:0.0},{r:tip,z:band},{r:body,z:band},
      {r:body,z:height},{r:0.0,z:height},{r:0.0,z:height-end},{r:inner,z:height-end}]);
  }
}
