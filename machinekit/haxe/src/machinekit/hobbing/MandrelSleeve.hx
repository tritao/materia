package machinekit.hobbing;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
/** Separate replaceable sleeve envelope; pressure deformation is NOT solved. */
class MandrelSleeve extends MachineComponent {
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.quantity("outside",59,"mm"),P.quantity("wall",1.5,"mm"),P.quantity("height",16,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.mandrel-sleeve",parameters(),v->new MandrelSleeve(v));return recipe;
  }
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);super("MANDREL-SLEEVE","Hydraulic mandrel sleeve (assumed)","steel",false,v);
    if(parameterNumber("wall")<=0||parameterNumber("outside")<=2*parameterNumber("wall")||parameterNumber("height")<=0)throw "invalid mandrel sleeve";
    addConnector("mount",Mount,Solids.axial(0,0,0));addFacet(new EngineeringMetadata(v));addFacet(machinekit.motion.PrecisionErrors.fromValues(v,[]));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part return Solids.cut(
    cadkit.modeling.Part.cylinderSpan(parameterNumber("outside")/2,0,parameterNumber("height")),
    [cadkit.modeling.Part.cylinderSpan(parameterNumber("outside")/2-parameterNumber("wall"),-0.1,parameterNumber("height")+0.1)]);
}
