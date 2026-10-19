package machinekit.hobbing;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
/** Assumed precision needle/roller radial support. No flanges or axial locating
    stiffness: the inner race slides +/- axialFloat on the arbor journal. */
class FloatingRadialBearing extends MachineComponent {
  static var recipe:Null<ComponentType>;
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.length("outside",32),P.length("bore",15),P.length("height",9),P.length("axialFloat",1),
    P.quantity("radialStiffness",100000,"N/mm"),P.quantity("radialClearance",0,"mm")]);
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.floating-radial-bearing",parameters(),v->new FloatingRadialBearing(v));return recipe;
  }
  public function new(?values:ComponentValues){var v=P.resolve(parameters(),values);
    super("FLOATING-RADIAL-15-32","Assumed preloaded radial roller support; axially floating +/-1 mm, not a second thrust locator","bearing steel",false,v);
    ComponentEnvelopes.validateAnnular(v);
    if(v.number("axialFloat")<=0||v.number("radialStiffness")<=0||v.number("radialClearance")<0)throw "invalid floating radial support";addFacet(new EngineeringMetadata(v));addConnector("mount",Mount,Solids.axial(0,0,0));addConnector("axis",Axis,Solids.axial(0,0,0));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part return ComponentEnvelopes.annular(values());
}
