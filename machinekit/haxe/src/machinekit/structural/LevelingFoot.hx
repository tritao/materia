package machinekit.structural;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;
/** Three-point rigid support, swivel-pad envelope. No elastomer or supplier rating claimed. */
class LevelingFoot extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return [P.length("height",47),P.length("padDiameter",100),P.length("stemDiameter",24)];
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {if(recipe==null)recipe=new ComponentType("machinekit.structural.leveling-foot",parameters(),v->new LevelingFoot(v));return recipe;}
  public function new(?overrides:ComponentValues){var v=P.resolve(parameters(),overrides);super("LEVELING-FOOT-M24-ASSUMED","Rigid swivel levelling foot, assumed 15 kN design target","steel",false,v);
    if(v.number("height")<30||v.number("padDiameter")<2*v.number("stemDiameter")||v.number("stemDiameter")<=0)throw "Invalid levelling foot";
    addConnector("mount",Mount,Solids.axial(0,0,v.number("height")));addConnector("floor",Mount,Solids.axial(0,0,0));
  }
  override public function componentType():Null<ComponentType>return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part return Solids.union([Part.cylinderSpan(parameterNumber("padDiameter")/2,0,15),
    Part.cylinderSpan(parameterNumber("stemDiameter")/2,15,parameterNumber("height")),Part.cylinderSpan(20,parameterNumber("height")-8,parameterNumber("height"))]);
}
