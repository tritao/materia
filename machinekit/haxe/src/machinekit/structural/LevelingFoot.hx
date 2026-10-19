package machinekit.structural;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;
/** Rigid swivel-pad and nominal threaded stem envelope. Recessed mounts use
 * separate load plate and nuts; no elastomer or supplier rating claimed. */
class LevelingFoot extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return [P.length("height",47),P.length("padDiameter",100),P.length("stemDiameter",24),P.length("recessDepth",0)];
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {if(recipe==null)recipe=new ComponentType("machinekit.structural.leveling-foot",parameters(),v->new LevelingFoot(v));return recipe;}
  public function new(?overrides:ComponentValues){var v=P.resolve(parameters(),overrides);super("LEVELING-FOOT-M24-ASSUMED-R"+v.number("recessDepth"),"Rigid swivel levelling foot, nominal M24x3, assumed 15 kN design target","steel",false,v);
    if(v.number("height")<30||v.number("padDiameter")<2*v.number("stemDiameter")||v.number("stemDiameter")<=0)throw "Invalid levelling foot";
    addConnector("mount",Mount,Solids.axial(0,0,v.number("height")+v.number("recessDepth")));addConnector("floor",Mount,Solids.axial(0,0,0));
  }
  override public function componentType():Null<ComponentType>return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part return Solids.union([Part.cylinderSpan(parameterNumber("padDiameter")/2,0,15),
    Part.cylinderSpan(parameterNumber("stemDiameter")/2,15,parameterNumber("height")+parameterNumber("recessDepth")+(parameterNumber("recessDepth")>0?25:0)),Part.cylinderSpan(parameterNumber("recessDepth")>0?parameterNumber("stemDiameter")/2:20,parameterNumber("height")-8,parameterNumber("height"))]);
}
