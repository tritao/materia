package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Assumed steel carriage/support envelope; structural adequacy awaits Phase D. */
class HobberBracket extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.length("width",160),P.length("depth",50),P.length("height",100),P.flag("cantilever",false)]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.hobber-bracket",parameters(),v->new HobberBracket(v));return recipe;
  }
  public function new(?values:ComponentValues){
    var v=P.resolve(parameters(),values);
    super("HOBBER-BRACKET","Steel hobber carriage/support envelope","steel",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateBlock(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[]));
  }
  override public function componentType():Null<ComponentType>return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part {
    if(!values().boolean("cantilever"))return ComponentEnvelopes.block(values());
    function box(w:Float,d:Float,h:Float,x:Float,y:Float,z:Float):Part {
      var b=Part.box(w,d,h),p=b.translated(new Vector(x,y,z));b.close();return p;
    }
    return Solids.union([box(280,60,25,100,160,120),box(40,60,140,220,160,0),box(40,160,40,220,80,0)]);
  }
}
