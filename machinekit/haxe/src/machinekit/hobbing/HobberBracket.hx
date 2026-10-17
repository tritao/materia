package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Assumed steel carriage/support envelope; structural adequacy awaits Phase D. */
class HobberBracket extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    P.length("width",160),P.length("depth",50),P.length("height",100),P.flag("cantilever",false)]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.hobber-bracket",parameters(),v->new HobberBracket(v));return recipe;
  }
  public function new(?values:ComponentValues)super("HOBBER-BRACKET","Steel hobber carriage/support envelope","steel",PrecisionPart.resolve(parameters(),values),false,[]);
  override public function componentType():Null<ComponentType>return recipeType();
  override public function geometry(detail:ComponentDetail=Preview):Part {
    if(!values().boolean("cantilever"))return super.geometry(detail);
    function box(w:Float,d:Float,h:Float,x:Float,y:Float,z:Float):Part {
      var b=Part.box(w,d,h),p=b.translated(new Vector(x,y,z));b.close();return p;
    }
    return Solids.union([box(280,60,25,100,160,120),box(40,60,140,220,160,0),box(40,160,40,220,80,0)]);
  }
}
