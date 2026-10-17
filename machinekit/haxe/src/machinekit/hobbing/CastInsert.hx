package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Cast-in steel insert. Local +Z is the finished plane normal; its top is the precision mount. */
class CastInsert extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    P.length("width",35),P.length("depth",400),P.length("height",20),P.length("positionTolerance",0.01),
    P.choice("role",["rail seat","table plate","column plate","threaded bush","lifting point"],"rail seat"),
    P.text("finish","ground after casting")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.cast-in-steel-insert",parameters(),v->new CastInsert(v));return recipe;
  }
  public function new(?values:ComponentValues){
    var v=PrecisionPart.resolve(parameters(),values);
    super("CAST-IN-INSERT","Cast-in steel "+v.token("role")+", "+v.token("finish"),"steel",v,false,[]);
    if(n("positionTolerance")<=0||v.token("finish")!="ground after casting")throw "insert requires position tolerance and ground finish";
  }
  override public function setMaterial(spec:String):Void {
    if(spec!="steel")throw "cast-in precision insert requires steel";super.setMaterial(spec);
  }
  override public function componentType():Null<ComponentType>return recipeType();
}
