package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Static rotary clamp. Reference row is Assumed; dimensions are simplified envelopes. */
class RotaryClamp extends PrecisionPart implements machinekit.motion.MotorBrakeSpec {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("outside",150,"mm"),
    PrecisionPart.quantity("bore",120,"mm"),
    PrecisionPart.quantity("height",10,"mm"),
    PrecisionPart.quantity("holdingTorque",100,"N m"),
    PrecisionPart.quantity("releaseTime",0.1,"s")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.rotary-clamp",parameters(),v -> new RotaryClamp(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Static rotary clamp","ROTARY-CLAMP-100NM",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("ROTARY-CLAMP-100NM","Static rotary clamp (reference envelope)","steel",v,true,[]);
    if(n("holdingTorque")<=0)throw "clamp needs holding torque";
  }
  override public function componentType():Null<ComponentType>return recipeType();
  override public function motorBrakeSpec():Null<machinekit.motion.MotorBrakeSpec>return this;
  public function holdingTorque():Float return n("holdingTorque");
  public function releaseDelaySeconds():Float return n("releaseTime");
  public function applicationDelaySeconds():Float return n("releaseTime");

}
