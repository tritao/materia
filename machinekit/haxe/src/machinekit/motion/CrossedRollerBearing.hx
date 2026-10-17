package machinekit.motion;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Crossed-roller precision bearing. Reference row is Assumed; dimensions are simplified envelopes. */
class CrossedRollerBearing extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("outside",120,"mm"),
    PrecisionPart.quantity("bore",80,"mm"),
    PrecisionPart.quantity("height",16,"mm"),
    P.text("precisionClass","P4"),
    PrecisionPart.quantity("radialRunout",0.002,"mm"),
    PrecisionPart.quantity("axialRunout",0.002,"mm"),
    PrecisionPart.quantity("momentStiffness",1000000,"N m/rad")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.motion.crossed-roller-bearing",parameters(),v -> new CrossedRollerBearing(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Crossed-roller precision bearing","CROSSED-ROLLER-80-120-16",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("CROSSED-ROLLER-80-120-16","Crossed-roller precision bearing (reference envelope)","bearing steel",v,true,[{name:"radialRunout",unit:"mm"},{name:"axialRunout",unit:"mm"}]);
    if(n("momentStiffness")<=0)throw "bearing stiffness must be positive";
  }
  override public function componentType():Null<ComponentType>return recipeType();
  public var radialRunout(get,never):Float;
  function get_radialRunout():Float return n("radialRunout");
  public var axialRunout(get,never):Float;
  function get_axialRunout():Float return n("axialRunout");
  public var momentStiffness(get,never):Float;
  function get_momentStiffness():Float return n("momentStiffness");

}
