package machinekit.motion;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Angular-contact precision bearing. Reference row is Assumed; dimensions are simplified envelopes. */
class AngularContactBearing extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("outside",62,"mm"),
    PrecisionPart.quantity("bore",25,"mm"),
    PrecisionPart.quantity("height",17,"mm"),
    P.text("precisionClass","P4"),

    PrecisionPart.quantity("contactAngle",25*Math.PI/180,"rad"),
    PrecisionPart.quantity("radialRunout",0.002,"mm"),
    PrecisionPart.quantity("axialRunout",0.002,"mm"),
    PrecisionPart.quantity("momentStiffness",150000,"N m/rad")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.motion.angular-contact-bearing",parameters(),v -> new AngularContactBearing(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Angular-contact precision bearing","AC-BEARING-25-62-17",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("AC-BEARING-25-62-17","Angular-contact precision bearing (reference envelope)","bearing steel",v,true,[{name:"radialRunout",unit:"mm"},{name:"axialRunout",unit:"mm"}]);
    if(n("contactAngle")<=0||n("contactAngle")>=Math.PI/2||n("momentStiffness")<=0)throw "invalid bearing ratings";
  }
  override public function componentType():Null<ComponentType>return recipeType();
  public var radialRunout(get,never):Float;
  function get_radialRunout():Float return n("radialRunout");
  public var axialRunout(get,never):Float;
  function get_axialRunout():Float return n("axialRunout");
  public var momentStiffness(get,never):Float;
  function get_momentStiffness():Float return n("momentStiffness");

}
