package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hob outboard support housing. Reference row is Assumed; dimensions are simplified envelopes. */
class OutboardSupport extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("width",60,"mm"),
    PrecisionPart.quantity("depth",60,"mm"),
    PrecisionPart.quantity("height",75,"mm"),
    PrecisionPart.quantity("seatDiameter",32,"mm"),
    PrecisionPart.quantity("shaftDiameter",16,"mm"),
    PrecisionPart.quantity("radialRunout",0.002,"mm"),
    PrecisionPart.quantity("radialStiffness",100000000,"N/m")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.outboard-support",parameters(),v -> new OutboardSupport(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Hob outboard support housing","HOB-OUTBOARD-16",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("HOB-OUTBOARD-16","Hob outboard support housing (reference envelope)","steel",v,false,[{name:"radialRunout",unit:"mm"}]);
    if(n("shaftDiameter")<=0||n("seatDiameter")<=n("shaftDiameter")||n("seatDiameter")>=Math.min(n("width"),n("depth"))||n("radialStiffness")<=0)throw "invalid support seat";
    addConnector("bearing",Axis,Solids.axial(0,0,n("height")/2));
  }
  override public function componentType():Null<ComponentType>return recipeType();

  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    return Solids.cut(super.geometry(detail),[cadkit.modeling.Part.cylinderSpan(n("seatDiameter")/2,-0.1,n("height")+0.1)]);
}
