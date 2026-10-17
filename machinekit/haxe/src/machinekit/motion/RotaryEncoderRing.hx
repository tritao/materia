package machinekit.motion;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Rotary ring encoder. Reference row is Assumed; dimensions are simplified envelopes. */
class RotaryEncoderRing extends PrecisionPart implements machinekit.motion.EncoderPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("outside",130,"mm"),
    PrecisionPart.quantity("bore",100,"mm"),
    PrecisionPart.quantity("height",8,"mm"),
    P.count("countsPerRevolution",8388608),
    PrecisionPart.quantity("angularAccuracy",2*Math.PI/(180*3600),"rad"),
    PrecisionPart.quantity("eccentricity",0.001,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.motion.rotary-encoder-ring",parameters(),v -> new RotaryEncoderRing(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Rotary ring encoder","RING-ENCODER-23BIT",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("RING-ENCODER-23BIT","Rotary ring encoder (reference envelope)","steel",v,true,[{name:"angularAccuracy",unit:"rad"},{name:"eccentricity",unit:"mm"}]);
    if(count("countsPerRevolution")<1)throw "encoder needs positive counts";
  }
  override public function componentType():Null<ComponentType>return recipeType();
  public var angularAccuracy(get,never):Float;
  function get_angularAccuracy():Float return n("angularAccuracy");
  public var eccentricity(get,never):Float;
  function get_eccentricity():Float return n("eccentricity");
  public function encoder(id:String,joint:String):materia.assembly.AssemblyDefinition.AssemblyEncoder
    return {id:id,joint:joint,kind:"absolute",counts:count("countsPerRevolution")};
  public function quantum():Float return 2*Math.PI/count("countsPerRevolution");
}
