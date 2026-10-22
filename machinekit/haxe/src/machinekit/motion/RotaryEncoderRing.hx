package machinekit.motion;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Rotary ring encoder. Reference row is Assumed: its outside, bore and height are not a product's. */
class RotaryEncoderRing extends MachineComponent implements machinekit.motion.EncoderPart {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.quantity("outside",130,"mm"),
    P.quantity("bore",100,"mm"),
    P.quantity("height",8,"mm"),
    P.count("countsPerRevolution",8388608),
    P.quantity("angularAccuracy",2*Math.PI/(180*3600),"rad"),
    P.quantity("eccentricity",0.001,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.motion.rotary-encoder-ring",parameters(),v -> new RotaryEncoderRing(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return EngineeringMetadata.assumedCatalog("Rotary ring encoder","RING-ENCODER-23BIT",parameters(),"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)");
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("RING-ENCODER-23BIT","Rotary ring encoder (reference envelope)","steel",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[{name:"angularAccuracy",unit:"rad"},{name:"eccentricity",unit:"mm"}]));
    if(parameterCount("countsPerRevolution")<1)throw "encoder needs positive counts";
  }
  override public function componentType():Null<ComponentType>return recipeType();
  public var angularAccuracy(get,never):Float;
  function get_angularAccuracy():Float return parameterNumber("angularAccuracy");
  public var eccentricity(get,never):Float;
  function get_eccentricity():Float return parameterNumber("eccentricity");
  public function encoder(id:String,joint:String):materia.assembly.AssemblyDefinition.AssemblyEncoder
    return {id:id,joint:joint,kind:"absolute",counts:parameterCount("countsPerRevolution")};
  public function quantum():Float return 2*Math.PI/parameterCount("countsPerRevolution");
  override public function hasGeometry():Bool return true;
  /** Hub and graduated scale band (`RingParts.encoder`). Envelope detail is the annulus, which also sets the mass. */
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    return detail==Envelope ? ComponentEnvelopes.annular(values()) : RingParts.encoder(parameterNumber("outside"),parameterNumber("bore"),parameterNumber("height"));
  override public function massGeometry():Null<cadkit.modeling.Part> return ComponentEnvelopes.annular(values());
}
