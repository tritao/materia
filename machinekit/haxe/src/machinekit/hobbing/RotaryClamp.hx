package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Static rotary clamp. Reference row is Assumed; dimensions are simplified envelopes. */
class RotaryClamp extends MachineComponent implements machinekit.motion.MotorBrakeSpec {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.quantity("outside",150,"mm"),
    P.quantity("bore",120,"mm"),
    P.quantity("height",10,"mm"),
    P.quantity("holdingTorque",100,"N m"),
    P.quantity("releaseTime",0.1,"s")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.rotary-clamp",parameters(),v -> new RotaryClamp(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return EngineeringMetadata.assumedCatalog("Static rotary clamp","ROTARY-CLAMP-100NM",parameters(),"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)");
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("ROTARY-CLAMP-100NM","Static rotary clamp (reference envelope)","steel",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[]));
    if(parameterNumber("holdingTorque")<=0)throw "clamp needs holding torque";
  }
  override public function componentType():Null<ComponentType>return recipeType();
  override public function motorBrakeSpec():Null<machinekit.motion.MotorBrakeSpec>return this;
  public function holdingTorque():Float return parameterNumber("holdingTorque");
  public function releaseDelaySeconds():Float return parameterNumber("releaseTime");
  public function applicationDelaySeconds():Float return parameterNumber("releaseTime");

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    return ComponentEnvelopes.annular(values());
}
