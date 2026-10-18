package machinekit.motion;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Angular-contact precision bearing. Reference row is Assumed; dimensions are simplified envelopes. */
class AngularContactBearing extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.quantity("outside",62,"mm"),
    P.quantity("bore",25,"mm"),
    P.quantity("height",17,"mm"),
    P.text("precisionClass","P4"),

    P.quantity("contactAngle",25*Math.PI/180,"rad"),
    P.quantity("radialRunout",0.002,"mm"),
    P.quantity("axialRunout",0.002,"mm"),
    P.quantity("momentStiffness",150000,"N m/rad")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.motion.angular-contact-bearing",parameters(),v -> new AngularContactBearing(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return EngineeringMetadata.assumedCatalog("Angular-contact precision bearing","AC-BEARING-25-62-17",parameters(),"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)");
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("AC-BEARING-25-62-17","Angular-contact precision bearing (reference envelope)","bearing steel",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[{name:"radialRunout",unit:"mm"},{name:"axialRunout",unit:"mm"}]));
    if(parameterNumber("contactAngle")<=0||parameterNumber("contactAngle")>=Math.PI/2||parameterNumber("momentStiffness")<=0)throw "invalid bearing ratings";
  }
  override public function componentType():Null<ComponentType>return recipeType();
  public var radialRunout(get,never):Float;
  function get_radialRunout():Float return parameterNumber("radialRunout");
  public var axialRunout(get,never):Float;
  function get_axialRunout():Float return parameterNumber("axialRunout");
  public var momentStiffness(get,never):Float;
  function get_momentStiffness():Float return parameterNumber("momentStiffness");

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    return ComponentEnvelopes.annular(values());
}
