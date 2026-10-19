package machinekit.routing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class tool length setter. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class ToolLengthSetter extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("diameter", 40),
    P.length("height", 60),
    P.length("stroke", 5),
    P.quantity("repeatability", 0.005, "mm")
  ];
  public function new(?v:ComponentValues) {
    super("tool-length-setter", "tool length setter", "steel", inputs(), v);
    positive(["diameter", "height", "stroke", "repeatability"]);
    require(n("stroke") < n("height"), "setter stroke exceeds height");
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("trigger", Axis, triggerFrame());
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.routing.tool-length-setter",
      inputs(),
      v -> new ToolLengthSetter(v)
    );
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.named(Part.cylinder(n("diameter") / 2, n("height")), "setter");
  }
  public function triggerFrame():materia.assembly.AssemblyRecord.AssemblyFrame return Solids.axial(
    0,
    0,
    n("height") - n("stroke") / 2
  );
  override public function io():MachineIo return {inputs:["trigger"], outputs:["air_blast"], analogOutputs:[]};

}
