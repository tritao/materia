package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class hinged spring-preloaded pinion mount. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class PreloadedPinionMount extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("width", 100),
    P.length("length", 140),
    P.length("thickness", 15),
    P.length("leverArm", 100),
    P.length("springArm", 70),
    P.quantity("springForce", 700, "N"),
    P.quantity("springRate", 40, "N/mm"),
    P.length("stroke", 4)
  ];
  public function new(?v:ComponentValues) {
    super("preloaded-pinion-mount", "hinged spring-preloaded pinion mount", "steel", inputs(), v);
    positive(["width", "length", "thickness", "leverArm", "springArm", "springForce", "springRate", "stroke"]);
    require(n("leverArm") < n("length") && n("springArm") < n("length"), "lever arms must lie within plate");
    addConnector("hinge", Axis, Solids.axial(0, 0, 0));
    addConnector("pinion", Axis, Solids.axial(0, n("leverArm"), n("thickness")));
    addConnector("spring", Mount, Solids.axial(0, n("springArm"), n("thickness")));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.transmission.preloaded-pinion-mount",
      inputs(),
      v -> new PreloadedPinionMount(v)
    );
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.named(Part.box(n("width"), n("length"), n("thickness"), Center, Min), "hinged-plate");
  }
  /** N at the pinion for spring extension measured in mm. */
  public function preload(extension:Float = 0):Float {
    require(Math.isFinite(extension) && extension >= 0 && extension <= n("stroke"), "spring extension outside stroke");
    return(n("springForce") + n("springRate") * extension) * n("springArm") / n("leverArm");
  }
  public function resists(separatingForce:Float):Bool {
    require(Math.isFinite(separatingForce) && separatingForce >= 0, "invalid separating force");
    return preload() >= separatingForce;
  }
}
