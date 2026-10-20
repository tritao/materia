package machinekit.control;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class latching emergency stop with two NC contacts. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class EmergencyStopButton extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("headDiameter", 40),
    P.length("panelHole", 22),
    P.length("headHeight", 25),
    P.length("contactDepth", 45)
  ];
  public function new(?v:ComponentValues) {
    super("emergency-stop-button", "latching emergency stop with two NC contacts", "steel", inputs(), v);
    positive(["headDiameter", "panelHole", "headHeight", "contactDepth"]);
    require(n("panelHole") < n("headDiameter"), "panel bore exceeds head");
    addConnector("panel", Mount, Solids.axial(0, 0, 0));
    for (i in 1...3) addPort({
      name: "nc" + i,
      kind: Signal,
      role: Passive,
      iface: Plug("dry-contact", 2),
      required: true
    }
    );
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.control.emergency-stop-button",
      inputs(),
      v -> new EmergencyStopButton(v)
    );
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.union([
      Solids.named(Part.cylinder(n("headDiameter") / 2, n("headHeight")), "mushroom"),
      Solids.named(
        Part.cylinderSpan(
          n("panelHole") / 2,
          -n("contactDepth"),
          0
        ),
        "contact-block"
      )
    ]);
  }

}
