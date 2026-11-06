package machinekit.milling;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Bearing riser: Z up, +Y toward motor. Rear base bolts leave room for front motor anchors. */
class MillScrewPedestal extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();

  public static function parameters():Array<ComponentParameter> return [
    P.length("width", 100),
    P.length("depth", 25),
    P.length("height", 43),
    P.flag("motorAnchors", true)
  ];

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.screw-pedestal",
      parameters(),
      v -> new MillScrewPedestal(v)
    ).numbered(_ -> "MILL-SCREW-PEDESTAL"));
  }

  public function new(?values:ComponentValues) {
    var v = P.resolve(parameters(), values);
    if (v.number("width") < 80
      || v.number("height") < 15 || v.number("depth") < 20) throw "Screw pedestal is too small";
    super("MILL-SCREW-PEDESTAL", "Machined bearing and motor mounting pedestal", "steel", false, v);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
  }

  override public function componentType():Null<ComponentType> return recipeType();

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var w = parameterNumber("width"), d = parameterNumber("depth"), h = parameterNumber("height");
    var tools = [];
    for (k => x in [-23.0, 23.0]) tools.push(Solids.named(Part.cylinderSpan(2.5, Math.max(-1, h - 16), h + 1, x, 0), "support-tap" + (k + 1)));
    for (k => s in [-1, 1]) {
      var x = s * (w / 2 - 8);
      tools.push(Solids.named(Part.cylinderSpan(3.3, -1, h + 1, x, -d / 2 + 6), "anchor" + (k + 1)));
      if (values().boolean("motorAnchors")) tools.push(Solids.named(Part.cylinderAlongY(2.5, d / 2 - 12, d / 2 + 1, x, h / 2), "motor-anchor" + (k + 1)));
    }
    return Solids.cut(Part.box(w, d, h), tools);
  }
}
