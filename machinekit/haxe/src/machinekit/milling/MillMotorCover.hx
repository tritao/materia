package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Removable three-sided motor hood, open below and toward the coupling (+Y).
 * Mounting screw details depend on the selected motor cage; 2 mm sheet is assumed.
 */
class MillMotorCover extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return [
    P.length("width", 118),
    P.length("depth", 220),
    P.length("height", 110)
  ];
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.motor-cover",
      parameters(),
      v -> new MillMotorCover(v)
    ).numbered(_ -> "MILL-MOTOR-HOOD"));
  }
  public function new(?values:ComponentValues) {
    var v = P.resolve(parameters(), values);
    if (Math.min(v.number("width"),
      Math.min(v.number("depth"), v.number("height"))) <= 4) throw "Motor hood has no interior";
    super("MILL-MOTOR-HOOD", "Removable motor hood, mounting detail pending", "painted steel", false, v);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var w = parameterNumber("width"), d = parameterNumber("depth"), h = parameterNumber("height");
    return Solids.union([
      Solids.named(Part.box(w, d, 2).translated(new Vector(0, 0, h - 2)), "top"),
      Solids.named(Part.box(w, 2, h).translated(new Vector(0, -d / 2 + 1, 0)), "front"),
      Solids.named(Part.box(2, d, h).translated(new Vector(-w / 2 + 1, 0, 0)), "left"),
      Solids.named(Part.box(2, d, h).translated(new Vector(w / 2 - 1, 0, 0)), "right")
    ]);
  }
}
