package machinekit.enclosure;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;

/** Removable stepped toe clamp. Local +X points over the panel; M6 clearance bore is explicit. */
class TrayClamp extends MachineComponent {
  public static inline final WIDTH = 20.0;
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public final height:Float;
  public function new(height:Float = TraySeat.TOP) {
    if (!(height > 0) || !Math.isFinite(height)) throw "Clamp needs a finite positive seat height";
    super("TRAY-TOE-CLAMP-M6-" + height, "Stepped laminate tray toe clamp, M6 clearance", "steel");
    this.height = height;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("head", Mount, Solids.axial(0, 0, height + 4));
  }
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.enclosure.tray-toe-clamp",
      [ComponentRecipeSupport.length("height", TraySeat.TOP)],
      v -> new TrayClamp(v.number("height")),
      true
    ));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setNumber("height", height).setToken(
    "material",
    materialSpec()
  );
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var rawHeel = Part.box(20, WIDTH, height), rawToe = Part.box(40, WIDTH, 4);
    var heel = Solids.named(rawHeel.translated(new Vector(-5, 0, 0)), "heel"), toe = Solids.named(rawToe.translated(new Vector(5, 0, height)), "toe");
    rawHeel.close();
    rawToe.close();
    return Solids.cut(Solids.union([heel, toe]), [Solids.named(Part.cylinderSpan(3.3, -1, height + 5), "bolt")]);
  }
}
