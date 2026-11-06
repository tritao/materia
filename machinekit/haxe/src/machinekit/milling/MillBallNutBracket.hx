package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.BallNut;

/** SFU1605 flange bracket, shaft +Z, up +Y. Optional top foot attaches under a table. */
class MillBallNutBracket extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();

  public static function parameters():Array<ComponentParameter> return [P.flag("tableFoot", false)];

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.ball-nut-bracket",
      parameters(),
      v -> new MillBallNutBracket(v)
    ).numbered(_ -> "MILL-BALL-NUT-BRACKET"));
  }

  public function new(?values:ComponentValues) {
    var v = P.resolve(parameters(), values);
    super("MILL-BALL-NUT-BRACKET", "SFU1605 flange bracket", "steel", false, v);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
  }

  override public function componentType():Null<ComponentType> return recipeType();

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var pieces = [Solids.named(Part.box(84, 60, 10), "plate")];
    if (values().boolean("tableFoot")) pieces.push(Solids.named(Part.box(84, 18, 40).translated(new Vector(0, 39, 0)), "foot"));
    var body = Solids.union(pieces), tools = [Solids.named(Part.cylinderSpan(9, -1, 11), "shaft-clearance")];
    for (k => p in new BallNut().boltPattern()) tools.push(Solids.named(Part.cylinderSpan(2.75, -1, 11, p.x, p.y), "flange-bolt" + (k + 1)));
    var anchor = 0;
    if (values().boolean("tableFoot")) for (x in [-25.0, 25.0]) tools.push(Solids.named(Part.cylinderAlongY(2.5, 29, 49, x, 20), "foot-bolt" + (++anchor)));
    else for (x in [-35.0, 35.0]) for (y in [-20.0, 20.0]) tools.push(Solids.named(Part.cylinderSpan(3.3, -1, 11, x, y), "anchor" + (++anchor)));
    return Solids.cut(body, tools);
  }
}
