package machinekit.hmi;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as R;

/**
 * A panel rotary switch with a knob: a threaded bush through the panel with a nut, a round switch body
 * behind, a knob in front. `positions` says how many throws it has (3 for the step switch, 12
 * for the axis switch). All sizes are assumed until a real switch is measured: knob 25, bush 10, body 30 by 25.
 */
class RotarySwitch extends HmiPart {
  static var recipe:Null<ComponentType>;

  static function inputs():Array<ComponentParameter> return [
    HmiPart.basisParameter("assumed"),
    R.count("positions", 12),
    R.length("knobDiameter", 25), R.length("knobHeight", 18),
    R.length("bushDiameter", 10), R.length("panelThickness", 3), R.length("nutDiameter", 14), R.length("nutThickness", 2),
    R.length("bodyDiameter", 30), R.length("behindDepth", 25),
    R.scalar("massGrams", 25, 1)
  ];

  public function new(?v:ComponentValues) {
    super("hmi-rotary-switch", "rotary switch with knob, " + (v == null || v.get("positions") == null ? 12 : v.integer("positions")) + " positions", "plastic", inputs(), v);
    positive(["knobDiameter", "knobHeight", "bushDiameter", "panelThickness", "nutDiameter", "nutThickness", "bodyDiameter", "behindDepth", "massGrams"]);
    require(count("positions") >= 2 && count("positions") <= 24, "two to 24 positions");
    require(n("bushDiameter") < n("nutDiameter") && n("nutDiameter") < n("knobDiameter"), "the nut is wider than the bush and narrower than the knob");
    require(n("behindDepth") > n("panelThickness") + n("nutThickness"), "the body has room behind the nut");
    addConnector("panel", Mount, Solids.axial(0, 0, 0));
    declareMass(n("massGrams") / 1000.0, new Vector(0, 0, (aboveFace() - behindFace()) / 2));
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.hmi.rotary-switch", inputs(), v -> new RotarySwitch(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();

  override public function panelCutout():Footprint return Circle(n("bushDiameter") + 0.2);
  override public function panelKeepOut():Footprint return Circle(n("knobDiameter"));
  override public function aboveFace():Float return n("knobHeight");
  override public function behindFace():Float return n("behindDepth");
  override public function rearBody():Footprint return Circle(n("bodyDiameter"));
  override public function massGrams():Float return n("massGrams");

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var r = n("knobDiameter") / 2, k = n("knobHeight");
    var knob = Part.revolve([{r: 0.0, z: 0.0}, {r: r - 1.0, z: 0.0}, {r: r, z: 1.0}, {r: r, z: k - 2.0}, {r: r - 2.0, z: k}, {r: 0.0, z: k}]);
    var parts:Array<Part> = [Solids.named(knob, "knob")];
    var t = n("panelThickness"), nut = n("nutThickness");
    parts.push(Solids.named(Part.cylinderSpan(n("bushDiameter") / 2, -t - nut, 0.0), "bush"));
    parts.push(Solids.named(Part.cylinderSpan(n("nutDiameter") / 2, -t - nut, -t), "nut"));
    parts.push(Solids.named(Part.cylinderSpan(n("bodyDiameter") / 2, -n("behindDepth"), -t - nut), "body"));
    return Solids.union(parts);
  }
}
