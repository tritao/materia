package machinekit.pendant;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as R;

/**
 * A 22 mm twist-release emergency stop (XB2-ES542 style): a mushroom head on a collar, the stem through the
 * panel, a ring nut behind it, a latching base and one to three snap-on contact blocks across it. Defaults
 * are a seller's drawing of that family: 31 above the panel, 43 behind it (nut, base and a 24 mm block),
 * a 42 mm wide base, 29 mm blocks. The block row, the base's other dimension and the head profile are drawn
 * from photos. A compact maker's part (EAO Series 84, APEM EC Compact) is the same recipe with a thin base.
 */
class PendantEmergencyStop extends PendantPart {
  static var recipe:Null<ComponentType>;

  static function inputs():Array<ComponentParameter> return [
    PendantPart.basisParameter("stated"),
    R.length("headDiameter", 40), R.length("collarDiameter", 30), R.length("collarHeight", 9), R.length("headHeight", 22),
    R.length("panelHole", 22.3), R.length("panelThickness", 3),
    R.length("nutDiameter", 30), R.length("nutThickness", 5),
    R.length("baseWidth", 42), R.length("baseDepth", 30), R.length("baseLength", 11),
    R.length("blockWidth", 29), R.length("blockThickness", 14.8), R.length("blockLength", 24), R.count("blocks", 1),
    R.scalar("massGrams", 52.4, 1)
  ];

  public function new(?v:ComponentValues) {
    super("pendant-emergency-stop", "twist-release emergency stop with " + (v == null || v.get("blocks") == null ? 1 : v.integer("blocks")) + " NC contact block(s)", "plastic", inputs(), v);
    positive(["headDiameter", "collarDiameter", "collarHeight", "headHeight", "panelHole", "panelThickness", "nutDiameter", "nutThickness",
      "baseWidth", "baseDepth", "baseLength", "blockWidth", "blockThickness", "blockLength", "massGrams"]);
    require(n("collarDiameter") < n("headDiameter") && n("panelHole") < n("collarDiameter"), "the head is wider than its collar and the collar wider than the opening");
    require(count("blocks") >= 1 && count("blocks") <= 3, "one to three contact blocks");
    require(n("blockThickness") * count("blocks") <= n("baseDepth") + 1e-9 && n("blockWidth") <= n("baseWidth"), "the blocks fit the base");
    addConnector("panel", Mount, Solids.axial(0, 0, 0));
    declareMass(n("massGrams") / 1000.0, new Vector(0, 0, (aboveFace() - behindFace()) / 2));
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.pendant.emergency-stop", inputs(), v -> new PendantEmergencyStop(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();

  override public function panelCutout():Footprint return Circle(n("panelHole"));
  override public function panelKeepOut():Footprint return Circle(n("headDiameter"));
  override public function aboveFace():Float return n("collarHeight") + n("headHeight");
  override public function behindFace():Float return n("panelThickness") + n("nutThickness") + n("baseLength") + n("blockLength");
  override public function rearBody():Footprint return Rect(n("baseWidth"), n("baseDepth"));
  override public function massGrams():Float return n("massGrams");

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var above = aboveFace(), c = n("collarHeight"), h = n("headDiameter") / 2;
    var head = Part.revolve([
      {r: 0.0, z: 0.0}, {r: n("collarDiameter") / 2, z: 0.0}, {r: n("collarDiameter") / 2, z: c},
      {r: h - 1.5, z: c + 1.5}, {r: h, z: c + 4.0}, {r: h, z: above - 3.0}, {r: h - 3.0, z: above - 0.6},
      {r: h - 8.0, z: above}, {r: 0.0, z: above}
    ]);
    var parts:Array<Part> = [Solids.named(head, "head")];
    var t = n("panelThickness"), nut = n("nutThickness");
    parts.push(Solids.named(Part.cylinderSpan(n("panelHole") / 2, -t - nut, 0.0), "stem"));
    parts.push(Solids.named(Part.cylinderSpan(n("nutDiameter") / 2, -t - nut, -t), "nut"));
    var base = Part.box(n("baseWidth"), n("baseDepth"), n("baseLength"));
    parts.push(Solids.named(drop(base, -t - nut - n("baseLength")), "base"));
    // Contact blocks side by side across the base's depth, centred.
    var blockCount = count("blocks");
    for (i in 0...blockCount) {
      var block = Part.box(n("blockWidth"), n("blockThickness"), n("blockLength"));
      var y = (i - (blockCount - 1) / 2) * n("blockThickness");
      var moved = block.translated(new Vector(0, y, -t - nut - n("baseLength") - n("blockLength")));
      block.close();
      parts.push(Solids.named(moved, "block" + (i + 1)));
    }
    return Solids.union(parts);
  }

  static function drop(part:Part, z:Float):Part {
    var moved = part.translated(new Vector(0, 0, z));
    part.close();
    return moved;
  }
}
