package machinekit.hmi;

import cadkit.modeling.Footprint;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as R;

/**
 * A 16 mm panel push button: a bezel flange and a domed cap in front, a threaded barrel through the panel
 * with a nut, a body with terminals behind. `illuminated` buttons carry an LED. Sizes are assumed until a
 * real button is measured: 19 mm bezel, 16 mm cap standing 6 mm, 24 mm body 25 mm deep.
 */
class PushButton extends HmiPart {
  static var recipe:Null<ComponentType>;

  static function inputs():Array<ComponentParameter> return [
    HmiPart.basisParameter("assumed"),
    R.choice("colour", ["green", "yellow", "red", "white", "blue", "black"], "white"),
    R.flag("illuminated", true),
    R.length("bezelDiameter", 19),
    R.length("bezelHeight", 1.5),
    R.length("capDiameter", 16),
    R.length("capHeight", 6),
    R.length("keepOutDiameter", 20),
    R.length("barrelDiameter", 16.2),
    R.length("panelThickness", 3),
    R.length("nutDiameter", 22),
    R.length("nutThickness", 2.5),
    R.length("bodyDiameter", 24),
    R.length("behindDepth", 25),
    R.scalar("massGrams", 8, 1)
  ];

  public function new(?v:ComponentValues) {
    super(
      "hmi-push-button",
      "16 mm push button",
      "ABS " + (v == null || v.get("colour") == null ? "white" : v.token("colour")),
      inputs(),
      v
    );
    positive([
      "bezelDiameter",
      "bezelHeight",
      "capDiameter",
      "capHeight",
      "keepOutDiameter",
      "barrelDiameter",
      "panelThickness",
      "nutDiameter",
      "nutThickness",
      "bodyDiameter",
      "behindDepth",
      "massGrams"
    ]);
    require(
      n("capDiameter") < n("bezelDiameter") && n("bezelDiameter") <= n("keepOutDiameter"),
      "the cap is smaller than the bezel and the bezel fits its keep-out"
    );
    require(n("capHeight") > n("bezelHeight"), "the cap stands above the bezel");
    require(
      n("barrelDiameter") < n("bezelDiameter") && n("barrelDiameter") < n("nutDiameter"),
      "the barrel passes through bezel and nut"
    );
    require(n("behindDepth") > n("panelThickness") + n("nutThickness"), "the body has room behind the nut");
    addConnector("panel", Mount, Solids.axial(0, 0, 0));
    mount(panelCutout(), panelKeepOut(), [], aboveFace(), behindFace(), rearBody(), 1e9);
    declareMass(n("massGrams") / 1000.0, new Vector(0, 0, (aboveFace() - behindFace()) / 2));
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.hmi.push-button", inputs(), v -> new PushButton(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();

  function panelCutout():Footprint return Circle(n("barrelDiameter"));
  function panelKeepOut():Footprint return Circle(n("keepOutDiameter"));
  function aboveFace():Float return n("capHeight");
  function behindFace():Float return n("behindDepth");
  function rearBody():Footprint return Circle(n("bodyDiameter"));

  /** A metal barrel and nut, the grey rear body and the coloured cap and bezel in front. */
  override public function bodies():Null<Array<DeviceBodySpec>> {
    var t = n("panelThickness"), nut = n("nutThickness");
    return [new DeviceBodySpec("housing", "push button barrel and nut", "stainless A2-70", function() {
      return Solids.union([
        Solids.named(Part.cylinderSpan(n("barrelDiameter") / 2, -t - nut, 0.0), "barrel"),
        Solids.named(Part.cylinderSpan(n("nutDiameter") / 2, -t - nut, -t), "nut")
      ]);
    }
    ), new DeviceBodySpec("body", "push button rear body", "ABS grey", function() {
      return Solids.named(Part.cylinderSpan(n("bodyDiameter") / 2, -n("behindDepth"), -t - nut), "body");
    }
    ), new DeviceBodySpec("cap", "push button cap", materialSpec(), function() {
      var b = n("bezelDiameter") / 2, c = n("capDiameter") / 2, bh = n("bezelHeight"), top = n("capHeight");
      return Solids.named(
        Part.revolve([
          {r: 0.0, z: 0.0},
          {r: b, z: 0.0},
          {r: b, z: bh},
          {r: c, z: bh},
          {r: c, z: top - 1.2},
          {r: c - 1.2, z: top},
          {r: 0.0, z: top}
        ]),
        "cap"
      );
    }
    )];
  }
}
