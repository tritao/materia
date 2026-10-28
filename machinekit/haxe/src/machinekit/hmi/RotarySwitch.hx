package machinekit.hmi;

import cadkit.modeling.Footprint;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as R;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

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
    R.scalar("detentDegrees", 30, 1, 180),
    R.scalar("startDegrees", 0, -180, 180),
    R.length("knobDiameter", 25),
    R.length("knobHeight", 18),
    R.length("bushDiameter", 10),
    R.length("panelThickness", 3),
    R.length("nutDiameter", 14),
    R.length("nutThickness", 2),
    R.length("bodyDiameter", 30),
    R.length("behindDepth", 25),
    R.scalar("massGrams", 25, 1)
  ];

  public function new(?v:ComponentValues) {
    super(
      "hmi-rotary-switch",
      "rotary switch with knob, " + (v == null
      || v.get("positions") == null ? 12 : v.integer("positions")) + " positions",
      "ABS black",
      inputs(),
      v
    );
    positive([
      "knobDiameter",
      "knobHeight",
      "bushDiameter",
      "panelThickness",
      "nutDiameter",
      "nutThickness",
      "bodyDiameter",
      "behindDepth",
      "massGrams",
      "detentDegrees"
    ]);
    require(count("positions") >= 2 && count("positions") <= 24, "two to 24 positions");
    require((count("positions") - 1) * n("detentDegrees") < 360.0, "the positions fit round the switch");
    require(
      n("bushDiameter") < n("nutDiameter") && n("nutDiameter") < n("knobDiameter"),
      "the nut is wider than the bush and narrower than the knob"
    );
    require(n("behindDepth") > n("panelThickness") + n("nutThickness"), "the body has room behind the nut");
    addConnector("panel", Mount, Solids.axial(0, 0, 0));
    mount(panelCutout(), panelKeepOut(), [], aboveFace(), behindFace(), rearBody(), 1e9);
    declareMass(n("massGrams") / 1000.0, new Vector(0, 0, (aboveFace() - behindFace()) / 2));
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.hmi.rotary-switch", inputs(), v -> new RotarySwitch(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();

  function panelCutout():Footprint return Circle(n("bushDiameter") + 0.2);
  function panelKeepOut():Footprint return Circle(n("knobDiameter"));
  function aboveFace():Float return n("knobHeight");
  function behindFace():Float return n("behindDepth");
  function rearBody():Footprint return Circle(n("bodyDiameter"));

  /** The switch (brass bush and nut, grey body) and the knob that turns on it. */
  override public function bodies():Null<Array<DeviceBodySpec>> {
    var t = n("panelThickness"), nut = n("nutThickness");
    return [new DeviceBodySpec("switch", "rotary switch bush and nut", "brass", function() {
      return Solids.union([
        Solids.named(Part.cylinderSpan(n("bushDiameter") / 2, -t - nut, 0.0), "bush"),
        Solids.named(Part.cylinderSpan(n("nutDiameter") / 2, -t - nut, -t), "nut")
      ]);
    }
    ), new DeviceBodySpec("body", "rotary switch body", "ABS grey", function() {
      return Solids.named(Part.cylinderSpan(n("bodyDiameter") / 2, -n("behindDepth"), -t - nut), "body");
    }
    ), new DeviceBodySpec("knob", "rotary switch knob", materialSpec(), function() {
      var r = n("knobDiameter") / 2, k = n("knobHeight");
      return Solids.named(
        Part.revolve([
          {r: 0.0, z: 0.0},
          {r: r - 1.0, z: 0.0},
          {r: r, z: 1.0},
          {r: r, z: k - 2.0},
          {r: r - 2.0, z: k},
          {r: 0.0, z: k}
        ]),
        "knob"
      );
    }
    )];
  }

  /**
   * The knob turns on the switch between end stops and clicks into each of its positions, `detentDegrees` apart, the first at
   * `startDegrees` (the knob's rest angle). Both come from the part: pitch and start differ between switches, and the rotation
   * outside the first and last position is not available, because the end stops block it. Editors place the knob only at the
   * positions unless asked for free motion.
   */
  override public function joints():Array<DeviceJointSpec> {
    var pitch = n("detentDegrees") * Math.PI / 180.0, start = n("startDegrees") * Math.PI / 180.0;
    var positions = count("positions");
    var limits:AssemblyJointLimits = {
      lower: start,
      upper: start + (positions - 1) * pitch,
      velocity: null,
      effort: null,
      detents: [for (i in 0...positions) start + i * pitch]
    };
    return [new DeviceJointSpec("position", Revolute, "switch", "knob", limits, start)];
  }
}
