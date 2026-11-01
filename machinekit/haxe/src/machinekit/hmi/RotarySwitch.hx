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
 * for the axis switch). All sizes are assumed until a real switch is measured: knob 25, bush 10, body 30 by 25. Its
 * common and one contact per position are on solder terminals at the body's rear end, so a board reads the position as
 * one input per position: exactly one closed is a valid position.
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
    addConnector("terminals", Terminal, Solids.rearward(0, 0, -n("behindDepth")));
    addPort(contactsPort());
    mount(panelCutout(), panelKeepOut(), [], aboveFace(), behindFace(), rearBody(), 1e9);
    declareMass(n("massGrams") / 1000.0, new Vector(0, 0, (aboveFace() - behindFace()) / 2));
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.hmi.rotary-switch", inputs(), v -> new RotarySwitch(v))
      .numbered(v -> 'ROTARY-SWITCH-ASSUMED-${v.integer("positions")}POS');
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();

  /** The common and one contact per position, wired at the rear terminals. On the device, and on its body when it is expanded. */
  function contactsPort():ComponentPort return {
    name: "contacts",
    kind: Signal,
    role: Passive,
    iface: Plug("solder-terminal", count("positions") + 1),
    required: true,
    connector: "terminals"
  };

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
    }, [contactsPort()]), new DeviceBodySpec("knob", "rotary switch knob", materialSpec(), function() {
      var r = n("knobDiameter") / 2, k = n("knobHeight");
      var body = Part.revolve([
        {r: 0.0, z: 0.0},
        {r: r - 1.0, z: 0.0},
        {r: r, z: 1.0},
        {r: r, z: k - 2.0},
        {r: r - 2.0, z: k},
        {r: 0.0, z: k}
      ]);
      // The pointer's groove, filled flush by the white pointer body.
      var groove = pointerGroove();
      var cut = body.subtract(groove);
      body.close();
      groove.close();
      return Solids.named(cut, "knob");
    }
    ), new DeviceBodySpec(
      "pointer",
      "rotary switch pointer",
      "ABS white",
      () -> Solids.named(pointerGroove(), "pointer")
    )];
  }

  /** The white pointer's shape: a line along the knob's top from near the centre toward its rim, 0.5 mm deep, at the knob's rest angle (up). */
  function pointerGroove():Part {
    var r = n("knobDiameter") / 2, k = n("knobHeight"), from = 4.0, to = r - 3.5, depth = 0.5;
    var raw = Part.box(1.4, to - from, depth);
    var moved = raw.translated(new Vector(0.0, (from + to) / 2, k - depth));
    raw.close();
    return moved;
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
    // The pointer is part of the knob: it is fixed to it and turns with it.
    return [
      new DeviceJointSpec("position", Revolute, "switch", "knob", limits, start),
      new DeviceJointSpec("pointer-mount", Fixed, "knob", "pointer")
    ];
  }
}
