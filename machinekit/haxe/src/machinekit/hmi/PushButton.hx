package machinekit.hmi;

import cadkit.modeling.Footprint;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.threads.Details;
import cadkit.threads.SurfaceDetail;
import cadkit.threads.SurfaceDetail.DisplayBinding;
import cadkit.threads.SurfaceDetail.DisplaySpec;
import machinekit.component.*;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import machinekit.component.ComponentRecipeSupport as R;

/**
 * A 16 mm panel push button: a bezel flange and a domed cap in front, a threaded barrel through the panel
 * with a nut, a body with terminals behind. `illuminated` buttons carry an LED. Sizes are assumed until a
 * real button is measured: 19 mm bezel, 16 mm cap standing 6 mm, 24 mm body 25 mm deep. The terminals (assumed solder
 * terminals) are on the body's rear end, where the wires leave backward.
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
    R.length("pressTravel", 1),
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
      "pressTravel",
      "massGrams"
    ]);
    require(
      n("capDiameter") < n("bezelDiameter") && n("bezelDiameter") <= n("keepOutDiameter"),
      "the cap is smaller than the bezel and the bezel fits its keep-out"
    );
    require(n("capHeight") > n("bezelHeight"), "the cap stands above the bezel");
    require(n("pressTravel") < n("bezelHeight"), "the cap stays in its bezel through its travel");
    require(
      n("barrelDiameter") < n("bezelDiameter") && n("barrelDiameter") < n("nutDiameter"),
      "the barrel passes through bezel and nut"
    );
    require(n("behindDepth") > n("panelThickness") + n("nutThickness"), "the body has room behind the nut");
    addConnector("panel", Mount, Solids.axial(0, 0, 0));
    addConnector("terminals", Terminal, Solids.rearward(0, 0, -n("behindDepth")));
    for (port in terminalPorts()) addPort(port);
    mount(panelCutout(), panelKeepOut(), [], aboveFace(), behindFace(), rearBody(), 1e9);
    declareMass(n("massGrams") / 1000.0, new Vector(0, 0, (aboveFace() - behindFace()) / 2));
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.hmi.push-button", inputs(), v -> new PushButton(v))
      .numbered(v -> 'PUSHBUTTON-ASSUMED-${v.token("colour").toUpperCase()}-${v.boolean("illuminated") ? "LIT" : "UNLIT"}');
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();

  /**
   * What is wired at the rear terminals: the normally open contact, and the LED of an illuminated button. The LED is not
   * required: an unused key stays dark. On the device, and on its rear body when it is expanded.
   */
  function terminalPorts():Array<ComponentPort> {
    var result:Array<ComponentPort> = [{
      name: "no",
      kind: Signal,
      role: Passive,
      iface: Plug("solder-terminal", 2),
      required: true,
      connector: "terminals"
    }];
    if (flag("illuminated")) result.push({
      name: "led",
      kind: ElectricalPower,
      role: Consumer,
      iface: Plug("solder-terminal", 2),
      required: false,
      connector: "terminals"
    });
    return result;
  }

  function panelCutout():Footprint return Circle(n("barrelDiameter"));
  function panelKeepOut():Footprint return Circle(n("keepOutDiameter"));
  function aboveFace():Float return n("capHeight");
  function behindFace():Float return n("behindDepth");
  function rearBody():Footprint return Circle(n("bodyDiameter"));

  /**
   * The cap slides in the bezel by `pressTravel` and clicks at its two ends: released at 0, pressed at the end of its travel.
   * It is a momentary button: its spring returns it to 0 when released, so a simulator reads "pressed" only while it is held.
   */
  override public function joints():Array<DeviceJointSpec> {
    var travel = n("pressTravel");
    var limits:AssemblyJointLimits = {lower: -travel,
      upper: 0.0, velocity: null, effort: null, detents: [-travel, 0.0], rest: 0.0};
    return [new DeviceJointSpec("press", Prismatic, "bezel", "cap", limits, 0.0)];
  }

  var light:Null<DisplaySpec> = null;

  /**
   * Says who lights the cap in a simulation: the driver `source`, the operator profile it serves, the joints that feed its signals
   * and the `output` of the driver that is this button's LED. The cap's top face then carries a disc a simulator fills with the
   * LED's colour. Without it the cap is plain. Returns the button, for a layout to chain.
   */
  public function lit(source:String, profile:String, bindings:Array<DisplayBinding>, output:String):PushButton {
    var size = n("capDiameter") * 0.8;
    light = new DisplaySpec(size, size, 0.0, 0.0, source, profile, bindings, output, true);
    return this;
  }

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
    }, terminalPorts()), new DeviceBodySpec("bezel", "push button bezel", materialSpec(), function() {
      var b = n("bezelDiameter") / 2, hole = n("capDiameter") / 2, bh = n("bezelHeight");
      return Solids.named(Part.revolve([{r: hole, z: 0.0}, {r: b, z: 0.0}, {r: b, z: bh}, {r: hole, z: bh}]), "bezel");
    }
    ), new DeviceBodySpec("cap", "push button cap", materialSpec(), function() {
      // The cap stands clear of the barrel by its travel (no more than the bezel is deep, so it stays in the bezel), and sinks as it is pressed.
      var c = n("capDiameter") / 2, top = n("capHeight"), rest = n("pressTravel");
      var plain = Solids.named(
        Part.revolve([{r: 0.0,
        z: rest}, {r: c, z: rest}, {r: c, z: top - 1.2}, {r: c - 1.2, z: top}, {r: 0.0, z: top}]),
        "cap"
      );
      var spec = light;
      if (spec == null) return plain;
      try {
        var shape = Details.attach(plain.shape, HmiPart.frontFace(plain.shape, "push button cap"), Display(spec));
        plain.close();
        return new Part(shape);
      } catch (error:Dynamic) {
        plain.close();
        throw error;
      }
    }
    )];
  }
}
