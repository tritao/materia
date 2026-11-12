package machinekit.hmi;

import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.modeling.Footprint;
import cadkit.modeling.Part;
import cadkit.modeling.RoundedRectangle;
import cadkit.modeling.StrokeText;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as R;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;

/**
 * A 22 mm twist-release emergency stop (XB2-ES542 style): a mushroom head on a collar, the stem through the
 * panel, a ring nut behind it, a latching base and one to three snap-on contact blocks across it. Defaults
 * are a seller's drawing of that family: 31 above the panel, 43 behind it (nut, base and a 24 mm block),
 * a 42 mm wide base, 29 mm blocks. The block row, the base's other dimension and the head profile are drawn
 * from photos. A compact maker's part (EAO Series 84, APEM EC Compact) is the same recipe with a thin base.
 */
class EmergencyStop extends HmiPart {
  static final recipe = new machinekit.cache.Once<ComponentType>();

  /** Depth of the counterbore each terminal screw head sits in. */
  static inline var TERMINAL_POCKET:Float = 1.5;

  static function inputs():Array<ComponentParameter> return [
    R.catalog("model", ControlCatalogs.eStops(), ControlCatalogs.XB2),
    HmiPart.basisParameter("stated"),
    R.length("headDiameter", 40),
    R.length("collarDiameter", 30),
    R.length("collarHeight", 9),
    R.length("headHeight", 22),
    R.length("pressTravel", 4),
    R.length("panelHole", 22.3),
    R.length("panelThickness", 3),
    R.length("nutDiameter", 30),
    R.length("nutThickness", 5),
    R.length("baseWidth", 42),
    R.length("baseDepth", 30),
    R.length("baseLength", 11),
    R.length("blockWidth", 29),
    R.length("blockThickness", 14.8),
    R.length("blockLength", 24),
    R.count("blocks", 1),
    R.scalar("massGrams", 52.4, 1)
  ];

  public function new(?v:ComponentValues) {
    super(
      "hmi-emergency-stop",
      "twist-release emergency stop with " + (v == null
      || v.get("blocks") == null ? 1 : v.integer("blocks")) + " NC contact block(s)",
      "ABS red",
      inputs(),
      ControlCatalogs.withModel(ControlCatalogs.eStops(), ControlCatalogs.XB2, v)
    );
    addFacet(EngineeringMetadata.assumed(""));
    positive([
      "headDiameter",
      "collarDiameter",
      "collarHeight",
      "headHeight",
      "pressTravel",
      "panelHole",
      "panelThickness",
      "nutDiameter",
      "nutThickness",
      "baseWidth",
      "baseDepth",
      "baseLength",
      "blockWidth",
      "blockThickness",
      "blockLength",
      "massGrams"
    ]);
    require(
      n("collarDiameter") < n("headDiameter") && n("panelHole") < n("collarDiameter"),
      "the head is wider than its collar and the collar wider than the opening"
    );
    require(n("pressTravel") + 1 < n("collarHeight"), "the collar stands taller than the head's travel");
    require(count("blocks") >= 1 && count("blocks") <= 3, "one to three contact blocks");
    require(
      n("blockThickness") * count("blocks") <= n("baseDepth") + 1e-9 && n("blockWidth") <= n("baseWidth"),
      "the blocks fit the base"
    );
    addConnector("panel", Mount, Solids.axial(0, 0, 0));
    mount(panelCutout(), panelKeepOut(), [], aboveFace(), behindFace(), rearBody(), 1e9);
    for (i in 1...count("blocks") + 1) addConnector(
      "block" + i,
      Terminal,
      Solids.rearward(0, blockY(i - 1), blockRear())
    );
    for (port in contactPorts()) addPort(port);
    declareMass(n("massGrams") / 1000.0, new Vector(0, 0, (aboveFace() - behindFace()) / 2));
  }

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.hmi.emergency-stop",
      inputs(), v -> new EmergencyStop(v)).numbered(v -> v.token("model") == "CUSTOM"
        ? 'ESTOP-ASSUMED-${v.integer("blocks")}NC' : '${v.token("model")}-${v.integer("blocks")}NC')
      .procured(_ -> Buy(null, null, null)).requiringInputs(["panelHole", "blocks"], [], []));
  }
  override public function componentType():Null<ComponentType> return recipeType();

  /**
   * One NC dry contact per block, to be wired into the safety chain, at the terminal screws on the block's rear face. On the
   * device, and on the blocks when it is expanded.
   */
  function contactPorts():Array<ComponentPort> return [for (i in 1...count("blocks") + 1) {
    name:"nc" + i, kind:Signal, role:Passive, iface:Plug("dry-contact", 2), required:true, connector:"block" + i
  }
  ];

  function panelCutout():Footprint return Circle(n("panelHole"));
  function panelKeepOut():Footprint return Circle(n("headDiameter"));
  function aboveFace():Float return n("collarHeight") + n("headHeight");
  function behindFace():Float return n("panelThickness") + n("nutThickness") + n("baseLength") + n("blockLength");
  function rearBody():Footprint return Rect(n("baseWidth"), n("baseDepth"));

  /**
   * The head is pushed in by `pressTravel` and latches there until it is twisted out: released at 0, pushed at the end of its travel.
   * It latches (no rest position), and a pushed head opens the contacts, so a simulator reads the chain as open while it is in.
   */
  override public function joints():Array<DeviceJointSpec> {
    var travel = n("pressTravel");
    var limits:AssemblyJointLimits = {lower: -travel,
      upper: 0.0, velocity: null, effort: null, detents: [-travel, 0.0]};
    return [new DeviceJointSpec("press", Prismatic, "operator", "head", limits, 0.0)];
  }

  /**
   * The grey operator (stem and latching base), the stainless ring nut, the dark contact blocks that carry the NC contacts, and
   * the red mushroom head on its collar.
   */
  override public function bodies():Null<Array<DeviceBodySpec>> {
    var t = n("panelThickness"), nut = n("nutThickness");
    return [new DeviceBodySpec("operator", "emergency stop operator and latching base", "ABS grey", b -> {
      // The latching base is a rounded block; the stem goes up through the panel from it.
      b.label(b.unionAll([
        b.label(HmiPart.post(b, n("panelHole") / 2, -t - nut, 0.0), "Stem"),
        b.label(HmiPart.post(b, postRadius(), -1.0, postTop()), "Post"),
        b.label(HmiPart.roundedBlock(b, n("baseWidth"), n("baseDepth"), 3.0, -t - nut - n("baseLength"), -t - nut), "Latching base")
      ]), "Operator");
    }),
      new DeviceBodySpec("nut", "emergency stop ring nut", "stainless A2-70", b -> b.label(HmiPart.tube(b, n("nutDiameter") / 2,
        n("panelHole") / 2, -t - nut, -t), "Ring nut")),
      new DeviceBodySpec("blocks", "emergency stop contact blocks", "device housing dark", b -> {
        // Contact blocks side by side across the base's depth, centred. Each has a shoulder at one rear corner and four shallow
        // counterbores in its rear face for the terminal screws, and its terminal numbers inlaid flush in that face.
        var w = n("blockWidth"), th = n("blockThickness"), rear = blockRear();
        var blocks:Array<Feature> = [];
        for (i in 0...count("blocks")) {
          var y = blockY(i), which = 'block ${i + 1}';
          var body = HmiPart.block(b, w, th, n("blockLength"), 0.0, y, rear);
          var tools:Array<Feature> = [b.boxBetween(w / 2 - 2.0, w / 2, y - th / 2 - 1.0, y + th / 2 + 1.0, rear, rear + 5.0)];
          var labels = ['$which shoulder'];
          for (k => spot in terminalSpots()) {
            tools.push(HmiPart.post(b, 3.1, rear - 1.0, rear + TERMINAL_POCKET, spot.x, y + spot.y));
            labels.push('$which terminal ${k + 1} counterbore');
          }
          tools.push(legend(b, y, rear));
          labels.push('$which terminal numbers');
          blocks.push(b.label(b.cutAll(body, tools, labels), 'Contact block ${i + 1}'));
        }
        b.label(HmiPart.fuse(b, blocks), "Contact blocks");
      }, contactPorts()),
      new DeviceBodySpec("contact-screws", "emergency stop terminal screws", "stainless A2-70", b -> {
        var heads:Array<Feature> = [];
        for (i in 0...count("blocks")) for (spot in terminalSpots())
          heads.push(ScrewHeads.pan(b, 2.2, 1.2, 2.6, 0.7, spot.x, blockY(i) + spot.y, blockRear() + TERMINAL_POCKET));
        b.label(HmiPart.fuse(b, heads), "Terminal screws");
      }),
      new DeviceBodySpec("legend", "emergency stop terminal numbers", "ABS white", b -> {
        // The terminal numbers are an inlay, flush with the rear face: the same shape is cut from the block.
        var pieces:Array<Feature> = [for (i in 0...count("blocks")) legend(b, blockY(i), blockRear())];
        b.label(HmiPart.fuse(b, pieces), "Terminal numbers");
      }),
      new DeviceBodySpec("head", "emergency stop mushroom head", materialSpec(), b -> {
        // Drawn released: the collar's foot stands `pressTravel` above the panel and reaches it when the head is pushed in. The head is
        // hollow underneath and slides over the operator's post, on its wall, at both ends of the travel.
        var above = aboveFace(), c = n("collarHeight"), h = n("headDiameter") / 2, travel = n("pressTravel");
        var post = postRadius();
        b.label(HmiPart.turned(b, [
          {r: post, z: travel},
          {r: n("collarDiameter") / 2, z: travel},
          {r: n("collarDiameter") / 2, z: c},
          {r: h - 1.5, z: c + 1.5},
          {r: h, z: c + 4.0},
          {r: h, z: above - 3.0},
          {r: h - 3.0, z: above - 0.6},
          {r: h - 8.0, z: above},
          {r: 0.0, z: above},
          {r: 0.0, z: postTop() + travel + 1.0},
          {r: post, z: postTop() + travel + 1.0}
        ]), "Mushroom head");
      })];
  }

  /** The post the head slides over: a little narrower than the opening, and as tall as the head's travel plus the length it stays engaged. */
  function postRadius():Float return n("panelHole") / 2 - 1.0;
  function postTop():Float return n("pressTravel") + 3.0;

  /** Where each block's four terminal screws sit on its rear face, from the block's centre. */
  function terminalSpots():Array<{x:Float, y:Float}> return [for (sx in [-1.0, 1.0]) for (sy in [-1.0, 1.0]) {
    x:sx * 9.0, y:sy * (n("blockThickness") / 2 - 4.0)
  }
  ];

  /** A block's terminal numbers, 0.3 mm deep in its rear face at `rear`, centred on y: 11 and 12, the NC pair, mirrored to read from behind. */
  function legend(b:DocumentBuilder, y:Float, rear:Float):Feature return b.strokeText("11 12", 2.4, 0.3, rear, rear + 0.3, true, 0.0, y - 1.2);

  /** One block's centre across the base's depth. */
  function blockY(i:Int):Float return (i - (count("blocks") - 1) / 2) * n("blockThickness");

  /** The block's rear face: the base's back face less the block's length. */
  function blockRear():Float return -n("panelThickness") - n("nutThickness") - n("baseLength") - n("blockLength");


}
