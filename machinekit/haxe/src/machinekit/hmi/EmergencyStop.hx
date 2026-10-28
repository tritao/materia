package machinekit.hmi;

import cadkit.modeling.Footprint;
import cadkit.modeling.Part;
import cadkit.modeling.RoundedRectangle;
import cadkit.modeling.StrokeText;
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
class EmergencyStop extends HmiPart {
  static var recipe:Null<ComponentType>;

  /** Depth of the counterbore each terminal screw head sits in. */
  static inline var TERMINAL_POCKET:Float = 1.5;

  static function inputs():Array<ComponentParameter> return [
    R.catalog("model", ControlCatalogs.eStops(), ControlCatalogs.XB2),
    HmiPart.basisParameter("stated"),
    R.length("headDiameter", 40),
    R.length("collarDiameter", 30),
    R.length("collarHeight", 9),
    R.length("headHeight", 22),
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
    positive([
      "headDiameter",
      "collarDiameter",
      "collarHeight",
      "headHeight",
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
    require(count("blocks") >= 1 && count("blocks") <= 3, "one to three contact blocks");
    require(
      n("blockThickness") * count("blocks") <= n("baseDepth") + 1e-9 && n("blockWidth") <= n("baseWidth"),
      "the blocks fit the base"
    );
    addConnector("panel", Mount, Solids.axial(0, 0, 0));
    mount(panelCutout(), panelKeepOut(), [], aboveFace(), behindFace(), rearBody(), 1e9);
    for (port in contactPorts()) addPort(port);
    declareMass(n("massGrams") / 1000.0, new Vector(0, 0, (aboveFace() - behindFace()) / 2));
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.hmi.emergency-stop",
      inputs(), v -> new EmergencyStop(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();

  /** One NC dry contact per block, to be wired into the safety chain. On the device, and on the blocks when it is expanded. */
  function contactPorts():Array<ComponentPort> return [for (i in 1...count("blocks") + 1) {
    name:"nc" + i, kind:Signal, role:Passive, iface:Plug("dry-contact", 2), required:true
  }
  ];

  function panelCutout():Footprint return Circle(n("panelHole"));
  function panelKeepOut():Footprint return Circle(n("headDiameter"));
  function aboveFace():Float return n("collarHeight") + n("headHeight");
  function behindFace():Float return n("panelThickness") + n("nutThickness") + n("baseLength") + n("blockLength");
  function rearBody():Footprint return Rect(n("baseWidth"), n("baseDepth"));

  /**
   * The grey operator (stem and latching base), the stainless ring nut, the dark contact blocks that carry the NC contacts, and
   * the red mushroom head on its collar.
   */
  override public function bodies():Null<Array<DeviceBodySpec>> {
    var t = n("panelThickness"), nut = n("nutThickness");
    return [new DeviceBodySpec("operator", "emergency stop operator and latching base", "ABS grey", function() {
      // The latching base is a rounded block; the stem goes up through the panel from it.
      var base = RoundedRectangle.box(n("baseWidth"), n("baseDepth"), 3.0, -t - nut - n("baseLength"), -t - nut);
      return Solids.union([
        Solids.named(Part.cylinderSpan(n("panelHole") / 2, -t - nut, 0.0), "stem"),
        Solids.named(base, "base")
      ]);
    }
    ), new DeviceBodySpec("nut", "emergency stop ring nut", "stainless A2-70", function() {
      var ring = Part.cylinderSpan(n("nutDiameter") / 2, -t - nut, -t);
      var bore = Part.cylinderSpan(n("panelHole") / 2, -t - nut - 1.0, -t + 1.0);
      var result = ring.subtract(bore);
      ring.close();
      bore.close();
      return Solids.named(result, "nut");
    }
    ), new DeviceBodySpec("blocks", "emergency stop contact blocks", "device housing dark", function() {
      // Contact blocks side by side across the base's depth, centred. Each has a shoulder at one rear corner and four shallow
      // counterbores in its rear face for the terminal screws.
      var blocks:Array<Part> = [];
      for (i in 0...count("blocks")) {
        var block = Part.box(n("blockWidth"), n("blockThickness"), n("blockLength"));
        var y = blockY(i);
        var step = Part.box(2.0, n("blockThickness") + 2.0, 5.0);
        var shoulder = step.translated(new Vector(n("blockWidth") / 2 - 1.0, 0, 0));
        step.close();
        var cut = block.subtract(shoulder);
        block.close();
        shoulder.close();
        var tools = [for (spot in terminalSpots()) Part.cylinderSpan(3.1, -1.0, TERMINAL_POCKET, spot.x, spot.y)];
        tools.push(legendLocal());
        var drilled = cut.subtractAll(tools);
        cut.close();
        for (tool in tools) tool.close();
        var moved = drilled.translated(new Vector(0, y, blockRear()));
        drilled.close();
        blocks.push(Solids.named(moved, "block" + (i + 1)));
      }
      return Solids.union(blocks);
    }, contactPorts()),
      new DeviceBodySpec("contact-screws", "emergency stop terminal screws", "stainless A2-70", function() {
      var head = ScrewHeads.pan(2.2, 1.2, 2.6, 0.7);
      var heads:Array<Part> = [];
      for (i in 0...count("blocks")) for (spot in terminalSpots()) heads.push(head.translated(new Vector(
        spot.x,
        blockY(i) + spot.y,
        blockRear() + TERMINAL_POCKET
      )));
      head.close();
      return Solids.union(heads);
    }
    ), new DeviceBodySpec("legend", "emergency stop terminal numbers", "ABS white", function() {
      // The terminal numbers are an inlay, flush with the rear face: the same shape is cut from the block.
      var pieces:Array<Part> = [];
      for (i in 0...count("blocks")) {
        var text = legendLocal();
        pieces.push(text.translated(new Vector(0, blockY(i), blockRear())));
        text.close();
      }
      return Solids.union(pieces);
    }
    ), new DeviceBodySpec("head", "emergency stop mushroom head", materialSpec(), function() {
      var above = aboveFace(), c = n("collarHeight"), h = n("headDiameter") / 2;
      return Solids.named(
        Part.revolve([
          {r: 0.0, z: 0.0},
          {r: n("collarDiameter") / 2, z: 0.0},
          {r: n("collarDiameter") / 2, z: c},
          {r: h - 1.5, z: c + 1.5},
          {r: h, z: c + 4.0},
          {r: h, z: above - 3.0},
          {r: h - 3.0, z: above - 0.6},
          {r: h - 8.0, z: above},
          {r: 0.0, z: above}
        ]),
        "head"
      );
    }
    )];
  }

  /** Where each block's four terminal screws sit on its rear face, from the block's centre. */
  function terminalSpots():Array<{x:Float, y:Float}> return [for (sx in [-1.0, 1.0]) for (sy in [-1.0, 1.0]) {
    x:sx * 9.0, y:sy * (n("blockThickness") / 2 - 4.0)
  }
  ];

  /** The terminal numbers in a block's own frame (rear face on z = 0), 0.3 mm deep: 11 and 12, the NC pair, mirrored to read from behind. */
  function legendLocal():Part {
    var text = StrokeText.solid("11 12", 2.4, 0.3, 0.0, 0.3, true);
    var moved = text.translated(new Vector(0, -1.2, 0));
    text.close();
    return moved;
  }

  /** One block's centre across the base's depth. */
  function blockY(i:Int):Float return (i - (count("blocks") - 1) / 2) * n("blockThickness");

  /** The block's rear face: the base's back face less the block's length. */
  function blockRear():Float return -n("panelThickness") - n("nutThickness") - n("baseLength") - n("blockLength");

  static function drop(part:Part, z:Float):Part {
    var moved = part.translated(new Vector(0, 0, z));
    part.close();
    return moved;
  }
}
