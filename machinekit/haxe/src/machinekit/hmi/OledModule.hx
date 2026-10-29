package machinekit.hmi;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Footprint;
import cadkit.modeling.Part;
import cadkit.modeling.RoundedRectangle;
import cadkit.modeling.Vector;
import cadkit.threads.Details;
import cadkit.threads.SurfaceDetail;
import cadkit.threads.SurfaceDetail.DisplayBinding;
import cadkit.threads.SurfaceDetail.DisplaySpec;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as R;
import materia.assembly.AssemblyFrames;

/**
 * The Waveshare 2.42 in 128x64 OLED module (SKU 25742, SSD1309), modelled from its vendor drawing and 3D model. Origin at
 * the centre of the active area on the panel's front face, +z toward the operator, x to the right and y up as seen from
 * the front. The panel's back is the glass (60.5 x 37 x 1.5, the active area 2.7 mm off its centre toward the top), the
 * circuit board sits 2.4 mm behind the glass, and the module is 4.0 mm thick over all. Behind the board are the 7-pin
 * 2.54 right-angle header, the 1.25 mm 7-pin connector, the flex-circuit connector and the flex circuit itself, which
 * folds from the glass round the board's lower edge. The board's four plain 2.5 mm holes lie under the glass and cannot be
 * used to fasten it. The module is held by a printed frame over the board's edge strip (open at the bottom for the flex
 * circuit), screwed from behind into heat-set inserts in four bosses on the plate's back just outside the corners; four
 * pegs on the frame go through the board's holes to locate it. It is wired at the 2.54 mm header's pin ends, the wires
 * leaving along +x; its seven pins are VCC, GND, DIN (SDA), CLK (SCL), CS, DC and RES, for I2C with the module's 0 ohm
 * links moved from the SPI default.
 */
class OledModule extends HmiPart {
  static var recipe:Null<ComponentType>;

  // Vendor drawing 2_42inch_oled_module-sku25742 (millimetres).
  static inline var GLASS_WIDTH:Float = 60.5;
  static inline var GLASS_HEIGHT:Float = 37.0;
  static inline var GLASS_THICKNESS:Float = 1.5;
  static inline var VIEW_WIDTH:Float = 57.01;
  static inline var VIEW_HEIGHT:Float = 29.49;
  /** The active area's centre is this far above the glass's centre. */
  static inline var ACTIVE_OFFSET:Float = 2.675;
  static inline var PCB_WIDTH:Float = 61.5;
  static inline var PCB_HEIGHT:Float = 39.5;
  static inline var PCB_THICKNESS:Float = 1.6;
  static inline var PCB_CORNER:Float = 1.25;
  /** Front of the board behind the glass's front. */
  static inline var PCB_DEPTH:Float = 4.0;
  static inline var HOLE_DX:Float = 27.75;
  static inline var HOLE_DY:Float = 16.25;
  static inline var HOLE_DIAMETER:Float = 2.7;
  static inline var BOSS_DIAMETER:Float = 6.4;
  /** Clamp bar bosses: just outside the board's edges, in the strip above the flex connector. */
  static inline var CLAMP_X:Float = 34.5;
  static inline var CLAMP_THICKNESS:Float = 2.5;
  /** The frame's rails over the board's back edge: side rails and top and bottom rails (to the opening's edge). */
  static inline var RAIL_SIDE:Float = 3.15;
  static inline var RAIL_END:Float = 4.55;
  /** The ears' rows, from the board's centre line. */
  static inline var CLAMP_ROW:Float = 17.3;
  static inline var EAR_WIDTH:Float = 4.25;
  /** The flex circuit passes through the bottom rail here (half width), and small parts under it need this relief. */
  static inline var FLEX_OPENING:Float = 8.5;
  static inline var RELIEF_FROM:Float = -24.2;
  static inline var RELIEF_TO:Float = -13.2;
  static inline var RELIEF_DEPTH:Float = 1.3;
  static inline var CLAMP_HOLE:Float = 2.7;
  /** Locating pegs through the board's upper holes: 2.4 mm in the 2.7 mm holes, ending 0.7 mm short of the glass. */
  static inline var PEG_DIAMETER:Float = 2.4;
  static inline var PEG_PAD:Float = 5.0;
  static inline var PEG_CLEARANCE:Float = 0.7;
  static inline var HEADER_PITCH:Float = 2.54;
  static inline var HEADER_PINS:Int = 7;
  /** From the header body's inboard face to the pins' ends. */
  static inline var HEADER_LENGTH:Float = 11.1;
  static inline var CONNECTOR_HEIGHT:Float = 3.4;

  static function inputs():Array<ComponentParameter> return [
    HmiPart.basisParameter("catalog"),
    R.length("panelThickness", 3),
    R.scalar("massGrams", 30, 1)
  ];

  public function new(?v:ComponentValues) {
    super("hmi-oled-module", "2.42 in 128x64 OLED module (Waveshare SKU 25742)", "ABS black", inputs(), v);
    positive(["panelThickness", "massGrams"]);
    addConnector("panel", Mount, Solids.axial(0, 0, 0));
    addConnector(
      "header",
      Terminal,
      AssemblyFrames.alongY(headerX0() + HEADER_LENGTH, headerCentre(), boardBack() - HEADER_PITCH / 2, 1, 0, 0)
    );
    addPort(headerPort());
    var insert = ThreadedInsert.m2_5Short();
    // The glass covers the board's own holes, so the board is held by a frame over its back, screwed into four bosses on the
    // plate's back just outside the corners.
    var bosses = [for (sx in [-1.0, 1.0]) for (sy in [-1.0, 1.0]) {
      x:sx * CLAMP_X, y:boardCentre() + sy * CLAMP_ROW,
        diameter:BOSS_DIAMETER, height:PCB_DEPTH, pilotDiameter:insert.pilotDiameter, pilotDepth:insert.pilotDepth
    }
    ];
    // The window is the viewing area; the footprint of what is behind the plate is the module and bosses about the window's centre.
    mount(
      Rect(VIEW_WIDTH, VIEW_HEIGHT),
      Rect(VIEW_WIDTH + 6.0, VIEW_HEIGHT + 6.0),
      [],
      0.0,
      n("panelThickness") + PCB_DEPTH + CONNECTOR_HEIGHT,
      rearBody(),
      1e9,
      bosses
    );
    declareMass(n("massGrams") / 1000.0, new Vector(0, -ACTIVE_OFFSET, -n("panelThickness") - PCB_DEPTH / 2));
  }

  /** What is behind the plate, seen from the front: the board, its flex circuit and the frame's bosses, below the window's centre. */
  static function rearBody():Footprint {
    var half = Math.max(PCB_HEIGHT / 2 + 0.5, CLAMP_ROW + BOSS_DIAMETER / 2);
    return Shifted(Rect(2 * (CLAMP_X + BOSS_DIAMETER / 2), 2 * half), 0.0, boardCentre());
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.hmi.oled-module", inputs(), v -> new OledModule(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();


  /** The glass's centre, and the board's, below the active area's. */
  static function boardCentre():Float return -ACTIVE_OFFSET;
  /** The board's front stands this far behind the glass's back: the boss that holds it off the plate. */
  static function bossHeight():Float return PCB_DEPTH - PCB_THICKNESS;
  function glassFront():Float return -n("panelThickness");
  function boardFront():Float return glassFront() - bossHeight();
  function boardBack():Float return glassFront() - PCB_DEPTH;

  /** The printed frame, four inserts in the bosses and four screws from behind. */
  override public function hardware():Array<PanelHardware> {
    var insert = ThreadedInsert.m2_5Short();
    var screw = ButtonHeadScrew.m2_5(5.0);
    var frame = new PanelSolid("DISPLAY-FRAME", "display frame", clampFrame, "PA12 nylon");
    var result:Array<PanelHardware> = [new PanelHardware(
      "frame",
      frame,
      0.0,
      boardCentre(),
      boardBack() - CLAMP_THICKNESS
    )];
    var i = 0;
    for (sx in [-1.0, 1.0]) for (sy in [-1.0, 1.0]) {
      i++;
      var y = boardCentre() + sy * CLAMP_ROW;
      result.push(new PanelHardware('insert-$i', insert, sx * CLAMP_X, y, boardBack(), true));
      result.push(new PanelHardware('screw-$i', screw, sx * CLAMP_X, y, boardBack() - CLAMP_THICKNESS, true));
    }
    return result;
  }

  /**
   * The frame in its own frame: its origin on the board's centre at the frame's underside; its top lies on the board's back.
   * It presses the board's edge strip (open at the bottom so the flex circuit passes, relieved over the small parts there),
   * has an ear at each corner for the screws, and four pegs that go through the board's holes and stop short of the glass,
   * which covers them. The pegs locate the board; the screws clamp it.
   */
  function clampFrame():Part {
    var halfW = PCB_WIDTH / 2, halfH = PCB_HEIGHT / 2;
    var pieces:Array<Part> = [];
    // The ring: the board's outline less the opening.
    var outer = box(PCB_WIDTH, PCB_HEIGHT, CLAMP_THICKNESS, 0.0, 0.0, 0.0);
    var opening = box(PCB_WIDTH - 2 * RAIL_SIDE, PCB_HEIGHT - 2 * RAIL_END, CLAMP_THICKNESS + 2.0, 0.0, 0.0, -1.0);
    pieces.push(outer.subtract(opening));
    outer.close();
    opening.close();
    for (sx in [-1.0, 1.0]) for (sy in [-1.0, 1.0]) {
      // An ear: a web out from the rail to a boss-sized pad around the screw.
      var webX = (halfW + CLAMP_X) / 2;
      pieces.push(box(CLAMP_X - halfW + 1.0, EAR_WIDTH, CLAMP_THICKNESS, sx * webX, sy * CLAMP_ROW, 0.0));
      pieces.push(Part.cylinderSpan(BOSS_DIAMETER / 2, 0.0, CLAMP_THICKNESS, sx * CLAMP_X, sy * CLAMP_ROW));
      // A peg through the board's hole, on a pad that fills the corner of the opening.
      pieces.push(Part.cylinderSpan(PEG_PAD / 2, 0.0, CLAMP_THICKNESS, sx * HOLE_DX, sy * HOLE_DY));
      pieces.push(Part.cylinderSpan(
        PEG_DIAMETER / 2,
        CLAMP_THICKNESS,
        CLAMP_THICKNESS + PCB_THICKNESS + PEG_CLEARANCE,
        sx * HOLE_DX,
        sy * HOLE_DY
      ));
    }
    var plain = Solids.union(pieces);
    var tools:Array<Part> = [];
    for (sx in [-1.0, 1.0]) for (sy in [-1.0, 1.0]) tools.push(Part.cylinderSpan(
      CLAMP_HOLE / 2,
      -1.0,
      CLAMP_THICKNESS + 1.0,
      sx * CLAMP_X,
      sy * CLAMP_ROW
    ));
    // The flex circuit passes through the bottom rail; small parts under it get a shallow relief.
    tools.push(box(2 * FLEX_OPENING, RAIL_END + 1.0, CLAMP_THICKNESS + 2.0, 0.0, -halfH + RAIL_END / 2 - 0.5, -1.0));
    tools.push(box(
      RELIEF_TO - RELIEF_FROM,
      RAIL_END + 1.0,
      RELIEF_DEPTH + 1.0,
      (RELIEF_FROM + RELIEF_TO) / 2,
      -halfH + RAIL_END / 2 - 0.5,
      CLAMP_THICKNESS - RELIEF_DEPTH
    ));
    var result = plain.subtractAll(tools);
    plain.close();
    for (t in tools) t.close();
    return result;
  }

  var driver:Null<DisplaySpec> = null;

  /**
   * Says who fills the display in a simulation: the driver `source`, the operator profile it serves and which joint feeds each of
   * its signals. Without it the glass is plain. Returns the module, for a layout to chain.
   */
  public function driven(source:String, profile:String, bindings:Array<DisplayBinding>):OledModule {
    driver = new DisplaySpec(VIEW_WIDTH, VIEW_HEIGHT, 0.0, ACTIVE_OFFSET, source, profile, bindings);
    return this;
  }

  override public function bodies():Null<Array<DeviceBodySpec>> {
    return [
      new DeviceBodySpec("pcb", "OLED module circuit board", "FR4 PCB blue", pcb),
      new DeviceBodySpec("glass", "OLED panel", "ABS black", glass),
      new DeviceBodySpec("foam", "panel adhesive foam", "rubber", foam),
      new DeviceBodySpec("fpc", "OLED flex circuit", "polyimide film", flex),
      new DeviceBodySpec("zif", "flex circuit connector", "ABS grey", zif),
      new DeviceBodySpec("header", "2.54 mm header body", "device housing dark", headerBody),
      new DeviceBodySpec("pins", "2.54 mm header pins", "brass", headerPins, [headerPort()]),
      new DeviceBodySpec("connector", "1.25 mm 7-pin connector", "ABS white", sevenPinConnector)
    ];
  }

  function pcb():Part {
    var raw = RoundedRectangle.box(PCB_WIDTH, PCB_HEIGHT, PCB_CORNER, boardBack(), boardBack() + PCB_THICKNESS);
    var board = at(raw, 0.0, boardCentre(), 0.0);
    var tools:Array<Part> = [];
    for (sx in [-1.0, 1.0]) for (sy in [-1.0, 1.0]) tools.push(Part.cylinderSpan(
      HOLE_DIAMETER / 2,
      boardBack() - 1.0,
      boardBack() + PCB_THICKNESS + 1.0,
      sx * HOLE_DX,
      boardCentre() + sy * HOLE_DY
    ));
    var result = board.subtractAll(tools);
    board.close();
    for (t in tools) t.close();
    return Solids.named(result, "pcb");
  }

  /** Double-sided foam strips that bond the glass to the board across the gap between them, above the flex circuit. */
  function foam():Part {
    var pieces:Array<Part> = [];
    var top = boardCentre() + GLASS_HEIGHT / 2;
    for (dy in [3.0, 9.0]) pieces.push(box(
      2 * 24.0,
      3.0,
      bossHeight() - GLASS_THICKNESS,
      0.0,
      top - dy - 1.5,
      boardFront()
    ));
    return Solids.named(Solids.union(pieces), "foam");
  }

  /** The glass; when the module is `driven`, its front face carries the display (the active area), which a simulator fills. */
  function glass():Part {
    var plain = Solids.named(
      box(GLASS_WIDTH, GLASS_HEIGHT, GLASS_THICKNESS, 0.0, boardCentre(), glassFront() - GLASS_THICKNESS),
      "glass"
    );
    var spec = driver;
    if (spec == null) return plain;
    try {
      var shape = Details.attach(plain.shape, frontFace(plain.shape), Display(spec));
      plain.close();
      return new Part(shape);
    } catch (error:Dynamic) {
      plain.close();
      throw error;
    }
  }

  /** The strong name of the face looking toward the operator. */
  static function frontFace(shape:Shape):String {
    var names = shape.elementNames(CadKit.ShapeKind.Face);
    var found:Null<String> = null;
    for (index in 0...names.length) {
      var face = shape.subshape(CadKit.ShapeKind.Face, index);
      var front = face.surfaceKind() == CadKit.SurfaceKind.Plane && face.faceNormal().get_z() > 0.99;
      face.close();
      if (front) {
        if (found != null) throw "The OLED glass has more than one front face";
        found = names[index];
      }
    }
    if (found == null) throw "The OLED glass has no front face";
    return found;
  }

  /** The flex circuit: along the glass's back, round the board's lower edge and up its back to the connector. */
  function flex():Part {
    var w = 14.0, t = 0.12;
    var gap = glassFront() - (GLASS_THICKNESS + bossHeight()) / 2 - 0.0;
    var rear = boardBack() - 1.2;
    var low = boardCentre() - PCB_HEIGHT / 2 - 0.3;
    var glassBottom = boardCentre() - GLASS_HEIGHT / 2;
    var zifFront = boardCentre() + zifY() - 2.6;
    return Solids.named(
      Solids.union([
        box(w, glassBottom - low, t, 0.0, (glassBottom + low) / 2, gap - t / 2),
        box(w, t, gap - rear + t, 0.0, low - t / 2, rear - t / 2),
        box(w, zifFront - low, t, 0.0, (zifFront + low) / 2, rear - t / 2)
      ]),
      "flex"
    );
  }

  /** The flex connector's centre above the board's centre. */
  static function zifY():Float return 12.35;

  function zif():Part return Solids.named(box(16.1, 5.2, 2.25, 0.0, boardCentre() + zifY(), boardBack() - 2.25), "zif");

  /** Right-angle header: its 2.54 mm body stands inboard, the pins run out toward the board's edge. */
  static function headerCentre():Float return boardCentre() - 1.5;
  static function headerX0():Float return 16.09;

  function headerBody():Part return Solids.named(
    box(
      HEADER_PITCH,
      HEADER_PITCH * HEADER_PINS,
      HEADER_PITCH,
      headerX0() + HEADER_PITCH / 2,
      headerCentre(),
      boardBack() - HEADER_PITCH
    ),
    "header"
  );

  /** The module's I2C and power, on the header's seven pins. On the device, and on its header pins when it is expanded. */
  function headerPort():ComponentPort return {
    name: "header",
    kind: Signal,
    role: Consumer,
    iface: Plug("2.54-header", HEADER_PINS),
    required: true,
    connector: "header"
  };

  function headerPins():Part {
    var pins:Array<Part> = [];
    var length = HEADER_LENGTH - HEADER_PITCH;
    for (i in 0...HEADER_PINS) pins.push(box(
      length,
      0.64,
      0.64,
      headerX0() + HEADER_PITCH + length / 2,
      headerCentre() + (i - (HEADER_PINS - 1) / 2) * HEADER_PITCH,
      boardBack() - HEADER_PITCH / 2 - 0.32
    ));
    return Solids.named(Solids.union(pins), "pins");
  }

  /** The 1.25 mm connector, open toward the board's left edge, with a slot for the mating housing. */
  function sevenPinConnector():Part {
    var body = box(5.2, 14.01, CONNECTOR_HEIGHT, -24.15, boardCentre() + 0.5, boardBack() - CONNECTOR_HEIGHT);
    var slot = box(3.5, 10.5, 2.2, -24.15 - 1.3, boardCentre() + 0.5, boardBack() - 2.8);
    var result = body.subtract(slot);
    body.close();
    slot.close();
    return Solids.named(result, "connector");
  }

  /** A box centred on (x, y) with its lower face at z0. */
  static function box(w:Float, h:Float, d:Float, x:Float, y:Float, z0:Float):Part {
    var raw = Part.box(w, h, d);
    return at(raw, x, y, z0);
  }

  static function at(raw:Part, x:Float, y:Float, z:Float):Part {
    var moved = raw.translated(new Vector(x, y, z));
    raw.close();
    return moved;
  }
}
