package machinekit.hmi;

import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.modeling.Footprint;
import cadkit.modeling.Part;
import cadkit.modeling.Polygon;
import cadkit.threads.SurfaceDetail;
import cadkit.threads.ThreadedFace.ThreadStart;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as R;
import machinekit.standard.FastenerThreads;
import machinekit.standard.HexNut;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/**
 * A manual pulse generator hand wheel: a flange on the panel, a flared body with a knurled grip ring and a
 * recessed dial, an optional spinner handle in the recess, and behind the panel a boss through the opening,
 * three studs on a circle and a terminal strip. Defaults are the YUMO ISM6045 (maker's drawing: 60 mm overall,
 * 26 above the panel, M3 studs on a 50.8 circle, 43 boss, studs 10 behind); the Vallder wheel is the same
 * recipe with other values. The body's flare and the dial recess are drawn to look right, not measured.
 */
class HandWheel extends HmiPart {
  static final recipe = new machinekit.cache.Once<ComponentType>();

  /** The black seal round the flange's foot. */
  static inline var GASKET_WIDTH:Float = 1.6;
  static inline var GASKET_HEIGHT:Float = 1.4;

  static function inputs():Array<ComponentParameter> return [
    R.catalog("model", ControlCatalogs.wheels(), ControlCatalogs.YUMO),
    HmiPart.basisParameter("catalog"),
    R.length("outerDiameter", 60),
    R.length("flangeThickness", 6),
    R.length("coneHeight", 11),
    R.length("ringDiameter", 49),
    R.length("ringHeight", 9),
    R.length("dialDiameter", 44),
    R.length("dialRecess", 2),
    R.choice("surfaceDetail", ["textured", "modelled"], "textured"),
    R.count("knurlTeeth", 120),
    R.length("knurlDepth", 0.5),
    R.count("scaleDivisions", 100),
    R.flag("handle", true),
    R.length("handleDiameter", 10),
    R.length("handleProtrusion", 12),
    R.length("handleRadius", 17),
    R.length("bossDiameter", 43),
    R.length("bossDepth", 3),
    R.length("panelClearance", 1),
    R.length("panelThickness", 3),
    R.length("maxPanelThickness", 6),
    R.length("studCircleDiameter", 50.8),
    R.length("studDiameter", 3),
    R.length("studHoleDiameter", 3.2),
    R.length("studLength", 10),
    R.count("studCount", 3),
    R.length("terminalLength", 28),
    R.length("terminalWidth", 11),
    R.length("terminalDepth", 5.5),
    R.scalar("massGrams", 250, 1)
  ];

  public function new(?v:ComponentValues) {
    super(
      "hmi-hand-wheel",
      "manual pulse generator hand wheel",
      "aluminium 6061",
      inputs(),
      ControlCatalogs.withModel(ControlCatalogs.wheels(), ControlCatalogs.YUMO, v)
    );
    positive([
      "outerDiameter",
      "flangeThickness",
      "ringDiameter",
      "ringHeight",
      "dialDiameter",
      "bossDiameter",
      "studCircleDiameter",
      "studDiameter",
      "studHoleDiameter",
      "studLength",
      "knurlDepth",
      "panelThickness",
      "massGrams",
      "maxPanelThickness"
    ]);
    require(n("studHoleDiameter") >= n("studDiameter"), "the stud holes pass the studs");
    require(
      n("ringDiameter") <= n("outerDiameter") && n("dialDiameter") < n("ringDiameter") - 2 * n("knurlDepth") - 0.8,
      "the wheel narrows from flange to ring to dial"
    );
    require(
      n("dialRecess") >= 0 && n("dialRecess") < n("ringHeight") + n("coneHeight"),
      "the dial recess is shallower than the wheel"
    );
    require(n("bossDiameter") < n("studCircleDiameter") - n("studDiameter"), "the studs clear the boss");
    require(
      !flag("handle") || n("handleRadius") + n("handleDiameter") / 2 <= n("dialDiameter") / 2 + 1e-9,
      "the handle sits inside the dial"
    );
    require(count("studCount") >= 1 && count("studCount") <= 6, "one to six studs");
    addConnector("panel", Mount, Solids.axial(0, 0, 0));
    addConnector("terminals", Terminal, Solids.rearward(0, 0, -n("terminalDepth")));
    for (port in terminalPorts()) addPort(port);
    mount(panelCutout(), panelKeepOut(), panelHoles(), aboveFace(), behindFace(), rearBody(), n("maxPanelThickness"));
    declareMass(n("massGrams") / 1000.0, new Vector(0, 0, (aboveFace() - behindFace()) / 2));
  }

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.hmi.hand-wheel", inputs(), v -> new HandWheel(v))
      .numbered(v -> v.token("model") == "CUSTOM" ? 'HANDWHEEL-ASSUMED-${v.integer("scaleDivisions")}DIV' : v.token("model")));
  }
  override public function componentType():Null<ComponentType> return recipeType();

  function bodyHeight():Float return n("flangeThickness") + n("coneHeight") + n("ringHeight");

  function panelCutout():Footprint return Circle(n("bossDiameter") + n("panelClearance"));
  function panelKeepOut():Footprint return Circle(n("outerDiameter"));
  function panelHoles():Array<{x:Float, y:Float, diameter:Float}> {
    var studs = count("studCount"), result = [];
    for (i in 0...studs) {
      var angle = Math.PI / 2 + i * 2 * Math.PI / studs;
      result.push({
        x: n("studCircleDiameter") / 2 * Math.cos(angle),
        y: n("studCircleDiameter") / 2 * Math.sin(angle),
        diameter: n("studHoleDiameter")
      });
    }
    return result;
  }
  function aboveFace():Float return bodyHeight() + (flag("handle") ? n("handleProtrusion") : 0.0);
  function behindFace():Float return Math.max(n("studLength"), n("terminalDepth"));
  /** The studs' circle and the nuts' corners beyond it, with a millimetre of margin. */
  function rearBody():Footprint {
    var nut = HexNut.metric("M" + Dimension.format(n("studDiameter")));
    return Circle(n("studCircleDiameter") + nut.acrossFlats / Math.cos(Math.PI / 6) + 1.0);
  }

  /** The terminal block's poles and what is printed beside them, as a maker's wheel has them: A, B, 0 V and the supply. */
  static function terminalNames():Array<String> return ["A", "B", "0V", "VCC"];

  /** One port per terminal: the two encoder phases are signals, the other two are the supply. */
  function terminalPorts():Array<ComponentPort> return [for (name in terminalNames()) {
    name:name, kind:name == "A" || name == "B" ? Signal : ElectricalPower,
      role : name == "0V" || name == "VCC" ? Consumer : Supply, iface : Plug(
        "screw-terminal",
        1
      ), required : true, connector : "terminals"
  }
  ];

  /** The studs' angles round the stud circle, the first at the top. */
  function studPositions():Array<{x:Float, y:Float}> return [for (i in 0...count("studCount")) {
    var angle = Math.PI / 2 + i * 2 * Math.PI / count("studCount");
    {
      x:n("studCircleDiameter") / 2 * Math.cos(angle), y:n("studCircleDiameter") / 2 * Math.sin(angle)
    };
  }
  ];

  /** The studs, each a cylinder whose side carries the thread texture; plain when `textured` is false. */
  function studs(b:DocumentBuilder, textured:Bool):Feature {
    var thread = FastenerThreads.coarse(n("studDiameter"));
    var rods:Array<Feature> = [];
    for (i => position in studPositions()) {
      var rod = b.label(HmiPart.post(b, n("studDiameter") / 2, -n("studLength"), 0.0, position.x, position.y), 'Stud ${i + 1}');
      rods.push(textured ? b.thread(rod, "cyl.side", thread, AxisMaximum, n("studLength") - 0.5, 0.5) : rod);
    }
    return b.label(b.unionAll(rods), "Studs");
  }

  /** ISO 4032 hex nuts on the studs behind the panel, each bore carrying its own internal thread texture. */
  function nuts(b:DocumentBuilder, textured:Bool):Feature {
    var size = HexNut.metric("M" + Dimension.format(n("studDiameter")));
    var z0 = -n("panelThickness") - size.height;
    var nuts:Array<Feature> = [];
    for (i => position in studPositions()) {
      var plane = new cadkit.modeling.Plane(new Vector(position.x, position.y, z0), Vector.X(), Vector.Z());
      var hex = b.extrude(b.polygon(Polygon.regular(6, size.acrossFlats), plane), size.height);
      var bore:Feature = HmiPart.post(b, n("studDiameter") / 2, z0 - 0.1, z0 + size.height + 0.1, position.x, position.y);
      if (textured) bore = b.thread(bore, "cyl.side", size.thread, AxisMinimum);
      nuts.push(b.label(b.cutAll(hex, [bore], ["threaded bore"]), 'Nut ${i + 1}'));
    }
    return b.label(b.unionAll(nuts), "Nuts");
  }

  /**
   * The whole device as one solid, with plain studs and nuts (their coincident surfaces cannot keep two thread textures) and
   * the wheel's plain silhouette when it is textured or the envelope is asked for.
   */
  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    PartRecordings.recipeInputs(this, b);
    var parts:Array<Feature> = [];
    for (spec in bodies()) parts.push(switch spec.id {
      case "studs": studs(b, false);
      case "nuts": nuts(b, false);
      case "wheel" if (detail == Envelope || textured()): plainWheel(b);
      case _: b.label(spec.record(b), spec.label);
    });
    return b.label(HmiPart.fuse(b, parts), "Device");
  }

  /** The wheel's silhouette: the cone, the grip at its tip radius, the recessed dial and the spinner, without knurling or scale. */
  function plainWheel(b:DocumentBuilder):Feature {
    var top = bodyHeight(), flange = n("flangeThickness");
    var wheel = HmiPart.turned(b, [
      {r: 0.0, z: flange},
      {r: n("outerDiameter") / 2, z: flange},
      {r: n("ringDiameter") / 2, z: flange + n("coneHeight")},
      {r: n("ringDiameter") / 2, z: top},
      {r: n("dialDiameter") / 2, z: top},
      {r: n("dialDiameter") / 2, z: top - n("dialRecess")},
      {r: 0.0, z: top - n("dialRecess")}
    ]);
    if (!flag("handle")) return b.label(wheel, "Wheel");
    var handle = HmiPart.post(b, n("handleDiameter") / 2, top - n("dialRecess"), top + n("handleProtrusion"), 0.0, -n("handleRadius"));
    return b.label(b.unionAll([wheel, handle]), "Wheel");
  }

  /** The terminal block's height behind the boss, and the pole spacing between its barriers. */
  function blockHeight():Float return n("terminalDepth") - n("bossDepth");

  /** The poles' centres along x, between thin barriers that end the block and separate its poles. */
  function poleCentres():Array<Float> {
    var poles = terminalNames().length, wall = 1.2, pitch = (n("terminalLength") - wall) / poles;
    return [for (i in 0...poles) - n("terminalLength") / 2 + wall / 2 + (i + 0.5) * pitch];
  }

  /**
   * The housing (flange and boss) fixed to the panel; the three studs with their thread texture and the hex nuts on them
   * behind the panel; the terminal block with its screws and the names printed beside its poles; and the wheel that turns on
   * the housing: the cone, grip ring, dial and spinner handle. The wheel's rotation is the encoder's angle.
   */
  override public function bodies():Null<Array<DeviceBodySpec>> {
    var flange = n("flangeThickness");
    var rear = -n("bossDepth");
    return [new DeviceBodySpec("housing", "hand wheel housing", materialSpec(), b -> {
      // The flange, with a groove round its foot for the seal.
      var body = b.label(HmiPart.post(b, n("outerDiameter") / 2, 0.0, flange), "Flange");
      var groove = b.cutAll(HmiPart.post(b, n("outerDiameter") / 2 + 1.0, -1.0, GASKET_HEIGHT),
        [HmiPart.post(b, n("outerDiameter") / 2 - GASKET_WIDTH, -2.0, GASKET_HEIGHT + 1.0)], ["groove inside"]);
      b.label(b.cutAll(body, [groove], ["seal groove"]), "Seal groove");
    }),
      new DeviceBodySpec("gasket", "hand wheel panel seal", "rubber", b -> b.label(HmiPart.tube(b, n("outerDiameter") / 2,
        n("outerDiameter") / 2 - GASKET_WIDTH, 0.0, GASKET_HEIGHT), "Seal")),
      new DeviceBodySpec("backplate", "hand wheel back plate", "ABS black", b -> b.label(HmiPart.post(b, n("bossDiameter") / 2, rear, 0.0), "Boss")),
      new DeviceBodySpec("studs", "hand wheel mounting studs", "steel 10.9", b -> studs(b, true)),
      new DeviceBodySpec("nuts", "hand wheel stud nuts", "stainless A2-70", b -> nuts(b, true)),
      new DeviceBodySpec("terminals", "hand wheel terminal block", "ABS black", b -> {
        // A moulded comb on the boss: a base with a barrier at each end and between the poles, one section across its width.
        var height = blockHeight(), base = Math.min(1.0, height * 0.4), wall = 1.2, poles = terminalNames().length;
        var pitch = (n("terminalLength") - wall) / poles, x0 = -n("terminalLength") / 2;
        var section = [new Vector(x0, rear, 0)];
        for (k in 0...poles + 1) {
          var left = x0 + k * pitch;
          section.push(new Vector(left, rear - height, 0));
          section.push(new Vector(left + wall, rear - height, 0));
          if (k < poles) {
            section.push(new Vector(left + wall, rear - base, 0));
            section.push(new Vector(left + pitch, rear - base, 0));
          }
        }
        section.push(new Vector(x0 + n("terminalLength"), rear, 0));
        // The section lies in the XZ plane at the block's front face (y = -width / 2) and is extruded back across the width.
        var plane = new cadkit.modeling.Plane(new Vector(0, -n("terminalWidth") / 2, 0), Vector.X(), Vector.Y().scale(-1));
        b.label(b.extrude(b.polygon(section, plane), n("terminalWidth"), Vector.Y()), "Terminal block");
      }, terminalPorts()),
      new DeviceBodySpec("terminal-screws", "hand wheel terminal screws", "stainless A2-70", b -> {
        // A pan head with a cross recess in each pole, sitting on the base and pointing back.
        var base = Math.min(1.0, blockHeight() * 0.4), headHeight = Math.min(1.2, blockHeight() - base - 0.3);
        b.label(b.unionAll([for (x in poleCentres()) ScrewHeads.pan(b, 2.2, headHeight, 2.8, 0.8, x, 0.0, rear - base)]), "Terminal screws");
      }),
      new DeviceBodySpec("labels", "hand wheel terminal names", "ABS white", b -> {
        // Raised lettering on the boss's rear face beside the poles, mirrored so it reads from behind.
        var poles = poleCentres(), names = terminalNames();
        var y = -(n("terminalWidth") / 2 + 1.5 + 3.2);
        b.label(b.unionAll([for (k in 0...names.length) b.strokeText(names[k], 3.2, 0.4, rear - 0.3, rear, true, -poles[k], y)]), "Terminal names");
      }),
      new DeviceBodySpec("wheel", "hand wheel", materialSpec(), wheel)];
  }

  /** Whether the knurling and scale are textures on a plain wheel (the default) or modelled as geometry. */
  function textured():Bool return values().token("surfaceDetail") == "textured";

  /** The segments of the wheel's revolved profile below whose faces carry details. */
  static inline var CONE_SEGMENT:Int = 1;
  static inline var GRIP_SEGMENT:Int = 2;

  function wheel(b:DocumentBuilder):Feature {
    var top = bodyHeight(), flange = n("flangeThickness");
    // Textured, the grip is a plain cylinder at its tip radius; modelled, its root, with the teeth added to it.
    var root = textured() ? n("ringDiameter") / 2 : n("ringDiameter") / 2 - n("knurlDepth");
    var body:Feature = b.label(HmiPart.turned(b, [
      {r: 0.0, z: flange},
      {r: n("outerDiameter") / 2, z: flange},
      {r: root, z: flange + n("coneHeight")},
      {r: root, z: top},
      {r: n("dialDiameter") / 2, z: top},
      {r: n("dialDiameter") / 2, z: top - n("dialRecess")},
      {r: 0.0, z: top - n("dialRecess")}
    ]), "Wheel body");
    var parts:Array<Feature> = [];
    if (textured()) {
      // The knurling on the grip and the scale on the skirt are cosmetic details of those two faces.
      var knurled = b.label(b.surfaceDetail(body, HmiPart.sweptFace(body, GRIP_SEGMENT), Knurl(count("knurlTeeth"), n("knurlDepth"))), "Knurling");
      var rise = n("coneHeight"), reach = n("ringDiameter") / 2 - n("outerDiameter") / 2;
      parts.push(count("scaleDivisions") > 0 ? b.label(b.surfaceDetail(knurled, HmiPart.sweptFace(body, CONE_SEGMENT),
        Graduation(count("scaleDivisions"), 10, Math.sqrt(rise * rise + reach * reach), n("ringDiameter") / 2)), "Scale") : knurled);
    } else {
      parts.push(body);
      parts.push(knurledRing(b, flange + n("coneHeight"), top, root));
      if (count("scaleDivisions") > 0) parts.push(scaleBand(b));
    }
    if (flag("handle")) {
      var from = top - n("dialRecess"), end = top + n("handleProtrusion");
      var pin = HmiPart.post(b, n("handleDiameter") / 2, from, end, 0.0, -n("handleRadius"));
      // A hex socket in the spinner's end, for the screw that holds it.
      var socket = b.extrude(b.polygon(Polygon.regular(6, 3.0), new cadkit.modeling.Plane(new Vector(0.0, -n("handleRadius"), end - 1.5), Vector.X(),
        Vector.Z())), 1.6);
      parts.push(b.label(b.cutAll(pin, [socket], ["hex socket"]), "Spinner handle"));
    }
    return b.label(HmiPart.fuse(b, parts), "Wheel");
  }

  /** Fine straight knurling round the grip: a triangular tooth profile, one extrusion from root radius to tip radius. */
  function knurledRing(b:DocumentBuilder, z0:Float, z1:Float, root:Float):Feature {
    var teeth = count("knurlTeeth"), tip = n("ringDiameter") / 2;
    var outline:Array<Vector> = [];
    for (i in 0...teeth) {
      var a = 2 * Math.PI * i / teeth, c = a + Math.PI / teeth;
      outline.push(new Vector(tip * Math.cos(a), tip * Math.sin(a)));
      outline.push(new Vector(root * Math.cos(c), root * Math.sin(c)));
    }
    var toothed = b.extrude(b.polygon(outline, new cadkit.modeling.Plane(new Vector(0, 0, z0), Vector.X(), Vector.Z())), z1 - z0);
    return b.label(b.cutAll(toothed, [HmiPart.post(b, root - 0.4, z0 - 1.0, z1 + 1.0)], ["bore"]), "Knurled grip");
  }

  /**
   * The scale on the wheel's skirt: ticks (long every tenth, medium every fifth) along the outer part and the tens numbered
   * above them, raised 0.12 mm on the cone. A tick is a sliver revolved about the axis through its own small angle, so it
   * follows the cone, and one sliver per length is turned into place; the numerals are cut as plan-view shapes and intersected
   * with a thin skin along the cone. Zero is at the left and the numbers rise counter-clockwise.
   */
  function scaleBand(b:DocumentBuilder):Feature {
    var outerR = n("outerDiameter") / 2, flange = n("flangeThickness"), rise = n("coneHeight");
    var root = n("ringDiameter") / 2 - n("knurlDepth");
    var dr = root - outerR, length = Math.sqrt(dr * dr + rise * rise);
    var nx = rise / length, nz = -dr / length;
    // The cone at fraction t from its foot (plan radius and height).
    function radius(t:Float):Float return outerR + dr * t;
    function height(t:Float):Float return flange + rise * t;
    function band(from:Float, to:Float):Array<{r:Float, z:Float}> return [
      {r: radius(from) - nx * 0.1, z: height(from) - nz * 0.1},
      {r: radius(to) - nx * 0.1, z: height(to) - nz * 0.1},
      {r: radius(to) + nx * 0.12, z: height(to) + nz * 0.12},
      {r: radius(from) + nx * 0.12, z: height(from) + nz * 0.12}
    ];
    var divisions = count("scaleDivisions");
    var pieces:Array<Feature> = [];
    // One sliver of each length, then a copy turned to each division.
    var width = 0.22, turn = width / radius(0.2);
    for (reach in [0.36, 0.28, 0.2]) {
      var sliver = HmiPart.turned(b, band(0.06, reach), turn);
      for (k in 0...divisions) {
        var long = k % 10 == 0, medium = !long && k % 5 == 0;
        if ((reach == 0.36) != long || (reach == 0.28) != medium) continue;
        var a = Math.PI + 2 * Math.PI * k / divisions;
        pieces.push(b.rotate(sliver, new Vector(0, 0, 0), Vector.Z(), a - turn / 2));
      }
    }
    // The tens, upright from outside: the baseline on the outer side, the top toward the centre.
    var tens = divisions >= 10 ? Std.int(divisions / 10) : 0;
    if (tens > 0) {
      var skin = HmiPart.turned(b, band(0.40, 0.96));
      var z0 = flange - 1.0, z1 = flange + rise + 1.0;
      var numerals:Array<Feature> = [for (k in 0...tens) {
        var a = Math.PI + 2 * Math.PI * (k * 10) / divisions, baseline = radius(0.46);
        b.strokeText(Std.string(k * 10), 2.2, 0.24, z0, z1, false, baseline * Math.cos(a), baseline * Math.sin(a), a + Math.PI / 2);
      }];
      pieces.push(b.label(b.intersect(skin, numerals.length == 1 ? numerals[0] : b.unionAll(numerals)), "Numerals"));
    }
    return b.label(b.unionAll(pieces), "Scale");
  }

  override public function joints():Array<DeviceJointSpec> return [new DeviceJointSpec(
    "encoder",
    Continuous,
    "housing",
    "wheel"
  )];

}
