package machinekit.hmi;

import cadkit.Shape;
import cadkit.modeling.Footprint;
import cadkit.modeling.Axis;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Polygon;
import cadkit.modeling.StrokeText;
import cadkit.threads.ThreadedFace.ThreadStart;
import cadkit.threads.Threads;
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
  static var recipe:Null<ComponentType>;

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
    if (recipe == null) recipe = new ComponentType("machinekit.hmi.hand-wheel", inputs(), v -> new HandWheel(v));
    return recipe;
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

  /** The studs, each a named cylinder whose side carries the thread texture; plain when `textured` is false. */
  function studSolid(textured:Bool):Part {
    var thread = FastenerThreads.coarse(n("studDiameter"));
    var studs:Array<Part> = [];
    var i = 0;
    for (position in studPositions()) {
      i++;
      var rod = Solids.named(
        Part.cylinderSpan(n("studDiameter") / 2, -n("studLength"), 0.0, position.x, position.y),
        'stud$i'
      );
      if (textured) {
        studs.push(new Part(Threads.attach(
          rod.shape,
          'stud$i:cyl.side',
          thread,
          AxisMaximum,
          n("studLength") - 0.5,
          0.5
        )));
        rod.close();
      } else studs.push(rod);
    }
    return Solids.union(studs);
  }

  /** ISO 4032 hex nuts on the studs behind the panel, each bore named for its own internal thread texture. */
  function nutSolid(textured:Bool):Part {
    var size = HexNut.metric("M" + Dimension.format(n("studDiameter")));
    var nuts:Array<Part> = [];
    var i = 0;
    for (position in studPositions()) {
      i++;
      var body = Part.prism(Polygon.regular(6, size.acrossFlats), 0.0, size.height);
      var bore = Solids.named(Part.cylinderSpan(n("studDiameter") / 2, -0.1, size.height + 0.1), 'bore$i');
      var tool = bore;
      if (textured) {
        tool = new Part(Threads.attach(bore.shape, 'bore$i:cyl.side', size.thread, AxisMinimum));
        bore.close();
      }
      var cut = Solids.cut(body, [tool]);
      var moved = cut.translated(new Vector(position.x, position.y, -n("panelThickness") - size.height));
      cut.close();
      nuts.push(moved);
    }
    return Solids.union(nuts);
  }

  /**
   * The whole device as one solid, with plain studs and nuts: their coincident surfaces cannot keep two thread textures. The
   * envelope has a plain wheel (no knurling or scale), which is the silhouette; the preview is the fused bodies, built once
   * for each set of numbers because the knurling and scale make it cost seconds.
   */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var key = recipeType().key(values());
    if (detail != Envelope) {
      var cached = previewCache.get(key);
      if (cached != null) return new Part(cached.cloneShape());
    }
    var parts:Array<Part> = [];
    for (spec in bodies()) parts.push(spec.id == "studs" ? studSolid(false)
      : spec.id == "nuts" ? nutSolid(false) : spec.id == "wheel" && detail == Envelope ? plainWheel() : spec.build());
    var result = Solids.union(parts);
    if (detail != Envelope) previewCache.set(key, result.shape.cloneShape());
    return result;
  }

  /** Whole-device preview solids already built, by recipe key. */
  static var previewCache:Map<String, Shape> = [];

  /** The wheel's silhouette: the cone, the grip at its tip radius, the recessed dial and the spinner, without knurling or scale. */
  function plainWheel():Part {
    var top = bodyHeight(), flange = n("flangeThickness");
    var parts:Array<Part> = [Part.revolve([
      {r: 0.0, z: flange},
      {r: n("outerDiameter") / 2, z: flange},
      {r: n("ringDiameter") / 2, z: flange + n("coneHeight")},
      {r: n("ringDiameter") / 2, z: top},
      {r: n("dialDiameter") / 2, z: top},
      {r: n("dialDiameter") / 2, z: top - n("dialRecess")},
      {r: 0.0, z: top - n("dialRecess")}
    ])];
    if (flag("handle")) parts.push(Part.cylinderSpan(
      n("handleDiameter") / 2,
      top - n("dialRecess"),
      top + n("handleProtrusion"),
      0.0,
      -n("handleRadius")
    ));
    return Solids.union(parts);
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
    var top = bodyHeight(), flange = n("flangeThickness");
    var rear = -n("bossDepth");
    return [new DeviceBodySpec("housing", "hand wheel housing", materialSpec(), function() {
      // The flange, with a groove round its foot for the seal.
      var flangeBody = Part.cylinderSpan(n("outerDiameter") / 2, 0.0, flange);
      var groove = Part.cylinderSpan(n("outerDiameter") / 2 + 1.0, -1.0, GASKET_HEIGHT);
      var inner = Part.cylinderSpan(n("outerDiameter") / 2 - GASKET_WIDTH, -2.0, GASKET_HEIGHT + 1.0);
      var cutter = groove.subtract(inner);
      groove.close();
      inner.close();
      var result = flangeBody.subtract(cutter);
      flangeBody.close();
      cutter.close();
      return Solids.named(result, "flange");
    }
    ), new DeviceBodySpec("gasket", "hand wheel panel seal", "rubber", function() {
      var ring = Part.cylinderSpan(n("outerDiameter") / 2, 0.0, GASKET_HEIGHT);
      var bore = Part.cylinderSpan(n("outerDiameter") / 2 - GASKET_WIDTH, -1.0, GASKET_HEIGHT + 1.0);
      var result = ring.subtract(bore);
      ring.close();
      bore.close();
      return Solids.named(result, "gasket");
    }
    ), new DeviceBodySpec(
      "backplate",
      "hand wheel back plate",
      "ABS black",
      () -> Solids.named(Part.cylinderSpan(n("bossDiameter") / 2, rear, 0.0), "boss")
    ), new DeviceBodySpec(
      "studs",
      "hand wheel mounting studs",
      "steel 10.9",
      () -> studSolid(true)
    ), new DeviceBodySpec(
      "nuts",
      "hand wheel stud nuts",
      "stainless A2-70",
      () -> nutSolid(true)
    ), new DeviceBodySpec("terminals", "hand wheel terminal block", "ABS black", function() {
      // A base on the boss with a barrier at each end and between the poles.
      var height = blockHeight(), base = Math.min(1.0, height * 0.4), wall = 1.2, poles = terminalNames().length;
      var pitch = (n("terminalLength") - wall) / poles;
      var parts:Array<Part> = [boxAt(n("terminalLength"), n("terminalWidth"), base, 0.0, rear - base)];
      for (k in 0...poles + 1) parts.push(boxAt(
        wall,
        n("terminalWidth"),
        height,
        -n("terminalLength") / 2 + wall / 2 + k * pitch,
        rear - height
      ));
      return Solids.named(Solids.union(parts), "terminals");
    }, terminalPorts()),
      new DeviceBodySpec("terminal-screws", "hand wheel terminal screws", "stainless A2-70", function() {
      // A pan head with a cross recess in each pole, sitting on the base and pointing back.
      var base = Math.min(1.0, blockHeight() * 0.4), headHeight = Math.min(1.2, blockHeight() - base - 0.3);
      var one = ScrewHeads.pan(2.2, headHeight, 2.8, 0.8);
      var heads:Array<Part> = [for (x in poleCentres()) one.translated(new Vector(x, 0, rear - base))];
      one.close();
      return Solids.union(heads);
    }
    ), new DeviceBodySpec("labels", "hand wheel terminal names", "ABS white", function() {
      // Raised lettering on the boss's rear face beside the poles, mirrored so it reads from behind.
      var pieces:Array<Part> = [];
      var poles = poleCentres(), names = terminalNames();
      var y = -(n("terminalWidth") / 2 + 1.5 + 3.2);
      for (k in 0...names.length) {
        var text = StrokeText.solid(names[k], 3.2, 0.4, rear - 0.3, rear, true);
        pieces.push(text.translated(new Vector(-poles[k], y, 0)));
        text.close();
      }
      return Solids.named(Solids.union(pieces), "labels");
    }
    ), new DeviceBodySpec("wheel", "hand wheel", materialSpec(), wheelSolid)];
  }

  /** Wheel solids already built, by the numbers they depend on: the knurling and the scale make one cost seconds to build. */
  static var wheelCache:Map<String, Shape> = [];

  /** The wheel, from the cache when these numbers have been built before; the caller owns the solid and closes it. */
  function wheelSolid():Part {
    var key = [for (name in [
      "outerDiameter",
      "flangeThickness",
      "coneHeight",
      "ringDiameter",
      "ringHeight",
      "dialDiameter",
      "dialRecess",
      "handleDiameter",
      "handleProtrusion",
      "handleRadius",
      "knurlDepth"
    ]) Std.string(n(name))].join("|")
      + "|" + count("knurlTeeth") + "|" + count("scaleDivisions") + "|" + (flag("handle") ? "h" : "-");
    var cached = wheelCache.get(key);
    if (cached != null) return new Part(cached.cloneShape());
    var built = buildWheel();
    wheelCache.set(key, built.shape.cloneShape());
    return built;
  }

  function buildWheel():Part {
    var top = bodyHeight(), flange = n("flangeThickness");
    var root = n("ringDiameter") / 2 - n("knurlDepth");
    var parts:Array<Part> = [Solids.named(
      Part.revolve([
        {r: 0.0, z: flange},
        {r: n("outerDiameter") / 2, z: flange},
        {r: root, z: flange + n("coneHeight")},
        {r: root, z: top},
        {r: n("dialDiameter") / 2, z: top},
        {r: n("dialDiameter") / 2, z: top - n("dialRecess")},
        {r: 0.0, z: top - n("dialRecess")}
      ]),
      "wheel"
    )];
    parts.push(knurledRing(flange + n("coneHeight"), top, root));
    if (count("scaleDivisions") > 0) parts.push(scaleBand());
    if (flag("handle")) {
      var from = top - n("dialRecess");
      var pin = Part.cylinderSpan(n("handleDiameter") / 2, from, top + n("handleProtrusion"), 0.0, -n("handleRadius"));
      // A hex socket in the spinner's end, for the screw that holds it.
      var socket = Part.prism(
        Polygon.regular(6, 3.0),
        top + n("handleProtrusion") - 1.5,
        top + n("handleProtrusion") + 0.1
      );
      var socketMoved = socket.translated(new Vector(0.0, -n("handleRadius"), 0.0));
      socket.close();
      var handle = pin.subtract(socketMoved);
      pin.close();
      socketMoved.close();
      parts.push(Solids.named(handle, "handle"));
    }
    return Solids.union(parts);
  }

  /** Fine straight knurling round the grip: a triangular tooth profile, one extrusion from root radius to tip radius. */
  function knurledRing(z0:Float, z1:Float, root:Float):Part {
    var teeth = count("knurlTeeth"), tip = n("ringDiameter") / 2;
    var outline:Array<Vector> = [];
    for (i in 0...teeth) {
      var a = 2 * Math.PI * i / teeth, b = a + Math.PI / teeth;
      outline.push(new Vector(tip * Math.cos(a), tip * Math.sin(a)));
      outline.push(new Vector(root * Math.cos(b), root * Math.sin(b)));
    }
    var solid = Part.prism(outline, z0, z1);
    var bore = Part.cylinderSpan(root - 0.4, z0 - 1.0, z1 + 1.0);
    var ring = solid.subtract(bore);
    solid.close();
    bore.close();
    return Solids.named(ring, "knurl");
  }

  /**
   * The scale on the wheel's skirt: ticks (long every tenth, medium every fifth) along the outer part and the tens numbered
   * above them, raised 0.12 mm on the cone. A tick is a sliver revolved about the axis through its own small angle, so it
   * follows the cone, and one sliver per length is turned into place; the numerals are cut as plan-view shapes and intersected
   * with a thin skin along the cone. Zero is at the left and the numbers rise counter-clockwise.
   */
  function scaleBand():Part {
    var outerR = n("outerDiameter") / 2, flange = n("flangeThickness"), rise = n("coneHeight");
    var root = n("ringDiameter") / 2 - n("knurlDepth");
    var dr = root - outerR, length = Math.sqrt(dr * dr + rise * rise);
    var nx = rise / length, nz = -dr / length;
    // The cone at fraction t from its foot (plan radius and height).
    function radius(t:Float):Float return outerR + dr * t;
    function height(t:Float):Float return flange + rise * t;
    var divisions = count("scaleDivisions");
    var pieces:Array<Part> = [];
    // One sliver of each length, then a copy turned to each division.
    var width = 0.22;
    for (reach in [0.36, 0.28, 0.2]) {
      var turn = width / radius(0.2);
      var sliver = Part.revolve(
        [
          {r: radius(0.06) - nx * 0.1, z: height(0.06) - nz * 0.1},
          {r: radius(reach) - nx * 0.1, z: height(reach) - nz * 0.1},
          {r: radius(reach) + nx * 0.12, z: height(reach) + nz * 0.12},
          {r: radius(0.06) + nx * 0.12, z: height(0.06) + nz * 0.12}
        ],
        turn
      );
      for (k in 0...divisions) {
        var long = k % 10 == 0, medium = !long && k % 5 == 0;
        if ((reach == 0.36) != long || (reach == 0.28) != medium) continue;
        var a = Math.PI + 2 * Math.PI * k / divisions;
        pieces.push(sliver.placed(Location.rotation(Axis.Z(), a - turn / 2)));
      }
      sliver.close();
    }
    // The tens, upright from outside: the baseline on the outer side, the top toward the centre.
    var tens = divisions >= 10 ? Std.int(divisions / 10) : 0;
    if (tens > 0) {
      var skin = Part.revolve([
        {r: radius(0.40) - nx * 0.1, z: height(0.40) - nz * 0.1},
        {r: radius(0.96) - nx * 0.1, z: height(0.96) - nz * 0.1},
        {r: radius(0.96) + nx * 0.12, z: height(0.96) + nz * 0.12},
        {r: radius(0.40) + nx * 0.12, z: height(0.40) + nz * 0.12}
      ]);
      var z0 = flange - 1.0, z1 = flange + rise + 1.0;
      var numerals:Array<Part> = [];
      for (k in 0...tens) {
        var a = Math.PI + 2 * Math.PI * (k * 10) / divisions, baseline = radius(0.46);
        numerals.push(StrokeText.solid(
          Std.string(k * 10),
          2.2,
          0.24,
          z0,
          z1,
          false,
          baseline * Math.cos(a),
          baseline * Math.sin(a),
          a + Math.PI / 2
        ));
      }
      var flat = Part.fuseAll(numerals);
      for (numeral in numerals) numeral.close();
      pieces.push(skin.intersect(flat));
      skin.close();
      flat.close();
    }
    var all = Part.fuseAll(pieces);
    for (piece in pieces) piece.close();
    return Solids.named(all, "scale");
  }

  override public function joints():Array<DeviceJointSpec> return [new DeviceJointSpec(
    "encoder",
    Continuous,
    "housing",
    "wheel"
  )];

  /** A box centred on (x, y) with its lower face at z0. */
  static function boxAt(w:Float, h:Float, d:Float, x:Float, z0:Float):Part return boxAtY(w, h, d, x, 0.0, z0);

  static function boxAtY(w:Float, h:Float, d:Float, x:Float, y:Float, z0:Float):Part {
    var raw = Part.box(w, h, d);
    var moved = raw.translated(new Vector(x, y, z0));
    raw.close();
    return moved;
  }

  static function shifted(part:Part, z:Float):Part {
    var moved = part.translated(new Vector(0, 0, z));
    part.close();
    return moved;
  }
}
