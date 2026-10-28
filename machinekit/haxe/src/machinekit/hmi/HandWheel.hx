package machinekit.hmi;

import cadkit.modeling.Footprint;
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
      "panelThickness",
      "massGrams",
      "maxPanelThickness"
    ]);
    require(n("studHoleDiameter") >= n("studDiameter"), "the stud holes pass the studs");
    require(
      n("ringDiameter") <= n("outerDiameter") && n("dialDiameter") < n("ringDiameter"),
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
      ), required : true
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

  /** The whole device as one solid, with plain studs and nuts: their coincident surfaces cannot keep two thread textures. */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var parts:Array<Part> = [];
    for (spec in bodies()) parts.push(spec.id
      == "studs" ? studSolid(false) : spec.id == "nuts" ? nutSolid(false) : spec.build());
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
    return [new DeviceBodySpec("housing", "hand wheel housing", "ABS black", function() {
      return Solids.union([
        Solids.named(Part.cylinderSpan(n("outerDiameter") / 2, 0.0, flange), "flange"),
        Solids.named(Part.cylinderSpan(n("bossDiameter") / 2, rear, 0.0), "boss")
      ]);
    }
    ),
      new DeviceBodySpec("studs", "hand wheel mounting studs", "steel 10.9", () -> studSolid(true)), new DeviceBodySpec(
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
    ), new DeviceBodySpec("wheel", "hand wheel", materialSpec(), function() {
      var parts:Array<Part> = [Solids.named(
        Part.revolve([
          {r: 0.0, z: flange},
          {r: n("outerDiameter") / 2, z: flange},
          {r: n("ringDiameter") / 2, z: flange + n("coneHeight")},
          {r: n("ringDiameter") / 2, z: top},
          {r: n("dialDiameter") / 2, z: top},
          {r: n("dialDiameter") / 2, z: top - n("dialRecess")},
          {r: 0.0, z: top - n("dialRecess")}
        ]),
        "wheel"
      )];
      if (flag("handle")) {
        var from = top - n("dialRecess");
        parts.push(Solids.named(
          Part.cylinderSpan(n("handleDiameter") / 2, from, top + n("handleProtrusion"), 0.0, -n("handleRadius")),
          "handle"
        ));
      }
      return Solids.union(parts);
    }
    )];
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
