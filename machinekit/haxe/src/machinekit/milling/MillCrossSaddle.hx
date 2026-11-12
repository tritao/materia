package machinekit.milling;

import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames as F;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Cross slide with a bottom Z datum and optional through-Y nut tunnel. */
class MillCrossSaddle extends MachineComponent {
  public static inline var BLOCK_SEAT:Float = 12;
  public static inline var BLOCK_SCREW_LENGTH:Float = 16;
  public static inline var WRENCH_HEIGHT:Float = 34;
  static final recipe = new machinekit.cache.Once<ComponentType>();

  public static function parameters():Array<ComponentParameter> return [
    P.length("width", 450),
    P.length("depth", 174),
    P.length("height", 80),
    P.flag("ribbed", false),
    P.flag("blockAccess", false),
    P.flag("nutBoss", false),
    P.flag("nutServiceAccess", false),
    P.length("guideSpacing", 220),
    P.length("blockSpan", 50.7),
    P.length("blockHolePitchB", 26),
    P.length("blockHolePitchC", 26),
    P.length("blockClearance", 4.5),
    P.length("crossRailSpacing", 160),
    P.length("crossRailLength", 450),
    P.length("crossRailWidth", 15),
    P.length("bore", 54),
    P.length("boreZ", 40),
    P.flag("motorPocket", true)
  ];

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.cross-saddle",
      parameters(),
      v -> new MillCrossSaddle(v)
    ).numbered(_ -> "MILL-CROSS-SADDLE"));
  }

  public function new(?values:ComponentValues) {
    var v = P.resolve(parameters(), values);
    if (v.number("bore") + 20 > v.number("height")
      || v.number("bore") >= v.number("width")) throw "Saddle tunnel leaves insufficient material";
    if (v.boolean("ribbed") && (v.number("width") < 450 || v.number("depth") < 174
      || v.number("height") < 56)) throw "Ribbed saddle needs continuous chords and end webs";
    if (v.boolean("ribbed") && (v.number("guideSpacing") < 160 || v.number("guideSpacing") > v.number("width") - 80
      || v.number("blockHolePitchC") <= 0 || v.number("blockHolePitchC") > v.number("guideSpacing") / 2
      || v.number("blockHolePitchB") <= 0 || v.number("blockSpan")
      <= v.number("blockHolePitchB") / 2 + 17)) throw "Ribbed saddle cannot retain its guide pads and bolt wells";
    if (v.boolean("blockAccess") && (!v.boolean("ribbed")
      || v.number("blockSpan") + v.number("blockHolePitchB") / 2 + 10 >= v.number("depth") / 2
      || v.number("guideSpacing") / 2 - (v.number("blockHolePitchC") + 20) / 2 - 12
      <= 45)) throw "Block access pockets need ribbed guide pads, end clearance and retained central webs";
    if (v.boolean("nutBoss")
      && (!v.boolean("ribbed") || !v.boolean("blockAccess") || v.number("height") < 72 || v.number("boreZ") != 0
      || v.number("guideSpacing") / 2 - (v.number("blockHolePitchC") + 20) / 2 < MillYNutMount.BOSS_WIDTH
      / 2 + 12)) throw "Nut boss needs the underside screw datum and separated guide service webs";
    if (v.boolean("nutServiceAccess") && !v.boolean("nutBoss")) throw "Nut service notch requires reinforced nut boss";
    super("MILL-CROSS-SADDLE", "Machined cross saddle", "cast iron", false, v);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("top", Face, Solids.axial(0, 0, v.number("height")));
    if (v.boolean("nutServiceAccess")) addFacet(makeInterfaces());
  }

  function makeInterfaces():machinekit.design.DesignInterfaces {
    var holes:Array<machinekit.design.DesignInterfaces.HoleInterface> = [],
      d = parameterNumber("depth"), h = parameterNumber("height");
    for (p in MillYNutMount.anchors()) holes.push({
      id: 'anchor${p.x}-${p.y}',
      frame: F.fromRotationMatrix(p.x, -d / 2, p.y, [1, 0, 0, 0, 0, 1, 0, -1, 0]),
      radius: 2.5,
      depth: 24.5,
      minWall: 5,
      minClearance: 0.4,
      counterbore: null,
      approach: null,
      thread: {
        spec: machinekit.standard.FastenerThreads.coarse(6).mating(),
        tapDrill: 5,
        usableDepth: 22,
        cylindricalDepth: 23,
        point: 1.5,
        stock: MillYNutMount.BOSS_DEPTH,
        minEngagement: 9,
        minTipGap: 2,
        minBack: 3
      }
    });
    var material:Array<machinekit.design.DesignRequirement.DesignEnvelope> = [];
    for (x in [-16.0, 16.0]) material.push({
      id: 'notch-cheek$x',
      frame: F.translation(x, -d / 2 + 8, 36),
      volume: BoxVolume(10, 16, h - 40)
    });
    for (side in [-1, 1]) material.push({
      id: 'cross-rail-land$side',
      frame: F.translation(0, side * parameterNumber("crossRailSpacing") / 2, h - 3),
      volume: BoxVolume(parameterNumber("crossRailLength"), parameterNumber("crossRailWidth"), 3)
    });
    return new machinekit.design.DesignInterfaces(holes, [], material);
  }
  override public function componentType():Null<ComponentType> return recipeType();

  /** Independent cover datums on the retained end webs and the front sole/deck. */
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace> = [], w = parameterNumber("width"), d = parameterNumber("depth"), h = parameterNumber("height");
    for (side in [-1, 1]) for (y in [-62.0, 62.0]) faces.push(BoltedFace.drilledToSuit(
      'coverEnd$side-$y', BoltedFace.frameOf({x: side * w / 2, y: y, z: h - 29},
        {x: -side, y: 0, z: 0}, {x: 0, y: 1, z: 0}), 24, 40, 13, 20, CastIron));
    return faces;
  }
  override public function machinable():Bool return true;
  override public function hasGeometry():Bool return true;

  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    PartRecordings.recipeInputs(this, b);
    var guide = parameterNumber("guideSpacing") / 2;
    var w = parameterNumber("width"), d = parameterNumber("depth"), h = parameterNumber("height");
    var pitchB = parameterNumber("blockHolePitchB"), pitchC = parameterNumber("blockHolePitchC");
    var padHalf = Math.max(20, (pitchC + parameterNumber("blockClearance") + 8) / 2);
    var ribbed = values().boolean("ribbed"), access = values().boolean("blockAccess"), boss = values().boolean("nutBoss");
    var service = values().boolean("nutServiceAccess");
    var tools:Array<Feature> = [], labels:Array<String> = [];
    function cut(tool:Feature, label:String):Void {
      tools.push(tool);
      labels.push(label);
    }
    /** A box `sx` x `sy` x `sz` centred on (x, y), rising from z. */
    function block(sx:Float, sy:Float, sz:Float, x:Float, y:Float, z:Float):Feature
      return b.boxBetween(x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2, z, z + sz);
    function alongY(r:Float, y0:Float, y1:Float, x:Float, z:Float):Feature return b.cylinderAlong(new Vector(x, 0, z), Vector.Y(), r, y0, y1);
    function alongX(r:Float, x0:Float, x1:Float, y:Float, z:Float):Feature return b.cylinderAlong(new Vector(0, y, z), Vector.X(), r, x0, x1);
    function up(r:Float, z0:Float, z1:Float, x:Float, y:Float):Feature return b.cylinderAlong(new Vector(x, y, 0), Vector.Z(), r, z0, z1);
    function sided(v:Float):String return v < 0 ? "left" : "right";
    function ended(v:Float):String return v < 0 ? "front" : "rear";
    if (parameterNumber("bore") > 0) cut(alongY(parameterNumber("bore") / 2, -d / 2 - 1, d / 2 + 1, 0, parameterNumber("boreZ")), "nut tunnel");
    for (x in [-35.0, 35.0]) for (z in [-20.0, 20.0]) cut(alongY(2.5, d / 2 - 16, d / 2 + 1, x, h / 2 + z), 'rear tap ${sided(x)} ${z < 0 ? "low" : "high"}');
    if (values().boolean("motorPocket")) {
      // Square Leadshine flange/body and front-facing connector envelope.
      cut(block(102, 66, 66, -170, 0, 7), "motor pocket");
      // The connector envelope runs on through the end wall, so the motor (with its mounting plate) withdraws outwards along X.
      cut(block(107, 20, 24, -172.5, -38, 28), "connector channel");
      // Beyond the pocket the bore only clears the coupling; it stays inside the pocket's square so the pocket keeps its back face.
      cut(alongX(32, -226, -64, 0, 40), "motor bore");
      cut(block(12, 86, 82, MillXYStageLayout.X_MOTOR_FACE - 6, 0, -1), "motor plate slot");
      cut(block(17, 40, 42, -218.5, 0, 40), "end wall window");
      for (y in [-32.0, 32.0]) for (z in [10.0, 70.0])
        cut(alongX(2.5, MillXYStageLayout.X_MOTOR_FACE - 1, MillXYStageLayout.X_MOTOR_FACE + 13, y, z), 'motor plate tap ${ended(y)} ${z < 40 ? "low" : "high"}');
    }
    for (x in [-32.0, 32.0]) cut(alongY(2.5, -d / 2 - 1, -d / 2 + 16, x, 12), 'front tap ${sided(x)}');
    if (ribbed) {
      var cells:Array<Array<Float>>;
      if (access) {
        var pocketWidth = pitchC + 20, inner = parameterNumber("blockSpan") - pitchB / 2 - 10;
        // Side-entry service pockets are centered on the real block pattern.
        for (sx in [-1, 1]) for (sy in [-1, 1])
          cut(accessPocket(b, sx * guide, pocketWidth, sy < 0 ? -d / 2 - 1 : inner, sy < 0 ? -inner : d / 2 + 1, h), 'block access ${sided(sx)} ${ended(sy)}');
        // Separate lightening cells leave full-height webs beside each guide pad.
        cells = [[-w / 2 + 40, -guide - pocketWidth / 2 - 12], [-guide + pocketWidth / 2 + 12, -45.0], [45.0, guide - pocketWidth / 2 - 12],
          [guide + pocketWidth / 2 + 12, w / 2 - 20]];
      } else {
        // Four bottom guide pads, continuous X guide chords, transverse end webs and the central nut spine remain joined by
        // a 14 mm sole.
        cells = [[-w / 2 + 40, -guide - padHalf], [-guide + padHalf, -45.0], [45.0, guide - padHalf], [guide + padHalf, w / 2 - 20]];
      }
      for (side in [-1, 1]) for (k => cell in cells) if (cell[1] > cell[0])
        cut(block(cell[1] - cell[0], d / 2 - 28, h - 28, (cell[0] + cell[1]) / 2, side * (d / 4 + 14), 14), 'lightening cell ${ended(side)} ${k + 1}');
      for (k => cell in [[-w / 2 - 1, -guide - padHalf], [-guide + padHalf, -45.0], [45.0, guide - padHalf], [guide + padHalf, w / 2 + 1]])
        cut(block(cell[1] - cell[0], d + 2, 5, (cell[0] + cell[1]) / 2, 0, -1), 'sole relief ${k + 1}');
      for (x in [-guide, guide]) cut(block(2 * padHalf, 24, 5, x, 0, -1), 'pad split ${sided(x)}');
      for (sx in [-1, 1]) for (sy in [-1, 1]) for (dx in [-pitchC / 2, pitchC / 2]) for (dy in [-pitchB / 2, pitchB / 2]) {
        var x = sx * guide + dx, y = sy * parameterNumber("blockSpan") + dy;
        var at = '${sided(sx)} ${ended(sy)} ${dx < 0 ? "inner" : "outer"} ${dy < 0 ? "front" : "rear"}';
        cut(up(parameterNumber("blockClearance") / 2, -1, access ? BLOCK_SEAT + 1 : h + 1, x, y), 'block screw $at');
        if (!access) cut(up(5, 12, h + 1, x, y), 'block screw counterbore $at');
      }
    }
    var body:Feature = b.label(b.boxBetween(-w / 2, w / 2, -d / 2, d / 2, 0, h), "Stock · Saddle");
    if (!boss) return b.label(b.cutAll(body, tools, labels), "Tunnel, pockets and screw holes");
    // Add the load-transfer ribs after lightening cuts, then retain all interfaces.
    var parts:Array<Feature> = [
      b.label(b.cutAll(body, tools, labels), "Tunnel, pockets and screw holes"),
      b.label(block(MillYNutMount.BOSS_WIDTH, MillYNutMount.BOSS_DEPTH, h, 0, -d / 2 + MillYNutMount.BOSS_DEPTH / 2, 0), "Nut boss")
    ];
    var ribX = guide - (pitchC + 20) / 2;
    for (sx in [-1, 1]) for (sy in [-1, 1]) {
      var profile = [new Vector(sx * 46, sy * d / 2, 0), new Vector(sx * 58, sy * d / 2, 0), new Vector(sx * ribX, sy * 40, 0), new Vector(sx * (ribX - 12), sy * 40, 0)];
      parts.push(b.label(b.extrude(b.polygon(profile, new cadkit.modeling.Plane(new Vector(0, 0, BLOCK_SEAT), Vector.X(), Vector.Z())), h - 14 - BLOCK_SEAT), 'Load rib ${sided(sx)} ${ended(sy)}'));
    }
    var ribbedBody = b.label(b.unionAll(parts), "Saddle with nut boss");
    tools = [];
    labels = [];
    cut(alongY(parameterNumber("bore") / 2, -d / 2 - 1, d / 2 + 1, 0, parameterNumber("boreZ")), "nut tunnel through boss");
    if (service) {
      var drilling = machinekit.design.DesignInterfaces.of(this).recordDrilling(b);
      for (i in 0...drilling.tools.length) cut(drilling.tools[i], drilling.labels[i]);
    } else for (p in MillYNutMount.anchors()) cut(alongY(2.5, -d / 2 - 1, -d / 2 + 22, p.x, p.y), 'nut anchor ${sided(p.x)} ${p.y < 30 ? "low" : "high"}');
    for (p in MillYNutMount.pilots()) {
      if (p.x < 0) cut(alongY(3.05, -d / 2 - 1, -d / 2 + 10, p.x, p.y), "nut pilot left");
      else {
        // The right pilot is slotted 0.4 mm along X so the nut mount locates on one pin.
        cut(alongY(3.05, -d / 2 - 1, -d / 2 + 10, p.x - 0.2, p.y), "nut pilot slot left end");
        cut(alongY(3.05, -d / 2 - 1, -d / 2 + 10, p.x + 0.2, p.y), "nut pilot slot right end");
        cut(block(0.4, 11, 6.1, p.x, -d / 2 + 4.5, p.y - 3.05), "nut pilot slot");
      }
    }
    if (service) {
      cut(block(20, 2, h - 23, 0, -d / 2, 24), "service notch");
      cut(up(10, 24, h + 1, 0, -d / 2 + 1), "service bore");
    } else cut(up(5, 24, h + 1, 0, -d / 2 + 5), "grease bore");
    return b.label(b.cutAll(ribbedBody, tools, labels), service ? "Nut tunnel, anchors, pilots and service notch" : "Nut tunnel, anchors, pilots and grease bore");
  }
  /** Rounded XZ profile extruded from the side face to the inner bolt row. */
  static function accessPocket(b:DocumentBuilder, x:Float, width:Float, y0:Float, y1:Float, height:Float):Feature {
    var floor = BLOCK_SEAT, top = height - 14, radius = 4.0;
    var parts:Array<Feature> = [
      b.boxBetween(x - width / 2 + radius, x + width / 2 - radius, y0, y1, floor, top),
      b.boxBetween(x - width / 2, x + width / 2, y0, y1, floor + radius, top - radius)
    ];
    for (sx in [-1, 1]) for (z in [floor + radius, top - radius])
      parts.push(b.cylinderAlong(new Vector(x + sx * (width / 2 - radius), 0, z), Vector.Y(), radius, y0, y1));
    return b.unionAll(parts);
  }
}
