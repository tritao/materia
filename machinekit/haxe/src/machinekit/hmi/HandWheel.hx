package machinekit.hmi;

import cadkit.modeling.Footprint;
import cadkit.modeling.FootprintGeometry;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as R;
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
  function rearBody():Footprint return Circle(n("studCircleDiameter") + 8.0);

  /**
   * The housing (flange, boss, studs, terminal strip) fixed to the panel, and the wheel that turns on it: the cone, grip
   * ring, dial and spinner handle. The wheel's rotation is the encoder's angle.
   */
  override public function bodies():Null<Array<DeviceBodySpec>> {
    var top = bodyHeight(), flange = n("flangeThickness");
    return [new DeviceBodySpec("housing", "hand wheel housing", "ABS black", function() {
      var parts:Array<Part> = [Solids.named(Part.cylinderSpan(n("outerDiameter") / 2, 0.0, flange), "flange")];
      parts.push(Solids.named(Part.cylinderSpan(n("bossDiameter") / 2, -n("bossDepth"), 0.0), "boss"));
      var studs = count("studCount");
      for (i in 0...studs) {
        var angle = Math.PI / 2 + i * 2 * Math.PI / studs;
        parts.push(Part.cylinderSpan(
          n("studDiameter") / 2,
          -n("studLength"),
          0.0,
          n("studCircleDiameter") / 2 * Math.cos(angle),
          n("studCircleDiameter") / 2 * Math.sin(angle)
        ));
      }
      var strip = Part.box(n("terminalLength"), n("terminalWidth"), n("terminalDepth") - n("bossDepth"));
      parts.push(Solids.named(shifted(strip, -n("terminalDepth")), "terminals"));
      return Solids.union(parts);
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

  static function shifted(part:Part, z:Float):Part {
    var moved = part.translated(new Vector(0, 0, z));
    part.close();
    return moved;
  }
}
