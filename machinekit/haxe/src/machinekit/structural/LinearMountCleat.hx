package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.AxisBuilder;
import machinekit.assembly.PosedParts;

/** Machined C45 angle cleat, not a welded overlap or virtual connection.
 * Local base spans X=0..28, Y=±32 (end pieces shifted 3 mm inward); web X=0..11. Source dimensions are engineering
 * inputs. Two recessed M6 base clearances and two recessed M6 web clearances have real connectors. */
class LinearMountCleat extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public final height:Float;
  public final bodyShift:Float;
  public static inline final WALL = 11.0;
  public static inline final BASE = 13.0;
  public static inline final BASE_WIDTH = 28.0;
  public static inline final BOLT_X = 20.0;
  public static inline final WEB_BOLT_LENGTH = 20.0;
  public static inline final BODY_LENGTH = 64.0;
  public static inline final WEB_RECESS = 2.6;
  public static inline function webWasherSeat():Float return WALL - WEB_RECESS;
  public static function webBoltSeat():Float return webWasherSeat()
    + machinekit.standard.FlatWasher.metric("M6").thickness;
  static function inputs():Array<ComponentParameter> return [
    P.length("height", 32),
    P.choice("endBias", ["middle", "front", "back"], "middle")
  ];
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(inputs(), overrides);
    height = v.number("height");
    bodyShift = switch (v.token("endBias")) {
      case "front":
        3.0;
      case "back":
        -3.0;
      case _:
        0.0;
    };
    if (!Math.isFinite(height) || height < 29 || height > 120) throw "Cleat web height must be 29..120 mm";
    super("LINEAR-MOUNT-CLEAT", "C45 discrete bolted mount cleat", "steel C45", false, v);
    addFacet(new EngineeringMetadata(new ComponentValues().setToken("errorsBasis", "Assumed").setToken("source", "").setToken("ratingsBasis", "Stated")));
    lockManufacturingMaterial("steel C45");
    for (y in [-18.0, 18.0]) addConnector('base-' + y, Mount, Solids.axial(BOLT_X, y, BASE - 8 + 1.6));
    for (y in [-14.0, 14.0]) addConnector(
      'web-' + y,
      Mount,
      AxisBuilder.orient(webBoltSeat(), y, height - 10, [0.0, 1, 0], [1.0, 0, 0])
    );
    addConnector("base", Mount, Solids.axial(BASE_WIDTH / 2, bodyShift, 0));
    addConnector("receiver", Mount, AxisBuilder.orient(0, 0, height - 10, [0.0, 1, 0], [-1.0, 0, 0]));
  }
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.structural.linear-mount-cleat",
      inputs(),
      v -> new LinearMountCleat(v),
      false,
      false,
      null,
      false
    ).numbered(v -> 'LINEAR-MOUNT-CLEAT-H${Dimension.format(v.number("height"))}').procured(_ -> Make("machining")));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  public function stockRegions():Array<Array<Float>> return [
    [0.0, BASE_WIDTH, -BODY_LENGTH / 2 + bodyShift, BODY_LENGTH / 2 + bodyShift, 0, BASE],
    [0.0, WALL, -BODY_LENGTH / 2 + bodyShift, BODY_LENGTH / 2 + bodyShift, BASE, height]
  ];
  /** Exact positive cutting tools, shared with local section integration. */
  public function cutBores():Array<{frame:materia.assembly.AssemblyRecord.AssemblyFrame, diameter:Float, depth:Float}> {
    var result:Array<{frame:materia.assembly.AssemblyRecord.AssemblyFrame, diameter:Float, depth:Float}> = [];
    for (y in [-18.0, 18.0]) {
      result.push({frame: F.translation(BOLT_X, y, -1), diameter: 6.6, depth: height + 2});
      result.push({frame: F.translation(BOLT_X, y, BASE - 8), diameter: 13, depth: 8});
    }
    for (y in [-14.0, 14.0]) {
      result.push({
        frame: AxisBuilder.orient(-1, y, height - 10, [0.0, 1, 0], [1.0, 0, 0]),
        diameter: 6.6,
        depth: WALL + 2
      });
      result.push({
        frame: AxisBuilder.orient(webWasherSeat(), y, height - 10, [0.0, 1, 0], [1.0, 0, 0]),
        diameter: 12.5,
        depth: WEB_RECESS
      });
    }
    return result;
  }
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var parts:Array<Part> = [];
    for (k => r in stockRegions()) {
      var raw = Part.box(r[1] - r[0], r[3] - r[2], r[5] - r[4]);
      parts.push(Solids.named(raw.translated(new Vector((r[0] + r[1]) / 2, (r[2] + r[3]) / 2, r[4])), k == 0 ? "base" : "wall"));
      raw.close();
    }
    var stock = Solids.union(parts), tools:Array<Part> = [];
    for (k => h in cutBores()) {
      var raw = Part.cylinder(h.diameter / 2, h.depth);
      try {
        tools.push(Solids.named(PosedParts.moved(raw, h.frame), "bore" + (k + 1)));
        raw.close();
      } catch (e:Dynamic) {
        raw.close();
        stock.close();
        for (t in tools) t.close();
        throw e;
      }
    }
    return Solids.cut(stock, tools);
  }
  /** Design-envelope engineering screen, N/mm/MPa. No fatigue or final J7
     * reaction qualification. Minimum installed preload must be verified. */
  public function connection(tangentialN:Float, radialN:Float, axialN:Float): {
    bearingMPa: Float,
    netMPa: Float,
    webBendingMPa: Float,
    boltTensionN: Float,
    boltTensionCapacityN: Float,
    slipDemandN: Float,
    slipCapacityN: Float,
    washerBearingMPa: Float,
    washerBearingLimitMPa: Float,
    punchingDemandN: Float,
    punchingCapacityN: Float,
    boltInteraction: Float,
    baseTearoutCapacityN: Float,
    baseBearingMPa: Float,
    webThreadEngagementMm: Float,
    webThreadStrippingCapacityN: Float,
    pass: Bool
  } {
    for (v in [tangentialN, radialN, axialN]) if (!Math.isFinite(v) || v < 0) throw "Invalid cleat load envelope";
    var force = Math.sqrt(tangentialN * tangentialN + axialN * axialN);
    // Conservative net-section bound: debit both washer recesses and
    // both key corridors together, even though the longer toe moves the
    // key corridors outside the web. No gross-section stiffness credit.
    var netWidth = BODY_LENGTH - 2 * 12.5 - 2 * 6.6, effectiveWall = WALL - WEB_RECESS;
    var bearing = force / (2 * 6 * effectiveWall), net = force / (netWidth * effectiveWall);
    // Retain the earlier 20 mm envelope plus the 11 mm added toe.
    // The local model must replace this bound with actual reaction arms.
    var moment = force * (31 + Math.abs(bodyShift)) + radialN * (height - 10);
    var bending = 6 * moment / (netWidth * effectiveWall * effectiveWall);
    var tension = 2 * (radialN / 2 + moment / 24); // factor-two prying allowance
    var area = 20.1, proof = 830.0, preload = 0.7 * proof * area;
    var capacity = 0.9 * 1000 * area / 1.25, shearCapacity = 0.6 * 1000 * area / 1.25;
    var slip = Math.max(0, 2 * preload - 2 * tension) * 0.1;
    var washerArea = Math.PI * (6 * 6 - 3.3 * 3.3), punchDemand = preload + tension;
    var washerBearing = punchDemand / washerArea;
    // Controlled-preload service bearing stays elastic at assumed Fy=235;
    // punching and external-load net/bending checks retain factor 1.5.
    var punchCapacity = Math.PI * 13 * (BASE - 8) * 235 / Math.sqrt(3) / 1.5;
    // The recessed head leaves only 1.5 mm to the base's narrow edge.
    // Check the two remaining shear planes per bolt at the counterbore
    // floor, rather than claiming the gross 13 mm thickness carries them.
    var edgeLigament = Math.min(BOLT_X, BASE_WIDTH - BOLT_X) - 13 / 2, floor = BASE - 8;
    var tearout = 2 * 2 * edgeLigament * floor * 235 / Math.sqrt(3) / 1.5;
    var baseBearing = force / (2 * 6 * floor);
    var interaction = tension / capacity + (force / 2) / shearCapacity;
    var engagement = WEB_BOLT_LENGTH - webBoltSeat();
    var rm = materia.casting.InsertMaterial.InsertMaterials.specification(C45Bright).minimumTensileStrengthMPa;
    // Conservative half-cylinder thread shear at (major - pitch), with
    // assumed ultimate shear 0.6 Rm and factor 1.25. Installation preload
    // is included; no friction or pin credit is needed for stripping.
    var threadCapacity = 0.5 * Math.PI * (6 - 1) * engagement * (0.6 * rm) / 1.25;
    return {
      bearingMPa: bearing,
      netMPa: net,
      webBendingMPa: bending,
      boltTensionN: tension,
      boltTensionCapacityN: capacity,
      slipDemandN: force,
      slipCapacityN: slip,
      washerBearingMPa: washerBearing,
      washerBearingLimitMPa: 235,
      punchingDemandN: punchDemand,
      punchingCapacityN: punchCapacity,
      boltInteraction: interaction,
      baseTearoutCapacityN: tearout,
      baseBearingMPa: baseBearing,
      webThreadEngagementMm: engagement,
      webThreadStrippingCapacityN: threadCapacity,
      pass: bearing <= 235 / 1.5 && net <= 235 / 1.5 && bending <= 235 / 1.5 && tension <= capacity && force <= slip
      && force <= tearout && baseBearing <= 235 / 1.5 && tension < preload && punchDemand < proof * area
      && washerBearing <= 235 && punchDemand <= punchCapacity && interaction <= 1 && punchDemand <= threadCapacity
    };
  }
}
