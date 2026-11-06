package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hydraulic expanding mandrel. Reference row is Assumed; dimensions are simplified envelopes.
    Pressure naming: `oilPressure`/`maxOilPressure` are the pressures of the oil inside the sleeve (MPa range, set
    by the air-to-oil booster). They are NOT the contact pressure on the cup bore, which is `contactPressureMin/Max`
    (0.5-1.5 MPa): the sleeve wall and the loading clearance absorb most of the oil pressure.
    Derivations and evidence: examples/flexspline-hobber/workholding/loads/README.md (sections 1-3) and
    workholding/fem/README.md (cases 1-2). */
class ExpandingMandrel extends MachineComponent {
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.quantity("outside", 59, "mm"),
    P.quantity("bore", 12, "mm"),
    P.quantity("height", 50, "mm"),
    P.length("baseDiameter", 0),
    P.length("baseHeight", 0),
    P.length("baseBore", 0),
    P.length("baseBoreDepth", 0),
    P.length("basePilotDiameter", 0),
    P.length("basePilotDepth", 0),
    P.length("baseBoltPcd", 0),
    P.quantity("workSeat", 30, "mm"),
    P.quantity("stopDiameter", 0, "mm"),
    P.flag("separateSleeve", false),
    P.quantity("expansion", 0.05, "mm"),
    P.quantity("sleeveWall", 1.5, "mm"),
    P.quantity("sleeveLength", 16, "mm"),
    P.quantity("sleeveStart", 26, "mm"),
    P.quantity("sleeveYoungs", 210000, "N/mm²"),
    P.quantity("sleevePoisson", 0.3, ""),
    P.quantity("oilCavity", 0.5, "mm"),
    P.quantity("oilBulkModulus", 1500, "N/mm²"),
    P.quantity("contactStiffnessFactor", 1, ""),
    P.quantity("radialFitGap", 0.025, "mm"),
    // Oil pressure inside the sleeve (operating / proof). Not the contact pressure on the cup.
    P.quantity("oilPressure", 10000000, "Pa"),
    P.quantity("maxOilPressure", 15000000, "Pa"),
    // Contact pressure on the cup bore, over the whole loading-clearance range (loads/README 2).
    P.quantity("contactPressureMin", 500000, "Pa"),
    P.quantity("contactPressureMax", 1500000, "Pa"),
    // Torque the clamp must transmit at the bore: 2 x 1.49 N m bound (maxTableTorqueBound), mu 0.08 (loads/README 1).
    P.quantity("holdingTorque", 3.0, "N m"),
    // Pull-off holding force: 2 x 14 N assumed axial load (loads/README 1).
    P.quantity("holdingAxialForce", 28, "N"),
    // Pressure must start at the cup's open edge and run past the tooth band by this much (fem/README case 1:
    // about 3 shell lengths 1/beta = 3.3 mm keeps band taper under ~4% of the mean).
    P.quantity("pressedMargin", 8, "mm"),
    // Uniform radial cup expansion allowed at working pressure: a size offset, compensated by hob depth
    // (6.8 um/MPa of contact pressure x 1.5 MPa = 10 um; loads/README 2).
    P.quantity("uniformExpansionLimit", 0.010, "mm"),
    // Clamped-state error bounds. Eccentricity is the offset amplitude (runout TIR = 2x).
    P.quantity("eccentricity", 0.0015, "mm"),
    P.quantity("reclampRepeatability", 0.0015, "mm"),
    P.quantity("tilt", 0.002, "mm"),
    P.quantity("tiltLength", 8, "mm"),
    // Non-uniform (n >= 2) part of the clamped bore displacement, peak.
    P.quantity("lobedDistortion", 0.0015, "mm"),
    // Max-min of the circumferential-mean expansion over the 8 mm tooth band (flank-line error = 0.34 x this).
    P.quantity("bandExpansionVariation", 0.003, "mm"),
    // Roundness of the clamped sleeve OD; the cup copies it at about 0.95 um/um (fem/README case 2).
    P.quantity("clampedRoundness", 0.0015, "mm")
  ]);
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.hobbing.expanding-mandrel",
      parameters(),
      v -> new ExpandingMandrel(v)
    ).numbered(v -> 'MANDREL-ASSUMED-D${Dimension.format(v.number("outside"))}-E${Dimension.format(v.number("expansion"))}')
      .procured(_ -> NotSupplied).requiringInputs(["outside", "bore", "height", "workSeat", "expansion"], ["oilPressure", "maxOilPressure"]));
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues
    > return EngineeringMetadata.assumedCatalog(
      "Hydraulic expanding mandrel",
      "HYDRAULIC-MANDREL-59",
      parameters(),
      "machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)"
    );
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    super("HYDRAULIC-MANDREL-59", "Hydraulic expanding mandrel (reference envelope)", "steel", false, v);
    if (v.get("material") != null) setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("axis", Axis, Solids.axial(0, 0, 0));
    addConnector("top", Mount, Solids.axial(0, 0, parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(
      v,
      [
        {name: "eccentricity", unit: "mm"},
        {name: "reclampRepeatability", unit: "mm"},
        {name: "tilt", unit: "mm"},
        {name: "lobedDistortion", unit: "mm"},
        {name: "bandExpansionVariation", unit: "mm"},
        {name: "clampedRoundness", unit: "mm"}
      ]
    ));
    if (parameterNumber("oilPressure") <= 0 || parameterNumber("oilPressure") > parameterNumber("maxOilPressure")
      || parameterNumber("contactPressureMin") <= 0
      || parameterNumber("contactPressureMax") < parameterNumber("contactPressureMin")
      || parameterNumber("holdingTorque") <= 0 || parameterNumber("holdingAxialForce") <= 0
      || parameterNumber("pressedMargin") < 0 || parameterNumber("uniformExpansionLimit") <= 0
      || parameterNumber("tiltLength") <= 0
      || parameterNumber("workSeat") >= parameterNumber("height")
      || parameterNumber("expansion") <= 0) throw "invalid mandrel ratings";
    for (key in ["sleeveWall", "sleeveLength", "sleeveYoungs", "oilCavity",
      "oilBulkModulus", "contactStiffnessFactor"]) if (parameterNumber(key) <= 0) throw 'invalid mandrel support $key';
    if (parameterNumber("sleeveStart") + parameterNumber("sleeveLength") > parameterNumber("height")
      || parameterNumber("sleeveWall") >= parameterNumber("outside") / 2
      || parameterNumber("radialFitGap") < 0 || parameterNumber("sleevePoisson") <= 0
      || parameterNumber("sleevePoisson") >= 0.5) throw "invalid sleeve/contact design";
    addConnector("work", Mount, Solids.axial(0, 0, parameterNumber("workSeat")));
    // Separate axial envelopes preserve real external shoulders. A single hull
    // bridges the large base to the sleeve cap and invents material at the cut.
    // Internal bores/oil cavities remain conservatively filled in collision only.
    var hulls:Array<Array<Float>> = [], r = parameterNumber("outside") / 2,
      baseHeight = parameterNumber("baseHeight"), height = parameterNumber("height");
    function section(radius:Float, lo:Float, hi:Float):Void {
      if (hi > lo) hulls.push(CollisionHullFacet.cylinder(radius, lo, hi));
    }
    if (baseHeight > 0) {
      if (parameterNumber("baseBoltPcd") > 0) {
        section(parameterNumber("baseDiameter") / 2 - 25, 0, baseHeight);
        for (i in 0...6) {
          var a = (i + 0.5) * Math.PI / 3, d = parameterNumber("baseBoltPcd") / 2, h = CollisionHullFacet.cylinder(
            15,
            0,
            baseHeight
          );
          for (j in 0...Std.int(h.length / 3)) {
            h[j * 3] += d * Math.cos(a);
            h[j * 3 + 1] += d * Math.sin(a);
          }
          hulls.push(h);
        }
      } else section(parameterNumber("baseDiameter") / 2, 0, baseHeight);
    }
    if (v.boolean("separateSleeve")) {
      var start = parameterNumber("sleeveStart"), end = start + parameterNumber("sleeveLength");
      section(r, baseHeight, start);
      section(r - parameterNumber("sleeveWall") - parameterNumber("oilCavity"), Math.max(baseHeight, start), end);
      section(r, Math.max(baseHeight, end), height);
    } else section(r, baseHeight, height);
    if (parameterNumber("stopDiameter") > 0) section(
      parameterNumber("stopDiameter") / 2,
      parameterNumber("workSeat") - 2,
      parameterNumber("workSeat")
    );
    addFacet(new CollisionHullFacet(hulls));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  /** Oil pressure inside the sleeve, Pa. */
  public var oilPressure(get, never):Float;
  function get_oilPressure():Float return parameterNumber("oilPressure");
  public var eccentricity(get, never):Float;
  function get_eccentricity():Float return parameterNumber("eccentricity");
  /** Pressure must start at or below the cup's open edge `rimStart` (work height) and cover the band plus `pressedMargin`. */
  public function pressesBand(rimStart:Float, band:Float):Bool {
    var start = parameterNumber("sleeveStart");
    return start <= rimStart + 1e-9 && start + parameterNumber("sleeveLength") >= rimStart + band + parameterNumber("pressedMargin") - 1e-9;
  }

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):cadkit.modeling.Part {
    var stem = ComponentEnvelopes.annular(values());
    if (parameterNumber("stopDiameter") > 0) stem = Solids.union([
      stem,
      Solids.cut(
        cadkit.modeling.Part.cylinderSpan(
          parameterNumber("stopDiameter") / 2,
          parameterNumber("workSeat") - 2,
          parameterNumber("workSeat")
        ),
        [cadkit.modeling.Part.cylinderSpan(
          parameterNumber("bore") / 2,
          parameterNumber("workSeat") - 2.1,
          parameterNumber("workSeat") + 0.1
        )]
      )
    ]);
    if (values().boolean("separateSleeve")) stem = Solids.cut(
      stem,
      [Solids.cut(
        cadkit.modeling.Part.cylinderSpan(
          parameterNumber("outside") / 2 + 0.1,
          parameterNumber("sleeveStart"),
          parameterNumber("sleeveStart") + parameterNumber("sleeveLength")
        ),
        [cadkit.modeling.Part.cylinderSpan(
          parameterNumber("outside") / 2 - parameterNumber("sleeveWall") - parameterNumber("oilCavity"),
          parameterNumber("sleeveStart") - 0.1,
          parameterNumber("sleeveStart") + parameterNumber("sleeveLength") + 0.1
        )]
      )]
    );
    if (parameterNumber("baseHeight") == 0) return stem;
    if (parameterNumber("baseDiameter") < parameterNumber("outside")
      || parameterNumber("baseHeight") >= parameterNumber("workSeat")) throw "invalid integral mandrel base";
    var outerBase:cadkit.modeling.Part;
    if (parameterNumber("baseBoltPcd") > 0) {
      var pieces = [cadkit.modeling.Part.cylinderSpan(
        parameterNumber("baseDiameter") / 2 - 25,
        0,
        parameterNumber("baseHeight")
      )];
      for (i in 0...6) {
        var a = (i + 0.5) * Math.PI / 3, r = parameterNumber("baseBoltPcd") / 2;
        pieces.push(cadkit.modeling.Part.cylinderSpan(
          15,
          0,
          parameterNumber("baseHeight"),
          r * Math.cos(a),
          r * Math.sin(a)
        ));
      }
      outerBase = Solids.union(pieces);
    } else outerBase = cadkit.modeling.Part.cylinderSpan(
      parameterNumber("baseDiameter") / 2,
      0,
      parameterNumber("baseHeight")
    );
    var base = Solids.cut(
      outerBase,
      [cadkit.modeling.Part.cylinderSpan(parameterNumber("bore") / 2, -0.1, parameterNumber("baseHeight") + 0.1)]
    );
    var complete = Solids.union([stem, base]);
    if (parameterNumber("basePilotDiameter") > 0) {
      if (parameterNumber("basePilotDepth") <= 0 || parameterNumber("basePilotDiameter")
        >= parameterNumber("baseDiameter")) throw "invalid provisional fixture pilot";
      complete = Solids.union([
        complete,
        Solids.cut(
          cadkit.modeling.Part.cylinderSpan(
            parameterNumber("basePilotDiameter") / 2,
            -parameterNumber("basePilotDepth"),
            0.1
          ),
          [cadkit.modeling.Part.cylinderSpan(
            Math.max(parameterNumber("baseBore"), parameterNumber("bore")) / 2,
            -parameterNumber("basePilotDepth") - 1,
            1
          )]
        )
      ]);
    }
    var cuts:Array<cadkit.modeling.Part> = [];
    if (parameterNumber("baseBore") > 0) {
      if (parameterNumber("baseBoreDepth") <= 0 || parameterNumber("baseBoreDepth") >= parameterNumber("baseHeight")
        || parameterNumber("baseBore") >= parameterNumber("baseDiameter")) throw "invalid rotary union recess";
      cuts.push(cadkit.modeling.Part.cylinderSpan(
        parameterNumber("baseBore") / 2,
        -parameterNumber("basePilotDepth") - 1,
        parameterNumber("baseBoreDepth")
      ));
    }
    if (parameterNumber("baseBoltPcd") > 0) for (i in 0...6) {
      var angle = (i + 0.5) * Math.PI / 3, r = parameterNumber("baseBoltPcd") / 2;
      if (r + 4.5 >= parameterNumber("baseDiameter") / 2) throw "provisional M8 face holes outside fixture flange";
      cuts.push(cadkit.modeling.Part.cylinderSpan(
        4.5,
        -1,
        parameterNumber("baseHeight") + 1,
        r * Math.cos(angle),
        r * Math.sin(angle)
      ));
      cuts.push(cadkit.modeling.Part.cylinderSpan(
        7,
        parameterNumber("baseHeight") - 9,
        parameterNumber("baseHeight") + 1,
        r * Math.cos(angle),
        r * Math.sin(angle)
      ));
    }
    return cuts.length == 0 ? complete : Solids.cut(complete, cuts);
  }
}
