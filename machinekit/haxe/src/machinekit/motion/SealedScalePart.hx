package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameter;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.EngineeringMetadata;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.PrecisionErrors;
import machinekit.motion.SealedScaleData.SealedScaleMounting;
import machinekit.motion.SealedScaleData.SealedScaleSeries;
import materia.assembly.AssemblyDefinition.AssemblyEncoder;
import materia.assembly.AssemblyFrames;

/**
 * One side of a sealed absolute glass scale (`SealedScaleData`): the `housing`, fixed to the machine frame, or the
 * reading `head`, fixed to the moving carriage, with a running gap between them and no contact. Unlike the exposed
 * tape (`LinearScale`) the glass sits inside the housing, so the pair is a bought, closed measuring system with
 * BiSS-C feedback; both sides implement `EncoderPart`, so wiring an encoder on a joint is unchanged.
 *
 * CAD frame (both parts): travel along +Z, housing section across X (depth) and Y. Housing: z=0 at its start end,
 * y=0 at the face toward the head and y=height at its mounting face; connector `mount` (Face, +Y out of the mounting
 * face), `axis` at the measuring start (travel zero), `end1` and `end2` fixing holes (JFT-40). Head: z=0 at its centre,
 * y=0 at the face toward the housing; `mount` is the face away from the housing (-Y out), `hole1` and `hole2` the
 * M5 holes, `axis` at its centre. Place the head `gap` below the housing (y<0) at the carriage position.
 * The head's reading datum relative to its body is not given; its centre is used.
 */
class SealedScalePart extends MachineComponent implements EncoderPart {
  public static inline final HOUSING = "housing";
  public static inline final HEAD = "head";

  public final variant:String;
  public final series:SealedScaleSeries;
  public final measuringLength:Float;
  public final mountingPlate:Bool;
  public final accuracy:Float;
  public final resolutionNm:Float;

  public static function parameters():Array<ComponentParameter> return [
    P.choice("variant", [HOUSING, HEAD], HOUSING),
    P.choice("series", ["JFT-40", "JFT-10"], "JFT-40"),
    P.length("measuringLength", 120),
    P.flag("mountingPlate", false),
    P.choice("accuracy", ["3", "5"], "3"),
    P.choice("resolution", ["2.5", "5", "10"], "5"),
    P.choice("interface", ["BiSS-C"], "BiSS-C")
  ];

  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    variant = v.token("variant");
    series = SealedScaleData.series(v.token("series"));
    measuringLength = v.number("measuringLength");
    mountingPlate = v.boolean("mountingPlate");
    accuracy = Std.parseFloat(v.token("accuracy"));
    resolutionNm = Std.parseFloat(v.token("resolution"));
    var ok = false;
    for (ml in series.lengths) if (Math.abs(ml - measuringLength) < 1e-9) ok = true;
    if (!ok) throw '${series.series} has no ${measuringLength} mm measuring length; use SealedScaleData.standardLength';
    if (mountingPlate && series.overhangWithPlate == null) throw '${series.series} has no mounting plate';
    var from = series.plateRequiredFrom;
    if (!mountingPlate && from != null && measuringLength >= from) throw '${series.series} needs the mounting plate from ML $from';
    if (accuracy == 3 && measuringLength > series.maxMlFor3um) throw '${series.series} is +-3 um only up to ML ${series.maxMlFor3um}';
    var tag = '${series.series}-ML${Dimension.format(measuringLength)}${mountingPlate ? "P" : ""}-A${Dimension.format(accuracy)}-R${Dimension.format(resolutionNm)}-BISSC';
    super(
      variant == HOUSING ? tag : tag + "-HEAD",
      variant == HOUSING
      ? 'Sealed absolute linear scale ${series.series}, ML ${Dimension.format(measuringLength)} mm, +-${Dimension.format(accuracy)} um, ${Dimension.format(resolutionNm)} nm, BiSS-C, housing'
      + (mountingPlate ? " with mounting plate" : "") : 'Reading head of sealed scale ${series.series}, ML ${Dimension.format(measuringLength)} mm',
      "aluminium 6061",
      false,
      v
    );
    addFacet(new EngineeringMetadata(new ComponentValues().setToken("ratingsBasis", "Catalog")
      .setToken("errorsBasis", "Catalog").setToken("source", SealedScaleData.SOURCE)));
    addFacet(new PrecisionErrors([
      new PrecisionError("scaleAccuracy", accuracy, "um", Catalog),
      new PrecisionError("thermalExpansion", thermalExpansion() * 1e6, "um/m/K", Catalog)
    ]));
    if (variant == HOUSING) {
      var h = series.housingHeight, d = series.housingDepth, len = overallLength();
      declareMass(
        housingMass(),
        new Vector(0, h / 2, len / 2)
      );
      addConnector("mount", Face, AssemblyFrames.alongY(0, h, len / 2, 0, 1, 0));
      addConnector("axis", Axis, Solids.axial(0, 0, startOffset()));
      if (series.holeDiameter != null) {
        addConnector("end1", Mount, AssemblyFrames.alongY(0, h, endHole(), 0, 1, 0));
        addConnector("end2", Mount, AssemblyFrames.alongY(0, h, len - endHole(), 0, 1, 0));
      }
      addPort({name: "air", kind: Pneumatic, role: Consumer, iface: Unspecified, required: false});
    } else {
      var hh = series.headHeight;
      declareMass(series.headMass, new Vector(0, -hh / 2, 0));
      addConnector("mount", Face, AssemblyFrames.alongY(0, -hh, 0, 0, -1, 0));
      addConnector("axis", Axis, Solids.axial(0, 0, 0));
      addConnector("hole1", Mount, AssemblyFrames.alongY(0, -hh, -series.headHolePitch / 2, 0, -1, 0));
      addConnector("hole2", Mount, AssemblyFrames.alongY(0, -hh, series.headHolePitch / 2, 0, -1, 0));
      addPort({name: "biss", kind: Signal, role: Supply, iface: Unspecified, required: true});
    }
  }

  /**
   * The housing and head for `travel` mm of carriage travel: the smallest standard measuring length that covers it,
   * with the mounting plate where the series requires it or `plate` asks for it.
   */
  public static function pair(
    series:String,
    travel:Float,
    accuracy:String = "3",
    resolution:String = "5",
    plate:Bool = false
  ):{housing:SealedScalePart, head:SealedScalePart} {
    var ml = SealedScaleData.standardLength(series, travel), from = SealedScaleData.series(series).plateRequiredFrom;
    var values = new ComponentValues().setToken("series", series).setNumber("measuringLength", ml)
      .setToken("accuracy", accuracy).setToken("resolution", resolution)
      .setBoolean("mountingPlate", plate || (from != null && ml >= from));
    return {
      housing: new SealedScalePart(values.copy().setToken("variant", HOUSING)),
      head: new SealedScalePart(values.copy().setToken("variant", HEAD))
    };
  }

  // Dimensions ---------------------------------------------------------------------------------------------

  /** Housing length: ML plus the series overhang. */
  public function overallLength():Float {
    var o = mountingPlate ? series.overhangWithPlate : series.overhang;
    if (o == null) throw "no overhang";
    return measuringLength + o;
  }

  /** Where the measuring length starts, from the housing's start end. */
  public function startOffset():Float {
    var o = mountingPlate ? series.startOffsetWithPlate : series.startOffset;
    if (o == null) throw "no start offset";
    return o;
  }

  /** Housing-end fixing hole position from its end (JFT-40: 11.5 on the plate-less housing). */
  function endHole():Float return 11.5;

  public function housingMass():Float return series.housingBaseMass + series.massPerMetre * measuringLength / 1000;
  /** Whole scale (housing and head), as the datasheet's `base + per-metre` weight. */
  public function totalMass():Float return housingMass() + series.headMass;
  public function thermalExpansion():Float {
    var t = mountingPlate ? series.thermalExpansionWithPlate : series.thermalExpansion;
    if (t == null) throw "no thermal expansion";
    return t;
  }
  public function mounting():SealedScaleMounting return series.mounting;
  public function gap():Float return series.mounting.gap;
  /** Counts per millimetre of travel at this resolution. */
  public function countsPerMillimetre():Float return 1e6 / resolutionNm;
  public function interfaceName():String return "BiSS-C";

  public function encoder(id:String, joint:String):AssemblyEncoder return {
    id: id,
    joint: joint,
    kind: "absolute",
    counts: countsPerMillimetre()
  };

  // Geometry -----------------------------------------------------------------------------------------------

  static function box(w:Float, y0:Float, y1:Float, z0:Float, z1:Float):Part return Part.prism(
    [new Vector(-w / 2, y0), new Vector(w / 2, y0), new Vector(w / 2, y1), new Vector(-w / 2, y1)],
    z0,
    z1
  );

  /**
   * The housing is clamped to its seat by two M8 through the end holes, heads in the counterbores (face frame: origin on the mounting face, +Z into the
   * housing, x along the travel); the reading head is tapped M5 for two screws from its carrier (origin on the face away from the housing, +Z into the head).
   * The head's thread depth is ASSUMED (`SealedScaleData`); the maker's rating is not given, so the thread is `rated` (one diameter of engagement).
   */
  override public function fasteningFaces():Array<machinekit.component.BoltedFace> {
    if (variant == HOUSING) {
      var hole = series.holeDiameter, bore = series.holeCounterbore, boreDepth = series.holeCounterboreDepth;
      if (hole == null || bore == null || boreDepth == null) return [];
      return [machinekit.component.BoltedFace.clamped(
        "mount",
        machinekit.component.BoltedFace.frameOf({x: 0, y: series.housingHeight, z: 0}, {x: 0, y: -1, z: 0}, {x: 0, y: 0, z: 1}),
        [{x: endHole(), y: 0.0}, {x: overallLength() - endHole(), y: 0.0}],
        hole,
        series.housingHeight - boreDepth,
        "M8",
        {diameter: bore, depth: boreDepth}
      )];
    }
    var depth = series.headThreadDepth;
    if (depth == null) return [];
    return [machinekit.component.BoltedFace.tapped(
      "mount",
      machinekit.component.BoltedFace.frameOf({x: 0, y: -series.headHeight, z: 0}, {x: 0, y: 1, z: 0}, {x: 0, y: 0, z: 1}),
      [{x: -series.headHolePitch / 2, y: 0.0}, {x: series.headHolePitch / 2, y: 0.0}],
      series.headThread,
      depth,
      series.headHeight,
      false,
      machinekit.component.BoltedFace.ThreadMaterial.Aluminium,
      true
    )];
  }

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (variant == HEAD) {
      // The nominal head is a block with its two M5 tapped holes (tap drill, `headThreadDepth` deep, ASSUMED) in the carriage-side face.
      var block = Solids.named(box(series.headDepth, -series.headHeight, 0, -series.headLength / 2, series.headLength / 2), "head");
      var depth = series.headThreadDepth;
      if (detail == Envelope || depth == null) return block;
      var drill = machinekit.standard.SocketHeadCapScrew.catalog().get(series.headThread).tapDrill;
      return Solids.cut(block, [for (z in [-series.headHolePitch / 2, series.headHolePitch / 2])
        Solids.named(Part.cylinderAlongY(drill / 2, -series.headHeight - 0.1, -series.headHeight + depth, 0, z), "headThread")]);
    }
    var h = series.housingHeight, d = series.housingDepth, len = overallLength();
    var body = Solids.named(box(d, 0, h, 0, len), "housing");
    var hole = series.holeDiameter, bore = series.holeCounterbore, boreDepth = series.holeCounterboreDepth;
    if (detail == Envelope || hole == null || bore == null || boreDepth == null) return body;
    var tools:Array<Part> = [];
    // The screw enters from the face toward the head (y = 0) and its head sinks into the counterbore; the mounting face (y = h) is clear of it.
    for (z in [endHole(), len - endHole()]) {
      tools.push(Solids.named(Part.cylinderAlongY(hole / 2, -0.1, h + 0.1, 0, z), "endHole"));
      tools.push(Solids.named(Part.cylinderAlongY(bore / 2, -0.1, boreDepth, 0, z), "endCounterbore"));
    }
    return Solids.cut(body, tools);
  }

  override public function massGeometry():Null<Part> return null;

  // Recipe -------------------------------------------------------------------------------------------------

  static var recipe:Null<ComponentType>;

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.motion.sealed-scale-part", parameters(), v -> new SealedScalePart(v), true)
      .procured(_ -> Buy(SealedScaleData.MAKER, null, null)).requiring(v -> {
        var req = machinekit.component.Procurement.ProcurementCodec.requirement;
        var choice = machinekit.component.Procurement.ProcurementCodec.choice;
        return [choice("series", v.token("series")), req("measuringLength", "mm", null, null, v.number("measuringLength")),
          req("accuracy", "um", null, Std.parseFloat(v.token("accuracy"))),
          req("resolution", "nm", null, Std.parseFloat(v.token("resolution"))), choice("interface", v.token("interface")),
          choice("mountingPlate", v.boolean("mountingPlate") ? "true" : "false")];
      });
    return recipe;
  }

  override public function componentType():Null<ComponentType> return recipeType();
}
