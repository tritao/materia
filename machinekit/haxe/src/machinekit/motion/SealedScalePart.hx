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
 * CAD frame (both parts share it): travel along +Z, Y from the free face (0) to the mounting face (the thickness) and X across the travel with 0 on the
 * housing's fixing-hole axis. The JFT-40's fixing holes and the head's mounting slots run through the thickness (STEP): the housing is bolted by its side
 * to a seat wall (a bar) and the head, level with it, stands beside it on the reading face's side (+X; `headSide` "-x" mirrors the pair, which is the same
 * hardware on the other side of the seat). The head is carried from the free side (-Y), its screws entering from the seat side beyond the bar's edge.
 * Housing: z=0 at its start end; connector `mount` (Face, +Y out of the mounting face), `axis` on the measuring line at the measuring start, `end1` and
 * `end2` the fixing holes, `air` the M5 purge port on `airPort`'s end. Head: z=0 at the reading finger's centre (the reading datum, ASSUMED: the maker gives
 * none); `mount` the carrier face (-Y out), `hole1` and `hole2` the slots, `axis` on the measuring line, `cable` the cable's end (leaves along -Z).
 * `airPort` ("start", "end", "none") draws the housing's purge fitting and its tube run, `cableBend` ("+x", "-x", "+y", "-y", "none", in the part frame)
 * the head's cable after its straight stub: the minimum bend radius (`SealedScaleData.CABLE_BEND_RADIUS`) and a straight run beyond it. Both are
 * installation choices and so do not change the part number.
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
  /** +1 when the head stands at +X of the housing, -1 mirrored. */
  public final side:Float;
  public final airPort:String;
  public final cableBend:String;

  public static function parameters():Array<ComponentParameter> return [
    P.choice("variant", [HOUSING, HEAD], HOUSING),
    P.choice("series", ["JFT-40", "JFT-10"], "JFT-40"),
    P.length("measuringLength", 120),
    P.flag("mountingPlate", false),
    P.choice("accuracy", ["3", "5"], "3"),
    P.choice("resolution", ["2.5", "5", "10"], "5"),
    P.choice("interface", ["BiSS-C"], "BiSS-C"),
    P.choice("headSide", ["+x", "-x"], "+x"),
    P.choice("airPort", ["none", "start", "end"], "none"),
    P.choice("cableBend", ["none", "+x", "-x", "+y", "-y"], "none")
  ];

  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    variant = v.token("variant");
    series = SealedScaleData.series(v.token("series"));
    measuringLength = v.number("measuringLength");
    mountingPlate = v.boolean("mountingPlate");
    accuracy = Std.parseFloat(v.token("accuracy"));
    resolutionNm = Std.parseFloat(v.token("resolution"));
    side = v.token("headSide") == "-x" ? -1.0 : 1.0;
    airPort = v.token("airPort");
    cableBend = v.token("cableBend");
    if (airPort != "none" && series.airThread == null) throw '${series.series} has no air port';
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
    var t = series.housingThickness;
    if (variant == HOUSING) {
      var len = overallLength();
      declareMass(
        housingMass(),
        new Vector(side * (readingFace() - series.housingWidth / 2), t / 2, len / 2)
      );
      addConnector("mount", Face, AssemblyFrames.alongY(side * (readingFace() - series.housingWidth / 2), t, len / 2, 0, 1, 0));
      addConnector("axis", Axis, Solids.axial(side * (readingFace() - series.readingDepth), t / 2, startOffset()));
      if (series.holeDiameter != null) {
        addConnector("end1", Mount, AssemblyFrames.alongY(0, t, endHole(), 0, 1, 0));
        addConnector("end2", Mount, AssemblyFrames.alongY(0, t, len - endHole(), 0, 1, 0));
      }
      if (airPort != "none") {
        var out = airPort == "start" ? -1.0 : 1.0;
        addConnector("air", Mount, AssemblyFrames.alongY(airX(), t / 2, airPort == "start" ? 0 : len, 0, 0, out));
      }
      addPort({name: "air", kind: Pneumatic, role: Consumer, iface: Unspecified, required: false});
    } else {
      declareMass(series.headMass, new Vector(side * headCentreX(), t / 2, -series.headFingerOffset));
      addConnector("mount", Face, AssemblyFrames.alongY(side * headCentreX(), 0, 0, 0, -1, 0));
      addConnector("axis", Axis, Solids.axial(side * (readingFace() - series.readingDepth), t / 2, 0));
      var hx = side * (headCentreX() - series.headWidth / 2 + series.headHoleHeight);
      if (series.headHoleDiameter == null) hx = side * headCentreX();
      addConnector("hole1", Mount, AssemblyFrames.alongY(hx, 0, holeZ(-1), 0, -1, 0));
      addConnector("hole2", Mount, AssemblyFrames.alongY(hx, 0, holeZ(1), 0, -1, 0));
      if (series.cableDiameter > 0) addConnector("cable", Mount, AssemblyFrames.alongY(side * headCentreX(), t / 2, cableEnd(), 0, 0, -1));
      addPort({name: "biss", kind: Signal, role: Supply, iface: Unspecified, required: true});
    }
  }

  /**
   * The housing and head for `travel` mm of carriage travel: the smallest standard measuring length that covers it,
   * with the mounting plate where the series requires it or `plate` asks for it. `options` may set `headSide`, `airPort` and `cableBend`.
   */
  public static function pair(
    series:String,
    travel:Float,
    accuracy:String = "3",
    resolution:String = "5",
    plate:Bool = false,
    ?options:ComponentValues
  ):{housing:SealedScalePart, head:SealedScalePart} {
    var ml = SealedScaleData.standardLength(series, travel), from = SealedScaleData.series(series).plateRequiredFrom;
    var values = new ComponentValues().setToken("series", series).setNumber("measuringLength", ml)
      .setToken("accuracy", accuracy).setToken("resolution", resolution)
      .setBoolean("mountingPlate", plate || (from != null && ml >= from));
    // Installation choices: which side the head stands (`headSide`), the housing's air port and the head's cable bend; the head takes only the first and the last.
    if (options != null) for (name in ["headSide", "airPort", "cableBend"]) if (options.get(name) != null) values.setToken(name, options.token(name));
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

  /** The housing's body and the head's (without strain relief and cable) as boxes of the part frame, mirrored for `headSide`: {x0, x1, y0, y1, z0, z1}. */
  public function bodyBox():Array<Float> {
    var t = series.housingThickness;
    if (variant == HOUSING) return [Math.min(side * bottomFace(), side * readingFace()), Math.max(side * bottomFace(), side * readingFace()), 0, t, 0, overallLength()];
    var x0 = side * gapFace(), x1 = side * (gapFace() + series.headWidth);
    return [Math.min(x0, x1), Math.max(x0, x1), 0, t, bodyStart(), bodyStart() + series.headLength];
  }

  /** The strain relief's box (head only): the same section as `bodyBox` of the head narrowed to the boot, from the body's cable end. */
  public function bootBox():Array<Float> {
    var t = series.housingThickness, half = series.bootSize / 2, xc = side * headCentreX();
    return [xc - half, xc + half, t / 2 - half, t / 2 + half, bootEnd(), bodyStart()];
  }

  /** Where the cable leaves the boot straight, in the part frame: {x, y, z} of the stub's end, pointing along -Z. */
  public function cableStubEnd():Array<Float> return [side * headCentreX(), series.housingThickness / 2, cableEnd()];

  /** The head's slot axis across the travel (mirrored, as drawn). */
  public function headHoleX():Float return side * (gapFace() + series.headHoleHeight);
  public function headHoleAlong(k:Int):Float return holeZ(k);

  /** Across the travel, in the unmirrored frame (X > 0 toward the head): the housing's reading face, and the lowest face. */
  function readingFace():Float return series.housingWidth - (series.holeOffset ?? series.housingWidth / 2);
  function bottomFace():Float return -(series.holeOffset ?? series.housingWidth / 2);
  /** The head body's gap face and its centre line across the travel, unmirrored. */
  function gapFace():Float return readingFace() + gap();
  function headCentreX():Float return gapFace() + series.headWidth / 2;
  /** The head body's centre along the travel from the reading finger's centre (the head frame's origin). */
  function bodyCentre():Float return -series.headFingerOffset;
  function bodyStart():Float return bodyCentre() - series.headLength / 2;
  /** The head's mounting slot `k` (-1 or +1), along the travel from the finger's centre: the pair centre sits `headHoleOffset` away from the cable. */
  function holeZ(k:Int):Float return series.headHoleOffset + k * series.headHolePitch / 2;
  /** The strain relief's end, and the cable stub's end (the straight run before any bend). */
  function bootEnd():Float return bodyStart() - series.bootLength;
  function cableEnd():Float return bootEnd() - series.cableStraight;
  /** The purge port's position across the travel (unmirrored): below the reading face. */
  function airX():Float return side * (readingFace() - series.airFromReadingFace);

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

  /** Box from x0..x1 (either order), y0..y1, z0..z1. */
  static function box(x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):Part {
    var lo = Math.min(x0, x1), hi = Math.max(x0, x1);
    return Part.prism([new Vector(lo, y0), new Vector(hi, y0), new Vector(hi, y1), new Vector(lo, y1)], z0, z1);
  }

  /** A slot `width` wide whose two ends sit `length` apart along Z, through y0..y1 at x; a round hole when `length` is 0. */
  static function slot(width:Float, length:Float, y0:Float, y1:Float, x:Float, z:Float, tag:String):Part {
    var round = Part.cylinderAlongY(width / 2, y0, y1, x, z - length / 2);
    if (!(length > 0)) return Solids.named(round, tag);
    return Solids.named(Solids.union([round, Part.cylinderAlongY(width / 2, y0, y1, x, z + length / 2), box(x - width / 2, x + width / 2, y0, y1, z - length / 2, z + length / 2)]), tag);
  }

  /**
   * The housing is clamped to its seat by two M8 through the end holes, which run through the thickness; the head of each sits in the counterbore from the
   * free face (face frame: origin on the mounting face on the hole axis, +Z into the housing, x along the travel); the seat-side counterbore is a relief.
   * The reading head's two M4 pass its slots from the seat side (counterbore Ø7.6 there) into the carrier on the free side (face frame: origin on the carrier
   * face on the slot axis, +Z into the head); the carrier is tapped to suit.
   */
  override public function fasteningFaces():Array<machinekit.component.BoltedFace> {
    var t = series.housingThickness;
    if (variant == HOUSING) {
      var hole = series.holeDiameter, bore = series.holeCounterbore, boreDepth = series.holeCounterboreDepth;
      if (hole == null || bore == null || boreDepth == null) return [];
      return [machinekit.component.BoltedFace.clamped(
        "mount",
        machinekit.component.BoltedFace.frameOf({x: 0, y: t, z: 0}, {x: 0, y: -1, z: 0}, {x: 0, y: 0, z: 1}),
        [{x: endHole(), y: 0.0}, {x: overallLength() - endHole(), y: 0.0}],
        hole,
        t - boreDepth,
        "M8",
        {diameter: bore, depth: boreDepth}
      ).lowHeaded()];
    }
    var d = series.headHoleDiameter, cb = series.headCounterbore, cbDepth = series.headCounterboreDepth;
    if (d == null || cb == null || cbDepth == null) return [];
    return [machinekit.component.BoltedFace.clamped(
      "mount",
      machinekit.component.BoltedFace.frameOf({x: side * (gapFace() + series.headHoleHeight), y: 0, z: 0}, {x: 0, y: 1, z: 0}, {x: 0, y: 0, z: 1}),
      [{x: holeZ(-1), y: 0.0}, {x: holeZ(1), y: 0.0}],
      d,
      t - cbDepth,
      series.headThread,
      {diameter: cb, depth: cbDepth}
    )];
  }

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var t = series.housingThickness, s = side;
    if (variant == HEAD) return headGeometry(detail);
    var len = overallLength();
    var body = Solids.named(box(s * bottomFace(), s * readingFace(), 0, t, 0, len), "housing");
    var hole = series.holeDiameter, bore = series.holeCounterbore, boreDepth = series.holeCounterboreDepth;
    var tools:Array<Part> = [];
    if (detail != Envelope && hole != null && bore != null && boreDepth != null) {
      // The holes run through the thickness; each has its counterbore from both faces.
      for (k => z in [endHole(), len - endHole()]) {
        tools.push(Solids.named(Part.cylinderAlongY(hole / 2, -0.1, t + 0.1, 0, z), "endHole" + (k + 1)));
        tools.push(Solids.named(Part.cylinderAlongY(bore / 2, -0.1, boreDepth, 0, z), "endCounterbore" + (2 * k + 1)));
        tools.push(Solids.named(Part.cylinderAlongY(bore / 2, t - boreDepth, t + 0.1, 0, z), "endCounterbore" + (2 * k + 2)));
      }
    }
    var cut = Solids.cut(body, tools);
    if (airPort == "none") return cut;
    // The purge fitting stands out of the port on the end face, then the tube runs straight on.
    var out = airPort == "start" ? -1.0 : 1.0, z0 = airPort == "start" ? 0.0 : len;
    var fit = SealedScaleData.AIR_FITTING_LENGTH, tube = SealedScaleData.AIR_TUBE_STRAIGHT;
    var at = new Vector(airX(), t / 2, z0), dir = new Vector(0, 0, out);
    return Solids.union([
      cut,
      Solids.named(Part.cylinderAlong(SealedScaleData.AIR_FITTING_DIAMETER / 2, at, dir, fit), "airFitting"),
      Solids.named(Part.cylinderAlong(SealedScaleData.AIR_TUBE_DIAMETER / 2, new Vector(airX(), t / 2, z0 + out * (fit - 0.1)), dir, tube + 0.1), "airTube")
    ]);
  }

  /** The head: body with its two slots, the strain relief and the cable stub (the STEP's), and the cable's bend when `cableBend` is set. */
  function headGeometry(detail:ComponentDetail):Part {
    var t = series.housingThickness, s = side, x0 = gapFace(), x1 = x0 + series.headWidth, xc = headCentreX();
    var z0 = bodyStart(), z1 = z0 + series.headLength;
    var body = Solids.named(box(s * x0, s * x1, 0, t, z0, z1), "head");
    var d = series.headHoleDiameter, cb = series.headCounterbore, cbDepth = series.headCounterboreDepth;
    if (detail != Envelope && d != null && cb != null && cbDepth != null) {
      var tools:Array<Part> = [];
      for (k in 0...2) {
        var z = holeZ(k * 2 - 1), x = s * (x0 + series.headHoleHeight);
        tools.push(slot(d, series.headHoleSlot, -0.1, t + 0.1, x, z, "headSlot" + (k + 1)));
        tools.push(slot(cb, series.headHoleSlot, -0.1, cbDepth, x, z, "headCounterbore" + (2 * k + 1)));
        tools.push(slot(cb, series.headHoleSlot, t - cbDepth, t + 0.1, x, z, "headCounterbore" + (2 * k + 2)));
      }
      body = Solids.cut(body, tools);
    }
    if (series.cableDiameter <= 0) return body;
    var half = series.bootSize / 2, pieces = [body];
    pieces.push(Solids.named(box(s * (xc - half), s * (xc + half), t / 2 - half, t / 2 + half, bootEnd(), z0 + 0.1), "boot"));
    var rc = series.cableDiameter / 2, from = new Vector(s * xc, t / 2, bootEnd() + 0.1);
    pieces.push(Solids.named(Part.cylinderAlong(rc, from, new Vector(0, 0, -1), series.cableStraight + 0.1), "cable"));
    if (cableBend != "none" && detail != Envelope) pieces.push(Solids.named(cableBendPart(new Vector(s * xc, t / 2, cableEnd()), rc), "cableBend"));
    return Solids.union(pieces);
  }

  /** The cable after its stub: a quarter bend of the minimum radius toward `cableBend`, then a straight run as long as the stub. */
  function cableBendPart(end:Vector, rc:Float):Part {
    var r = SealedScaleData.CABLE_BEND_RADIUS;
    var b = switch cableBend {
      case "+x": new Vector(1, 0, 0);
      case "-x": new Vector(-1, 0, 0);
      case "+y": new Vector(0, 1, 0);
      default: new Vector(0, -1, 0);
    };
    var d = new Vector(0, 0, -1), n = 24, ring = [for (i in 0...n) {r: r + rc * Math.cos(2 * Math.PI * i / n), z: rc * Math.sin(2 * Math.PI * i / n)}];
    var torus = Part.revolve(ring, Math.PI / 2);
    var centre = end.add(b.scale(r)), xDir = b.scale(-1), normal = xDir.cross(d);
    var bend:Part;
    try bend = torus.placed(new cadkit.modeling.Location(new cadkit.modeling.Plane(centre, xDir, normal))) catch (e:Dynamic) {
      torus.close();
      throw e;
    }
    torus.close();
    var tail = Part.cylinderAlong(rc, centre.add(d.scale(r)).subtract(b.scale(0.1)), b, series.cableStraight + 0.1);
    return Solids.union([bend, tail]);
  }

  override public function massGeometry():Null<Part> return null;

  // Recipe -------------------------------------------------------------------------------------------------

  static final recipe = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.motion.sealed-scale-part", parameters(), v -> new SealedScalePart(v), true)
      .procured(_ -> Buy(SealedScaleData.MAKER, null, null)).requiring(v -> {
        var req = machinekit.component.Procurement.ProcurementCodec.requirement;
        var choice = machinekit.component.Procurement.ProcurementCodec.choice;
        return [choice("series", v.token("series")), req("measuringLength", "mm", null, null, v.number("measuringLength")),
          req("accuracy", "um", null, Std.parseFloat(v.token("accuracy"))),
          req("resolution", "nm", null, Std.parseFloat(v.token("resolution"))), choice("interface", v.token("interface")),
          choice("mountingPlate", v.boolean("mountingPlate") ? "true" : "false")];
      }));
  }

  override public function componentType():Null<ComponentType> return recipeType();
}
