package machinekit.hobbing;

import gearkit.GearSpec;
import gearkit.Iso1328;

/** A linear axis as the budget needs it: its sealed scale and the distance from the scale's line to the hob centre (Abbe offset). */
class AccuracyAxis {
  public var id:String = "";
  public var scale:String = "";
  /** Scale accuracy, +- micrometres over its measuring length (catalogue). */
  public var accuracyUm:Float = 0;
  public var measuringLengthMm:Float = 0;
  /** Distance across the travel from the scale's line to the hob centre, mm (model, `HobberScales` poses). */
  public var abbeMm:Float = 0;
  /** Distance along the travel from the reading head's centre to the hob centre, mm. */
  public var steelLoopMm:Float = 0;
  /** Distance along the travel from the scale housing's start to the work axis, mm. */
  public var castLoopMm:Float = 0;
  public function new() {}
}

/** A runout or form bound read from a part's `PrecisionErrors` facet. */
class AccuracyRunout {
  public var part:String = "";
  public var quantity:String = "";
  public var micrometres:Float = 0;
  public var basis:String = "";
  public function new() {}
}

/**
 * Everything the budget reads from the machine and the process, plus the assumptions that the model does not hold. Each assumed
 * default carries its basis in `HobberAccuracyBudget.build`'s rows; the adapter (`HobberAccuracyChecks`) fills the model values.
 */
class HobberAccuracyInputs {
  public var spec:GearSpec;
  /** Pressure angle, radians. */
  public var pressureAngle:Float = 20 * Math.PI / 180;
  /** Hob lead angle at its pitch cylinder, radians (the A swivel sets the hob axis to it). */
  public var hobLeadAngle:Float = 0.0096;
  public var hobStarts:Int = 1;
  public var hobRpm:Float = 2984;
  public var axes:Array<AccuracyAxis> = [];

  /** Guide pitch/yaw/roll error over the travel, arcseconds (assumed: no rail class in the catalogue). */
  public var guideAngleArcsec:Float = 5.0;
  /** Share of the Abbe error left after the controller's tilt table is loaded (assumed). */
  public var compensationResidual:Float = 0.2;
  /** Rail running parallelism / straightness over the full stroke, micrometres (assumed, precision-class rail). */
  public var straightnessUm:Float = 3.0;
  public var strokeMm:Float = 200;

  /** Temperature difference between the scale/work loop and the hob side during a cut cycle, kelvin (assumed). */
  public var deltaKelvin:Float = 1.0;
  public var alphaSteel:Float = 11.7e-6;
  public var alphaCast:Float = 12.0e-6;

  /** HB10 radial and tangential reference peaks and the table's reaction torque bound (quoted, not re-derived). */
  public var radialForceN:Float = 16.989670571379065;
  public var tangentialForceN:Float = 46.7;
  public var tableTorqueNm:Float = 1.49;

  /** Radial stiffness at the hob of the cartridge nose (assumed), outboard support (model), table (model), N/um. */
  public var noseStiffness:Float = 50.0;
  public var outboardStiffness:Float = 100.0;
  public var tableRadialStiffness:Float = 100.0;
  /** Column sole tilt about the carriage's travel axis, N m/rad (`frame_stiffness.py`, plate share), and the hob's height above the sole, mm. */
  public var soleTilt:Float = 263923900.0;
  public var hobHeightMm:Float = 300.0;
  /** Compliance of everything not listed (column, casting, rail blocks) as a multiple of the listed compliance (assumed). */
  public var unlistedCompliance:Float = 1.0;

  /** The purchased table's values by name (`RotaryAxisSpec`) and their bases. */
  public var table:Map<String, Float> = new Map();
  public var tableBasis:Map<String, String> = new Map();
  public var tablePivotMm:Float = 100;

  /** A swivel: output encoder accuracy and repeatability (arcsec, catalogue), torsional stiffness (assumed), hob's lever about A (mm). */
  public var aAbsoluteArcsec:Float = 15;
  public var aRepeatArcsec:Float = 7;
  public var aTorsionStiffness:Float = 2.0e4;
  public var aLeverMm:Float = 10;
  /** Half the length of the hob's engagement along its axis, mm (assumed): the cutting force acts this far off the A axis, which passes through the hob centre. */
  public var aEngagementMm:Float = 10;

  /** B master encoder: accuracy (arcsec), counts per revolution. */
  public var bAccuracyArcsec:Float = 40;
  public var bCounts:Float = 524288;
  /** Electronic gearbox: unknown delay (us), cycle period (us), B speed ripple fraction. */
  public var syncDelayUs:Float = 1.0;
  public var cyclePeriodUs:Float = 250.0;
  public var bSpeedRipple:Float = 0.005;

  /** Mandrel (workholding): eccentricity amplitude, clamped roundness, band expansion variation, micrometres. */
  public var mandrelEccentricityUm:Float = 1.5;
  public var mandrelRoundnessUm:Float = 1.5;
  public var mandrelBandVariationUm:Float = 3.0;

  /** Runout and profile bounds of the hob chain (hob, arbor, outboard support, spindle bearings). */
  public var runouts:Array<AccuracyRunout> = [];
  public function new(spec:GearSpec) this.spec = spec;
}

/** One error source: its value and basis, how it maps to each gear parameter, and what would improve it. */
class AccuracyContributor {
  public final id:String;
  public final group:String;
  public final value:Float;
  public final unit:String;
  /** Catalog, Model (value read from the machine), Stated (quoted in the docs), Derived or Assumed. */
  public final basis:String;
  public final note:String;
  public final sensitivity:String;
  /** Bound on fp, Fp, Fr, Falpha, Fbeta from this source alone, micrometres. */
  public final impact:Array<Float>;
  /** Tooth thickness (size) effect, micrometres; not an ISO 1328 parameter (set by the first-article trim). */
  public final size:Float;
  public final improve:String;
  /** Rows that share one physical cause (the four hob-chain axial runouts) are ranked together by root sum of squares. */
  public var family:String;
  public function new(id:String, group:String, value:Float, unit:String, basis:String, note:String, sensitivity:String, impact:Array<Float>, size:Float, improve:String) {
    this.id = id;
    this.group = group;
    this.value = value;
    this.unit = unit;
    this.basis = basis;
    this.note = note;
    this.sensitivity = sensitivity;
    this.impact = impact;
    this.size = size;
    this.improve = improve;
    this.family = id;
  }
}

class AccuracyResult {
  public final contributors:Array<AccuracyContributor>;
  public final compensated:Bool;
  /** Per parameter (`HobberAccuracyBudget.PARAMETERS`): root-sum-square and worst-case (sum) totals, micrometres. */
  public final rss:Array<Float>;
  public final worst:Array<Float>;
  public final sizeRss:Float;
  public final sizeWorst:Float;
  /** ISO class limits at the target class and one finer, micrometres. */
  public final limitTarget:Array<Float>;
  public final limitFiner:Array<Float>;
  /** Integer class (null: worse than 11) and fractional equivalent class from the unrounded formula, for the RSS and worst totals. */
  public final classRss:Array<Null<Int>>;
  public final classWorst:Array<Null<Int>>;
  public final equivalentRss:Array<Float>;
  public final equivalentWorst:Array<Float>;
  public function new(contributors:Array<AccuracyContributor>, compensated:Bool, rss:Array<Float>, worst:Array<Float>, sizeRss:Float, sizeWorst:Float,
      limitTarget:Array<Float>, limitFiner:Array<Float>, classRss:Array<Null<Int>>, classWorst:Array<Null<Int>>, equivalentRss:Array<Float>, equivalentWorst:Array<Float>) {
    this.contributors = contributors;
    this.compensated = compensated;
    this.rss = rss;
    this.worst = worst;
    this.sizeRss = sizeRss;
    this.sizeWorst = sizeWorst;
    this.limitTarget = limitTarget;
    this.limitFiner = limitFiner;
    this.classRss = classRss;
    this.classWorst = classWorst;
    this.equivalentRss = equivalentRss;
    this.equivalentWorst = equivalentWorst;
  }
}

/**
 * Analytic accuracy budget of the flexspline hobber for the reference gear (FRAME_PLAN, MACHINE_DESIGN "Accuracy budget").
 *
 * Every error source is a row with its value, basis, sensitivity to the gear parameters and the bound it puts on each. A bound is the
 * largest value of the parameter that source alone can produce (peak to peak for Fp, Fr and the form parameters, as the standard defines
 * them). Totals are the root sum of squares (expected, independent sources) and the plain sum (every source at its bound at once).
 * The class is read against `gearkit.Iso1328`, which implements the tolerance formulas of ISO 1328-1:2013 (class 5 base values from the
 * module and reference diameter, times the square root of 2 per class step). The reference part (m 0.3) is below the standard's module
 * range (m >= 0.5), so those classes are extrapolated, as everywhere in this repository.
 *
 * The budget is untrimmed: static offsets (size, a constant helix error) can be cut by a first-article trim; they are listed as bounds.
 * Nothing here is a simulation of a cut. The hobbing error budget (`stockkit.hobbing.errors`) runs the cut; this one ranks the machine.
 */
class HobberAccuracyBudget {
  public static final PARAMETERS = ["fp", "Fp", "Fr", "Falpha", "Fbeta"];
  /** ISO 1328 class the machine is gated on. PLAN.md HB12: "the reference flexspline grades ISO 5-6 on every parameter"; the gate is class 6. */
  public static inline final TARGET_CLASS = 6;
  static inline final ARC = Math.PI / 648000;

  static function r2(v:Float):Float return Math.round(v * 100) / 100;
  static function r3(v:Float):Float return Math.round(v * 1000) / 1000;

  static function rowOf(list:Array<AccuracyContributor>, id:String, group:String, value:Float, unit:String, basis:String, note:String, sensitivity:String, fp:Float, Fp:Float, Fr:Float,
      Fa:Float, Fb:Float, size:Float, improve:String):Void {
    list.push(new AccuracyContributor(id, group, value, unit, basis, note, sensitivity, [fp, Fp, Fr, Fa, Fb], size, improve));
  }

  static function tv(inputs:HobberAccuracyInputs, name:String):Float {
    var v = inputs.table.get(name);
    if (v == null) throw "table value missing: " + name;
    return v;
  }

  static function axis(inputs:HobberAccuracyInputs, id:String):Null<AccuracyAxis> {
    for (a in inputs.axes) if (a.id == id) return a;
    return null;
  }

  static function runoutRows(inputs:HobberAccuracyInputs, list:Array<AccuracyContributor>, sinA:Float, cosA:Float):Void {
    for (u in inputs.runouts) {
      var partIsHob = u.part == "hob";
      var before = list.length;
      var where = partIsHob ? "hob" : u.part;
      if (u.quantity == "radialRunout") {
        rowOf(list, where + ".radialRunout", "B hob chain", u.micrometres, "um TIR", u.basis, "part facet PrecisionErrors, " + u.part,
          "profile: radial depth change times sin(alpha) = " + r3(sinA) + " per um (one hob tooth per work tooth: the pattern repeats each tooth, so no pitch error)", 0, 0, 0, u.micrometres * sinA, 0, 0,
          "Better arbor and outboard coaxiality (grind the arbor on its own bearings, indicate the hob on the machine and tap it true), or re-grind the hob on its arbor");
      } else if (u.quantity == "axialRunout") {
        rowOf(list, where + ".axialRunout", "B hob chain", u.micrometres, "um TIR", u.basis, "part facet PrecisionErrors, " + u.part,
          "profile: axial flank shift times cos(alpha) = " + r3(cosA) + " per um", 0, 0, 0, u.micrometres * cosA, 0, 0,
          "Square the hob's clamping faces to the arbor (lap the shoulder, ground spacer), indicate the hob face on the machine and shim it true; spindle bearing preload and a lower-runout cartridge grade");
      } else if (u.quantity == "profileError" && partIsHob) {
        rowOf(list, "hob.profileError", "B hob chain", u.micrometres, "um", u.basis, "part facet PrecisionErrors, hob",
          "profile: copied one to one onto the flank", 0, 0, 0, u.micrometres, 0, 0, "Hob accuracy grade AA (DIN 3968) or a ground carbide hob");
      }
      if (list.length > before) {
        list[list.length - 1].family = u.quantity == "radialRunout" ? "hob chain radial runout" : u.quantity == "axialRunout" ? "hob chain axial runout" : "hob profile error";
      }
    }
  }

  /** The rows. `compensated`: the controller's tilt table is loaded, so the Abbe error of the linear axes is left at its residual share. */
  public static function build(inputs:HobberAccuracyInputs, compensated:Bool):Array<AccuracyContributor> {
    var spec = inputs.spec, list:Array<AccuracyContributor> = [];
    var r = spec.pitchRadius(), z = spec.teeth, b = spec.faceWidth, a = inputs.pressureAngle;
    var sinA = Math.sin(a), cosA = Math.cos(a), tan2 = 2 * Math.tan(a), um = 1000.0;
    var theta = inputs.guideAngleArcsec * ARC;
    var omegaB = inputs.hobRpm * 2 * Math.PI / 60;
    var perToothIndex = Math.abs(Math.sin(Math.PI / z));

    // ---- Linear axes -------------------------------------------------------------------------------------------------------------
    var gx = "X radial depth", gy = "Y hob shift", gz = "Z feed";
    var x = axis(inputs, "X"), y = axis(inputs, "Y"), zz = axis(inputs, "Z");
    if (x != null) {
      rowOf(list, "X.scale", gx, x.accuracyUm, "um (+-)", "Catalog", x.scale + ", accuracy over its measuring length " + r2(x.measuringLengthMm) + " mm",
        "size: tooth thickness 2 tan(alpha) = " + r3(tan2) + " per um of radial error; static during the finish pass, so no class parameter", 0, 0, 0, 0, 0, tan2 * x.accuracyUm,
        "Trim the first article (X offset); a +-3 um scale is already the best catalogue grade");
      var abbe = x.abbeMm * theta * um * (compensated ? inputs.compensationResidual : 1.0);
      rowOf(list, compensated ? "X.abbe(compensated)" : "X.abbe(uncompensated)", gx, abbe, "um", "Assumed",
        "Abbe offset " + r2(x.abbeMm) + " mm (model, scale line to hob centre) x guide angle " + inputs.guideAngleArcsec + "\" (assumed, no rail class in the catalogue)"
        + (compensated ? " x " + inputs.compensationResidual + " residual after the tilt table (assumed)" : ""),
        "size: 2 tan(alpha) x offset x angle", 0, 0, 0, 0, 0, tan2 * abbe, "Load the pitch/yaw tilt table (x-compensate-tilt procedure); measure the real guide error");
      var thermal = (inputs.alphaSteel * x.steelLoopMm + inputs.alphaCast * x.castLoopMm) * inputs.deltaKelvin * um;
      rowOf(list, "X.thermal", gx, thermal, "um", "Assumed",
        "frame: steel " + r2(x.steelLoopMm) + " mm (head to hob) x " + inputs.alphaSteel * 1e6 + " um/m/K plus mineral cast " + r2(x.castLoopMm) + " mm (scale start to work axis) x "
        + inputs.alphaCast * 1e6 + " um/m/K, over " + inputs.deltaKelvin + " K (assumed: shop drift plus spindle heat during a cut cycle and between trims)",
        "size: 2 tan(alpha) x growth; " + r3((inputs.alphaSteel * x.steelLoopMm + inputs.alphaCast * x.castLoopMm) * um) + " um per K", 0, 0, 0, 0, 0, tan2 * thermal,
        "Thermal control: stable shop (+-0.5 K), spindle warm-up before the trim, a temperature-compensated X (frame sensor), or a trim every part");
      rowOf(list, "X.screw", gx, 0, "um", "Derived", "not counted: the scale closes the X loop; the screw's growth and the servo's positioning drop out",
        "none while the scale loop is closed", 0, 0, 0, 0, 0, 0, "");
      rowOf(list, "X.roll", gx, inputs.guideAngleArcsec, "arcsec", "Assumed", "roll of the X carriage about its travel tilts the hob axis about X; not measured by the scale or the tilt table, rail class assumed",
        "helix: face width x angle = " + r3(b * ARC * um) + " um per arcsec", 0, 0, 0, 0, b * theta * um, 0, "Rail grade and a two-rail spread; measure roll with a level at commissioning");
    }
    if (y != null) {
      rowOf(list, "Y.scale", gy, y.accuracyUm, "um (+-)", "Catalog", y.scale + ", accuracy over its measuring length " + r2(y.measuringLengthMm) + " mm",
        "none for the class: a hob shift moves along the hob axis, so the tooth phase moves with it and the blank follows; its radial share is the lead angle x tan2a", 0, 0, 0, 0, 0,
        tan2 * Math.sin(inputs.hobLeadAngle) * y.accuracyUm, "");
      var abbeY = y.abbeMm * theta * um * (compensated ? inputs.compensationResidual : 1.0);
      rowOf(list, compensated ? "Y.abbe(compensated)" : "Y.abbe(uncompensated)", gy, abbeY, "um", "Assumed",
        "Abbe offset " + r2(y.abbeMm) + " mm (model) x guide angle " + inputs.guideAngleArcsec + "\" (assumed)" + (compensated ? " x " + inputs.compensationResidual + " residual (assumed)" : ""),
        "size: 2 tan(alpha) x sin(lead angle " + r3(inputs.hobLeadAngle * 180 / Math.PI) + " deg)", 0, 0, 0, 0, 0, tan2 * Math.sin(inputs.hobLeadAngle) * abbeY, "Tilt table");
    }
    if (zz != null) {
      rowOf(list, "Z.scale", gz, zz.accuracyUm, "um (+-)", "Catalog", zz.scale + ", accuracy over its measuring length " + r2(zz.measuringLengthMm) + " mm",
        "none for a spur gear: Z only places the 8 mm band along the face; the flank line runs along Z", 0, 0, 0, 0, 0, 0, "");
      var abbeZ = zz.abbeMm * theta * um * (compensated ? inputs.compensationResidual : 1.0);
      rowOf(list, compensated ? "Z.abbe(compensated)" : "Z.abbe(uncompensated)", gz, abbeZ, "um", "Assumed",
        "Abbe offset " + r2(zz.abbeMm) + " mm (model) x guide angle " + inputs.guideAngleArcsec + "\" (assumed)" + (compensated ? " x " + inputs.compensationResidual + " residual (assumed)" : ""),
        "none for a spur gear (Z position only places the band)", 0, 0, 0, 0, 0, 0, "");
      rowOf(list, "Z.tilt", gz, inputs.guideAngleArcsec, "arcsec", "Assumed", "angular error of the Z saddle about X tilts the hob axis (the tilt table corrects the position, not the orientation); rail class assumed",
        "helix: face width x angle = " + r3(b * ARC * um) + " um per arcsec", 0, 0, 0, 0, b * theta * um, 0, "Rail grade; trim A against it at the first article");
      var line = inputs.straightnessUm * Math.min(1.0, b / inputs.strokeMm);
      rowOf(list, "Z.straightness", gz, inputs.straightnessUm, "um over " + r2(inputs.strokeMm) + " mm", "Assumed",
        "rail running parallelism (precision class, assumed) shared linearly over the " + r2(b) + " mm band; not seen by the Z scale",
        "flank line: radial change x sin(alpha) = " + r3(sinA), 0, 0, 0, 0, line * sinA, 0, "Rail straightness grade; scrape or shim the rail seat");
    }

    // ---- Structural deflection under cutting load --------------------------------------------------------------------------------
    var kHob = inputs.noseStiffness + inputs.outboardStiffness;
    var tilt = Math.pow(inputs.hobHeightMm / 1000, 2) / inputs.soleTilt * 1e6;
    var compliance = (1 / kHob + 1 / inputs.tableRadialStiffness + tilt) * (1 + inputs.unlistedCompliance);
    var deflection = inputs.radialForceN * compliance;
    rowOf(list, "structure.radial", "Structure under load", deflection, "um", "Assumed",
      "radial force " + r2(inputs.radialForceN) + " N (HB10 peak) / [hob side nose " + inputs.noseStiffness + " N/um (assumed) + outboard " + inputs.outboardStiffness + " N/um (model); table "
      + inputs.tableRadialStiffness + " N/um (model); column sole tilt " + r3(inputs.soleTilt / 1e6) + " MN m/rad over " + r2(inputs.hobHeightMm) + " mm (frame_stiffness.py, steel deck with ribs)]"
      + " x (1 + " + inputs.unlistedCompliance + " for the unlisted casting, column and blocks, assumed)",
      "size: 2 tan(alpha) x deflection; flank line: the force changes along the 8 mm band, x sin(alpha) = " + r3(sinA), 0, 0, 0, 0, deflection * sinA, tan2 * deflection,
      "Stiffer hob side (cartridge nose stiffness, two-sided outboard), FE of the column and rails to replace the assumed unlisted share");
    var rollTilt = tv(inputs, "momentStiffness");
    if (rollTilt > 0) {
      var angle = inputs.radialForceN * inputs.tablePivotMm / 1000 / rollTilt;
      rowOf(list, "table.tilt", "Structure under load", angle * 1e6, "urad", inputs.tableBasis.get("momentStiffness"), "radial force x pivot depth " + r2(inputs.tablePivotMm) + " mm / moment stiffness "
        + rollTilt + " N m/rad", "helix: face width x tilt", 0, 0, 0, 0, b * angle * um, 0, "Moment stiffness of the table bearing (vendor)");
    }
    var aAngle = inputs.tangentialForceN * inputs.aLeverMm / 1000 / inputs.aTorsionStiffness;
    rowOf(list, "A.torsion", "A swivel", aAngle * 1e6, "urad", "Assumed", "cutting force " + r2(inputs.tangentialForceN) + " N (load study peak) x lever " + r2(inputs.aLeverMm)
      + " mm about A (hob centre offset from the A axis (model) plus " + r2(inputs.aEngagementMm) + " mm of engagement half-length, assumed) / torsional stiffness " + inputs.aTorsionStiffness + " N m/rad (assumed: strain-wave size class, not in the maker data we hold; brake holding path unknown)",
      "helix: face width x angle", 0, 0, 0, 0, b * aAngle * um, 0, "Torsional stiffness and brake rigidity from ZeroErr; trim A at the first article against the load");
    var servoK = tv(inputs, "servoTorsionalStiffness");
    var dTorque = inputs.tableTorqueNm / servoK;
    rowOf(list, "C.reaction", "C table", dTorque * 1e6, "urad", inputs.tableBasis.get("servoTorsionalStiffness"), "table torque bound " + inputs.tableTorqueNm + " N m (HB10) / servo torsional stiffness "
      + servoK + " N m/rad", "pitch: 2 r x angle", 0, 2 * r * dTorque * um, 0, 0, 0, 0, "Stiffer C loop (feedforward of the load torque from B torque)");

    // ---- Rotary axes ------------------------------------------------------------------------------------------------------------
    var gc = "C table";
    function harmonic(id:String, field:String, amplitude:Float, order:Float, note:String, improve:String):Void {
      rowOf(list, id, gc, amplitude / ARC, "arcsec", inputs.tableBasis.get(field), note,
        "pitch: Fp = 2 r x angle = " + r3(2 * r * um * ARC) + " um per arcsec; fp = Fp x |sin(order x pi / z)| for order " + order, 2 * r * amplitude * Math.abs(Math.sin(order * Math.PI / z)) * um,
        2 * r * amplitude * um, 0, 0, 0, 0, improve);
    }
    harmonic("C.accuracy", "absoluteAccuracy", tv(inputs, "absoluteAccuracy"), 4, "table positioning accuracy (catalogue), assumed order 4", "Calibrate the table (error map in the controller)");
    var rep = tv(inputs, "repeatability");
    rowOf(list, "C.repeatability", gc, rep / ARC, "arcsec", inputs.tableBasis.get("repeatability"), "table repeatability (catalogue); acts tooth to tooth",
      "pitch: fp = Fp = 2 r x angle", 2 * r * rep * um, 2 * r * rep * um, 0, 0, 0, 0, "Direct-drive table with a finer encoder; repeatability is the floor for fp");
    harmonic("C.interpolation", "interpolationError", tv(inputs, "interpolationError"), 4096, "encoder interpolation, assumed 4096th harmonic", "Higher-grade encoder");
    harmonic("C.speedRipple", "speedRipple", tv(inputs, "speedRipple"), 1,
      "once-per-turn speed ripple fraction " + tv(inputs, "speedRipple") + " integrates to that many radians (assumed 0.1 %, loop rejection not credited, bench measurement at 15 rpm required)",
      "Measure the real ripple on the bench; raise the position-loop gain or add torque-ripple feedforward; ask the table vendor for the ripple at 10-20 rpm");
    harmonic("C.cogging", "coggingTorque", tv(inputs, "coggingTorque") / servoK, 48, "cogging torque / servo torsional stiffness, 48th harmonic", "Cogging compensation table");
    harmonic("C.sealDrag", "sealDragRipple", tv(inputs, "sealDragRipple") / servoK, 2, "seal drag ripple / servo torsional stiffness, twice per turn", "Low-drag seal");
    var runout = tv(inputs, "radialRunout") * um;
    rowOf(list, "C.radialRunout", gc, runout, "um TIR", inputs.tableBasis.get("radialRunout"), "table radial runout (total, periodic plus non-repeatable); the teeth are cut about the table axis, the bore datum is eccentric to it",
      "runout and pitch about the bore: Fr = Fp = TIR; fp = TIR x sin(pi/z)", runout * perToothIndex, runout, runout, 0, 0, 0, "Table bearing runout grade; tap the mandrel in on the table");
    var comp = 2 * inputs.radialForceN / inputs.tableRadialStiffness;
    rowOf(list, "C.radialCompliance", gc, comp, "um", "Assumed", "radial force " + r2(inputs.radialForceN) + " N / table radial stiffness " + inputs.tableRadialStiffness + " N/um (model, assumed) x 2 (force 0 to peak and back)",
      "runout: Fr = Fp = value", comp * perToothIndex, comp, comp, 0, 0, 0, "Table radial stiffness (vendor)");

    var ga = "A swivel";
    rowOf(list, "A.setting", ga, inputs.aAbsoluteArcsec, "arcsec (+-)", "Catalog", "eRob output encoder absolute accuracy " + inputs.aAbsoluteArcsec + "\" (repeatability " + inputs.aRepeatArcsec + "\"), model PrecisionErrors; the hob axis is set to the lead angle",
      "helix: Fbeta = face width x setting error = " + r3(b * ARC * um) + " um per arcsec", 0, 0, 0, 0, b * inputs.aAbsoluteArcsec * ARC * um, 0,
      "Trim A at the first article (leaves the " + inputs.aRepeatArcsec + "\" repeatability); a dial-indicator check of the hob axis against the work axis");

    var gb = "B spindle and sync";
    var bAtC = inputs.bAccuracyArcsec * ARC * inputs.hobStarts / z;
    rowOf(list, "B.encoder", gb, inputs.bAccuracyArcsec, "arcsec", "Assumed", "B master encoder accuracy over a revolution, assumed (" + inputs.bCounts + " counts = " + r2(1296000 / inputs.bCounts) + "\" per count; MACHINE_DESIGN example 40\")",
      "pitch: the error reaches C divided by z/starts = " + z / inputs.hobStarts + "; Fp = fp = 2 r x angle x starts/z", 2 * r * bAtC * um, 2 * r * bAtC * um, 0, 0, 0, 0, "Not limiting; no action");
    var tau = omegaB * inputs.syncDelayUs * 1e-6 * inputs.hobStarts / z;
    rowOf(list, "sync.delay", gb, inputs.syncDelayUs, "us", "Assumed", "unknown delay between the B sample and the C command (distributed-clock jitter, assumed 1 us) at " + r2(inputs.hobRpm) + " rpm",
      "pitch: 2 r x omega_B x delay x starts/z = " + r3(2 * r * um * omegaB * 1e-6 * inputs.hobStarts / z) + " um per us (1 ms uncompensated = 47 um, PLAN.md)", 2 * r * tau * um, 2 * r * tau * um, 0, 0, 0, 0,
      "Timestamped B sample with fixed compensated latency");
    var ripple = omegaB * inputs.cyclePeriodUs * 1e-6 * inputs.bSpeedRipple * inputs.hobStarts / z;
    rowOf(list, "sync.loop", gb, inputs.cyclePeriodUs, "us cycle", "Assumed", "cycle " + inputs.cyclePeriodUs + " us (4 kHz, assumed) x B speed ripple " + inputs.bSpeedRipple * 100 + " % (assumed), no feedforward credit",
      "pitch: lag changes with the ripple; 2 r x omega_B x Ts x ripple x starts/z", 2 * r * ripple * um, 2 * r * ripple * um, 0, 0, 0, 0, "Feedforward extrapolation of the master; faster cycle");
    runoutRows(inputs, list, sinA, cosA);

    // ---- Workholding --------------------------------------------------------------------------------------------------------------
    var gw = "Workholding";
    var ecc = 2 * inputs.mandrelEccentricityUm;
    rowOf(list, "mandrel.eccentricity", gw, inputs.mandrelEccentricityUm, "um amplitude", "Model", "clamped mandrel eccentricity (ExpandingMandrel), TIR = 2 x amplitude",
      "runout and pitch about the bore: Fr = Fp = TIR; fp = TIR x sin(pi/z)", ecc * perToothIndex, ecc, ecc, 0, 0, 0, "Tap the mandrel in on the table and hold 1 um; mandrel grind-after-mount");
    rowOf(list, "mandrel.roundness", gw, inputs.mandrelRoundnessUm, "um", "Model", "clamped sleeve roundness; the cup copies it at 0.95 um/um",
      "runout: Fr = 0.95 x roundness", 0, 0, 0.95 * inputs.mandrelRoundnessUm, 0, 0, 0, "Sleeve roundness grade");
    rowOf(list, "mandrel.bandExpansion", gw, inputs.mandrelBandVariationUm, "um", "Model", "expansion variation over the tooth band; flank-line effect 0.34 x (ExpandingMandrel)",
      "flank line: 0.34 x variation", 0, 0, 0, 0, 0.34 * inputs.mandrelBandVariationUm, 0, "Press-length margin; uniform contact pressure");
    return list;
  }

  /** Totals, classes and limits for one list of rows. */
  public static function summarise(inputs:HobberAccuracyInputs, rows:Array<AccuracyContributor>, compensated:Bool):AccuracyResult {
    var n = PARAMETERS.length, rss = [], worst = [], cls:Array<Null<Int>> = [], clw:Array<Null<Int>> = [], eqr = [], eqw = [], lt = [], lf = [];
    var spec = inputs.spec;
    for (p in 0...n) {
      var s2 = 0.0, s1 = 0.0;
      for (c in rows) {
        s2 += c.impact[p] * c.impact[p];
        s1 += c.impact[p];
      }
      rss.push(Math.sqrt(s2));
      worst.push(s1);
      var name = PARAMETERS[p];
      cls.push(Iso1328.grade(spec, name, Math.sqrt(s2)));
      clw.push(Iso1328.grade(spec, name, s1));
      var base = Iso1328.tolerance(spec, name, 5, false);
      eqr.push(equivalent(Math.sqrt(s2), base));
      eqw.push(equivalent(s1, base));
      lt.push(Iso1328.tolerance(spec, name, TARGET_CLASS));
      lf.push(Iso1328.tolerance(spec, name, TARGET_CLASS - 1));
    }
    var sz2 = 0.0, sz1 = 0.0;
    for (c in rows) {
      sz2 += c.size * c.size;
      sz1 += c.size;
    }
    return new AccuracyResult(rows, compensated, rss, worst, Math.sqrt(sz2), sz1, lt, lf, cls, clw, eqr, eqw);
  }

  /** Fractional class: the ISO step is 2^(1/2) per class, class 5 at `base`. */
  public static function equivalent(value:Float, base:Float):Float {
    if (value <= 0) return 0;
    return 5 + 2 * Math.log(value / base) / Math.log(2);
  }

  public static function run(inputs:HobberAccuracyInputs, compensated:Bool = true):AccuracyResult return summarise(inputs, build(inputs, compensated), compensated);

  /** One ranked cause: rows of one family combined by root sum of squares. */
  public static function families(result:AccuracyResult):Array<AccuracyContributor> {
    var names:Array<String> = [], out:Array<AccuracyContributor> = [];
    for (c in result.contributors) {
      var at = names.indexOf(c.family);
      if (at < 0) {
        names.push(c.family);
        out.push(c);
      } else {
        var first = out[at], members = [for (d in result.contributors) if (d.family == c.family) d];
        var impact = [for (p in 0...PARAMETERS.length) {
          var s2 = 0.0;
          for (d in members) s2 += d.impact[p] * d.impact[p];
          Math.sqrt(s2);
        }];
        var sz = 0.0;
        for (d in members) sz += d.size * d.size;
        var merged = new AccuracyContributor(c.family, first.group, 0, "", first.basis, members.length + " rows: " + [for (d in members) d.id].join(", "), first.sensitivity, impact, Math.sqrt(sz), first.improve);
        out[at] = merged;
      }
    }
    return out;
  }

  /** The share of the target class limit one row uses on its worst parameter, and that parameter. */
  public static function share(c:AccuracyContributor, limits:Array<Float>):{fraction:Float, parameter:String} {
    var best = 0.0, name = "";
    for (p in 0...PARAMETERS.length) {
      var f = c.impact[p] / limits[p];
      if (f > best) {
        best = f;
        name = PARAMETERS[p];
      }
    }
    return {fraction: best, parameter: name};
  }

  /** Causes (families) ranked by their share of the target class limit, largest first. */
  public static function ranked(result:AccuracyResult):Array<AccuracyContributor> {
    var rows = families(result);
    rows.sort((a, b) -> {
      var fa = share(a, result.limitTarget).fraction, fb = share(b, result.limitTarget).fraction;
      return fa > fb ? -1 : fa < fb ? 1 : 0;
    });
    return rows;
  }

  /** Gate: the parameters whose predicted (RSS) class is worse than `TARGET_CLASS`; empty means the budget passes. */
  public static function failures(result:AccuracyResult):Array<String> {
    var out = [];
    for (p in 0...PARAMETERS.length) {
      var c = result.classRss[p];
      if (c == null || c > TARGET_CLASS) out.push(PARAMETERS[p] + ": predicted class " + (c == null ? "worse than 11" : Std.string(c)) + " (" + r2(result.rss[p]) + " um against the class " + TARGET_CLASS
        + " limit " + result.limitTarget[p] + " um)");
    }
    return out;
  }

  /** The largest once-per-turn C speed ripple fraction that keeps Fp at the target class, all other rows unchanged (RSS). */
  public static function allowedRipple(inputs:HobberAccuracyInputs, rows:Array<AccuracyContributor>, limit:Float):Float {
    var others = 0.0, own = 0.0;
    for (c in rows) {
      if (c.id == "C.speedRipple") own = c.impact[1];
      else others += c.impact[1] * c.impact[1];
    }
    var room = limit * limit - others;
    if (room <= 0 || own <= 0) return 0;
    var ripple = tv(inputs, "speedRipple");
    return ripple * Math.sqrt(room) / own;
  }

  static function pad(s:String, n:Int):String {
    while (s.length < n) s += " ";
    return s;
  }

  static function f(v:Float):String return v == 0 ? "-" : Std.string(Math.round(v * 100) / 100);

  public static function classText(c:Null<Int>):String return c == null ? ">11" : Std.string(c);

  /** The printed report of one result: a table per group, totals, class, ranking. */
  public static function report(inputs:HobberAccuracyInputs, result:AccuracyResult, rows:Bool = true):Array<String> {
    var lines:Array<String> = [];
    var spec = inputs.spec;
    lines.push('Accuracy budget (tilt table ${result.compensated ? "loaded" : "NOT loaded"}), reference gear m ${spec.module} z ${spec.teeth} alpha ${r2(inputs.pressureAngle * 180 / Math.PI)} deg b ${spec.faceWidth} mm, d ${r2(spec.pitchDiameter())} mm');
    lines.push("Each row: bound on the parameter from that source alone, micrometres (size: tooth thickness, not an ISO parameter). Basis: Catalog (maker data), Model (read from the machine), Stated (quoted in the docs), Derived, Assumed.");
    lines.push(pad("source", 26) + pad("value", 22) + pad("basis", 9) + pad("fp", 8) + pad("Fp", 8) + pad("Fr", 8) + pad("Falpha", 8) + pad("Fbeta", 8) + "size");
    var group = "";
    for (c in rows ? result.contributors : []) {
      if (c.group != group) {
        group = c.group;
        lines.push("-- " + group);
      }
      lines.push(pad(c.id, 26) + pad(f(c.value) + " " + c.unit, 22) + pad(c.basis, 9) + pad(f(c.impact[0]), 8) + pad(f(c.impact[1]), 8) + pad(f(c.impact[2]), 8) + pad(f(c.impact[3]), 8) + pad(f(c.impact[4]), 8) + f(c.size));
      lines.push("      basis: " + c.note);
      if (c.sensitivity != "") lines.push("      sensitivity: " + c.sensitivity);
    }
    lines.push(pad("RSS", 57) + [for (p in 0...PARAMETERS.length) pad(f(result.rss[p]), 8)].join("") + f(result.sizeRss));
    lines.push(pad("worst case (sum)", 57) + [for (p in 0...PARAMETERS.length) pad(f(result.worst[p]), 8)].join("") + f(result.sizeWorst));
    lines.push(pad("class " + TARGET_CLASS + " limit (ISO 1328-1 formula)", 57) + [for (p in 0...PARAMETERS.length) pad(f(result.limitTarget[p]), 8)].join(""));
    lines.push(pad("class " + (TARGET_CLASS - 1) + " limit", 57) + [for (p in 0...PARAMETERS.length) pad(f(result.limitFiner[p]), 8)].join(""));
    lines.push(pad("predicted class, RSS (equivalent)", 57) + [for (p in 0...PARAMETERS.length) pad(classText(result.classRss[p]) + " (" + Math.round(result.equivalentRss[p] * 10) / 10 + ")", 8)].join(""));
    lines.push(pad("predicted class, worst case", 57) + [for (p in 0...PARAMETERS.length) pad(classText(result.classWorst[p]) + " (" + Math.round(result.equivalentWorst[p] * 10) / 10 + ")", 8)].join(""));
    var overall = 0, unknown = false;
    for (c in result.classRss) {
      if (c == null) unknown = true;
      else overall = Std.int(Math.max(overall, c));
    }
    lines.push('predicted ISO 1328 class (RSS, worst parameter): ${unknown ? "worse than 11" : Std.string(overall)}; target class $TARGET_CLASS');
    return lines;
  }

  /** The top `count` limiting rows with their improvements. */
  public static function limiting(result:AccuracyResult, count:Int = 3):Array<String> {
    var lines = [];
    var rows = ranked(result);
    for (i in 0...Std.int(Math.min(count, rows.length))) {
      var c = rows[i], s = share(c, result.limitTarget);
      lines.push((i + 1) + ". " + c.id + " (" + c.basis + "): " + f(c.impact[PARAMETERS.indexOf(s.parameter)]) + " um on " + s.parameter + " = " + Math.round(s.fraction * 100) + " % of the class "
        + TARGET_CLASS + " limit (" + c.note + "). Improve: " + c.improve);
    }
    var big:AccuracyContributor = null;
    for (c in result.contributors) if (big == null || c.size > big.size) big = c;
    if (big != null && big.size > 0) lines.push("Not class-graded: tooth thickness (size) RSS " + f(result.sizeRss) + " um, largest " + big.id + " " + f(big.size) + " um; static, set by the first-article trim. Improve: " + big.improve);
    return lines;
  }
}
