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
  /**
   * Column sole tilt about the carriage's travel axis, N m/rad, and the hob's height above the sole, mm. The value is the C3D10 FE of the CAD X carriage
   * (`fem/carriage_fem.py`, 2026-11-06: sole on a rigid master, the four block pads held in z; 14.1 MN m/rad with the pads held in all directions); the
   * `frame_stiffness.py` plate model gives 11.75 after its sign fix (it was quoted as 263.9) and overstates the front tie by about 27 %.
   */
  public var soleTilt:Float = 10344513.590821;
  /**
   * Hob tip displacement per newton of cutting force through the Z saddle plate, um/N: the FE of the 13 mm milled plate (`fem/saddle_fem.py`) with the head on the
   * eRob seat, the four blocks held at their bolt patches: 100 N at the hob moves the tip 5.68 um beyond the weight alone (seat tilt 346-377 kN m/rad about y; 21.1 um
   * with the 36.7 kg head's weight, which is a constant offset the first-article trim absorbs). Taken for the radial force too (the FE load is along the cut, down).
   */
  public var saddleTipCompliance:Float = 0.05676;
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

  /**
   * C table position loop and smoothness (the tracking-error model, `HobberAccuracyBudget.trackingTerms`). The C table is position-controlled and follows the
   * B encoder, so what reaches the gear is the loop's tracking error: disturbance torque / dynamic stiffness at the disturbance frequency.
   * Loop: standard cascade, PI velocity (bandwidth `cLoopVelocityHz`, gain Kv = J w_v, integral corner w_v / `cIntegralRatio`) inside a P position loop
   * (Kp = w_v / `cPositionRatio`); no delay, no structural mode (assumed; a vendor drive tuned to these would reach about this). Typical direct-drive
   * table drives run 100-200 Hz velocity bandwidth.
   */
  public var cLoopVelocityHz:Float = 100;
  public var cPositionRatio:Float = 4;
  public var cIntegralRatio:Float = 10;
  /** Mandrel, part and clamp inertia added to the table's rotor/platen inertia, kg m2 (assumed: a 100 mm steel mandrel and part, ~2 kg). */
  public var cLoadInertia:Float = 0.01;
  /** Torque ripple at 1x revolution (current offset/gain error, commutation), as a fraction of the table's continuous torque (assumed typical direct-drive value; the RFQ asks for it); 2x and 3x revolution at 1/2 and 1/4 of it. */
  public var cTorqueRipple:Float = 0.002;
  /** Share of the HB10 table reaction torque that varies at the hob's gash-pass frequency (assumed). */
  public var cCutRippleShare:Float = 0.3;
  /** Gashes (flutes) of the hob: the cutting force repeats at this many times the hob revolution (set from the hob design). */
  public var hobGashes:Int = 12;
  /** Share of the B speed ripple left in the C reference after the master's velocity feedforward (assumed; 1.0 would be no feedforward). */
  public var cFeedforwardResidual:Float = 0.05;
  /** Frequency of the B speed ripple, in B revolutions (assumed, once per B revolution). */
  public var bRippleOrder:Float = 1;

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

/** One smoothness disturbance of the C table as position error: its torque (or encoder error), harmonic order, frequency, loop stiffness there, and angle amplitude. */
class TrackingTerm {
  public final id:String;
  public final kind:String;
  public final basis:String;
  /** Disturbance amplitude: N m (arcsec for the interpolation term). */
  public final amplitude:Float;
  public final angle:Float;
  public final order:Float;
  public final frequencyHz:Float;
  public final stiffness:Float;
  public final note:String;
  public final improve:String;
  public final toothPass:Bool;
  public function new(id:String, kind:String, basis:String, amplitude:Float, angle:Float, order:Float, frequencyHz:Float, stiffness:Float, note:String, improve:String, toothPass:Bool) {
    this.id = id;
    this.kind = kind;
    this.basis = basis;
    this.amplitude = amplitude;
    this.angle = angle;
    this.order = order;
    this.frequencyHz = frequencyHz;
    this.stiffness = stiffness;
    this.note = note;
    this.improve = improve;
    this.toothPass = toothPass;
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
    var compliance = (1 / kHob + 1 / inputs.tableRadialStiffness + tilt + inputs.saddleTipCompliance) * (1 + inputs.unlistedCompliance);
    var deflection = inputs.radialForceN * compliance;
    rowOf(list, "structure.radial", "Structure under load", deflection, "um", "Assumed",
      "radial force " + r2(inputs.radialForceN) + " N (HB10 peak) / [hob side nose " + inputs.noseStiffness + " N/um (assumed) + outboard " + inputs.outboardStiffness + " N/um (model); table "
      + inputs.tableRadialStiffness + " N/um (model); column sole tilt " + r3(inputs.soleTilt / 1e6) + " MN m/rad over " + r2(inputs.hobHeightMm) + " mm (FE of the CAD carriage); Z saddle plate "
      + r3(inputs.saddleTipCompliance) + " um/N at the hob tip (FE)]"
      + " x (1 + " + inputs.unlistedCompliance + " for the unlisted casting, column and blocks, assumed)",
      "size: 2 tan(alpha) x deflection; flank line: the force changes along the 8 mm band, x sin(alpha) = " + r3(sinA), 0, 0, 0, 0, deflection * sinA, tan2 * deflection,
      "Stiffer hob side (cartridge nose stiffness, two-sided outboard), a thicker Z saddle plate (the saddle is now the largest listed structural term), FE of the column and rails to replace the assumed unlisted share");
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
    var fC = cSpeedHz(inputs);
    var kRev = loopStiffness(inputs, inputs.cLoopVelocityHz, fC);
    var dTorque = inputs.tableTorqueNm / kRev;
    rowOf(list, "C.reaction", "C table", dTorque * 1e6, "urad", "Derived", "table torque bound " + inputs.tableTorqueNm + " N m (HB10), varying once per revolution (" + r3(fC) + " Hz), / loop dynamic stiffness there "
      + r2(kRev) + " N m/rad (" + inputs.cLoopVelocityHz + " Hz velocity loop, assumed)", "pitch: 2 r x angle", 0, 2 * r * dTorque * um, 0, 0, 0, 0, "Feedforward of the load torque from B torque; higher loop bandwidth");

    // ---- Rotary axes ------------------------------------------------------------------------------------------------------------
    var gc = "C table";
    function harmonic(id:String, basis:String, amplitude:Float, order:Float, note:String, improve:String):Void {
      rowOf(list, id, gc, amplitude / ARC, "arcsec", basis, note,
        "pitch: Fp = 2 r x angle = " + r3(2 * r * um * ARC) + " um per arcsec; fp = Fp x |sin(order x pi / z)| for order " + order, 2 * r * amplitude * Math.abs(Math.sin(order * Math.PI / z)) * um,
        2 * r * amplitude * um, 0, 0, 0, 0, improve);
    }
    harmonic("C.accuracy", inputs.tableBasis.get("absoluteAccuracy"), tv(inputs, "absoluteAccuracy"), 4, "table positioning accuracy (catalogue), assumed order 4: encoder error is read by the loop, so it is not rejected", "Calibrate the table (error map in the controller)");
    var rep = tv(inputs, "repeatability");
    rowOf(list, "C.repeatability", gc, rep / ARC, "arcsec", inputs.tableBasis.get("repeatability"), "table repeatability (catalogue); acts tooth to tooth",
      "pitch: fp = Fp = 2 r x angle", 2 * r * rep * um, 2 * r * rep * um, 0, 0, 0, 0, "Direct-drive table with a finer encoder; repeatability is the floor for fp");
    for (t in trackingTerms(inputs, inputs.cLoopVelocityHz)) {
      var note = t.note + "; " + r3(t.frequencyHz) + " Hz at " + r2(fC * 60) + " rpm" + (t.stiffness > 0 ? ", dynamic stiffness " + r2(t.stiffness) + " N m/rad (loop " + inputs.cLoopVelocityHz + " Hz)" : "");
      if (t.toothPass) {
        // Every hob facet cuts every tooth at the same C phase (order = gashes x z aliases to zero): no pitch error, a ripple on the flank normal (u cos(alpha)).
        rowOf(list, t.id, gc, t.angle / ARC, "arcsec", t.basis, note, "profile: tangential ripple r x angle x cos(alpha), repeats on every tooth (no pitch error)", 0, 0, 0, r * t.angle * um * cosA, 0, 0, t.improve);
      } else {
        harmonic(t.id, t.basis, t.angle, t.order, note, t.improve);
      }
    }
    var omegaC = 2 * Math.PI * fC;
    var kp = 2 * Math.PI * inputs.cLoopVelocityHz / inputs.cPositionRatio;
    var wf = 2 * Math.PI * inputs.bRippleOrder * inputs.hobRpm / 60;
    var lag = omegaC * inputs.bSpeedRipple / Math.max(kp, wf) * inputs.cFeedforwardResidual;
    rowOf(list, "C.trackLag", gc, lag / ARC, "arcsec", "Assumed", "B speed ripple " + inputs.bSpeedRipple * 100 + " % (assumed) at " + r2(inputs.bRippleOrder * inputs.hobRpm / 60) + " Hz reaches the C reference as speed variation; the position loop (Kp " + r2(kp)
      + " 1/s) follows it with lag omega_C x ripple / max(Kp, w) = " + r2(omegaC * inputs.bSpeedRipple / Math.max(kp, wf) / ARC) + "\" without feedforward, x " + inputs.cFeedforwardResidual + " left after the master's velocity feedforward (assumed)",
      "pitch: 2 r x angle; order " + inputs.bRippleOrder * z + " per revolution", 2 * r * lag * um, 2 * r * lag * um, 0, 0, 0, 0, "Velocity feedforward from the B master; faster cycle");
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

  /** C table speed under the electronic gearbox, Hz: hob revolutions per second x starts / teeth. */
  public static function cSpeedHz(inputs:HobberAccuracyInputs):Float return inputs.hobRpm / 60 * inputs.hobStarts / inputs.spec.teeth;

  /** Total inertia of the C axis, kg m2. */
  public static function cInertia(inputs:HobberAccuracyInputs):Float return tv(inputs, "rotorInertia") + inputs.cLoadInertia;

  /** Torque constant from the table's continuous torque and current, N m/A rms (the table's own `torqueConstant` row has the same basis). */
  public static function torqueConstant(inputs:HobberAccuracyInputs):Float return tv(inputs, "continuousTorque") / tv(inputs, "continuousCurrent");

  /** Disturbance torque to position (dynamic stiffness) and encoder error to position (transmission) of the cascade, at `hz`: the denominator J s^2 + Kv (1 + wi/s)(s + Kp) and numerator Kv (1 + wi/s)(s + Kp). */
  static function loopTerms(inputs:HobberAccuracyInputs, velocityHz:Float, hz:Float):{stiffness:Float, transmission:Float} {
    var j = cInertia(inputs), wv = 2 * Math.PI * velocityHz, kv = j * wv, kp = wv / inputs.cPositionRatio, wi = wv / inputs.cIntegralRatio, w = 2 * Math.PI * Math.max(hz, 1e-6);
    var nr = kv * (kp + wi), ni = kv * (w - wi * kp / w);
    var dr = nr - j * w * w, di = ni;
    var d = Math.sqrt(dr * dr + di * di);
    return {stiffness: d, transmission: Math.sqrt(nr * nr + ni * ni) / d};
  }

  /** Dynamic stiffness of the C position loop at `hz`, N m/rad. */
  public static function loopStiffness(inputs:HobberAccuracyInputs, velocityHz:Float, hz:Float):Float return loopTerms(inputs, velocityHz, hz).stiffness;

  /**
   * The C table's smoothness disturbances turned into position error: torque / dynamic stiffness at the harmonic's frequency (cogging, torque ripple at 1x, 2x, 3x
   * revolution, seal drag ripple at 2x, cutting force at the gash-pass frequency), and the encoder's interpolation error times the loop's transmission at the
   * signal-period frequency. Angle amplitudes, radians. `scalable` terms are the ones the vendor's smoothness numbers set (the RFQ spec).
   */
  public static function trackingTerms(inputs:HobberAccuracyInputs, velocityHz:Float):Array<TrackingTerm> {
    var out:Array<TrackingTerm> = [], fC = cSpeedHz(inputs), z = inputs.spec.teeth;
    function torque(id:String, kind:String, basis:String, amplitudeNm:Float, order:Float, note:String, improve:String, tooth:Bool = false):Void {
      var f = fC * order, k = loopStiffness(inputs, velocityHz, f);
      out.push(new TrackingTerm(id, kind, basis, amplitudeNm, amplitudeNm / k, order, f, k, note, improve, tooth));
    }
    torque("C.trk.cogging", "cogging", inputs.tableBasis.get("coggingTorque"), tv(inputs, "coggingTorque"), 48, "cogging torque " + tv(inputs, "coggingTorque") + " N m peak, 48th harmonic", "Cogging compensation table; skewed or low-cogging motor; vendor value");
    var rip = inputs.cTorqueRipple * tv(inputs, "continuousTorque");
    for (h in 1...4) torque("C.trk.ripple" + h, "ripple", "Assumed", rip / Math.pow(2, h - 1), h, "torque ripple " + inputs.cTorqueRipple * 100 / Math.pow(2, h - 1) + " % of continuous torque " + tv(inputs, "continuousTorque") + " N m (assumed), " + h + "x revolution", "Current-loop offset and gain calibration; vendor ripple figure");
    torque("C.trk.seal", "seal", inputs.tableBasis.get("sealDragRipple"), tv(inputs, "sealDragRipple"), 2, "seal drag ripple " + tv(inputs, "sealDragRipple") + " N m peak, twice per revolution", "Low-drag or non-contact seal");
    torque("C.trk.cut", "cut", "Assumed", inputs.cCutRippleShare * inputs.tableTorqueNm, inputs.hobGashes * z, "cutting torque variation " + inputs.cCutRippleShare * 100 + " % (assumed) of the " + inputs.tableTorqueNm + " N m reaction at the gash-pass frequency (" + inputs.hobGashes + " gashes)", "Hob with more gashes; none needed", true);
    var order = 4096, f = fC * order, tr = loopTerms(inputs, velocityHz, f).transmission, amp = tv(inputs, "interpolationError");
    var t = new TrackingTerm("C.trk.interpolation", "interpolation", inputs.tableBasis.get("interpolationError"), amp / ARC, amp * tr, order, f, 0, "encoder interpolation " + r2(amp / ARC) + "\" at the 4096th harmonic ("
      + r2(f) + " Hz) x loop transmission " + Std.string(Math.round(tr * 1e4) / 1e4) + " there", "Encoder with a stated, low interpolation error; two read heads", false);
    out.push(t);
    return out;
  }

  /**
   * The C smoothness specification for the RFQ: for each velocity-loop bandwidth, the largest cogging torque, torque ripple (fraction of continuous torque at 1x revolution,
   * 2x and 3x at the same ratios), seal ripple and encoder interpolation error that each keep Fp at `limit` alone, and the common scale of the assumed values that does
   * so together, every other row unchanged (RSS).
   */
  public static function smoothnessSpec(inputs:HobberAccuracyInputs, rows:Array<AccuracyContributor>, limit:Float, bandwidths:Array<Float>):Array<String> {
    var lines = [], r2x = 2 * inputs.spec.pitchRadius() * 1000;
    var others = 0.0;
    for (c in rows) if (c.id.indexOf("C.trk.") != 0) others += c.impact[1] * c.impact[1];
    var room = limit * limit - others;
    lines.push("C table smoothness spec (Fp class " + TARGET_CLASS + " limit " + r2(limit) + " um; every row outside the C smoothness terms uses " + r2(Math.sqrt(others)) + " um RSS, room " + (room > 0 ? Std.string(r2(Math.sqrt(room))) : "none") + " um):");
    for (fv in bandwidths) {
      var terms = trackingTerms(inputs, fv), total = 0.0, kinds:Map<String, Float> = new Map(), amps:Map<String, Float> = new Map();
      for (t in terms) if (!t.toothPass) {
        var fp = r2x * t.angle;
        total += fp * fp;
        var k0 = got(kinds, t.kind);
        kinds.set(t.kind, k0 + fp * fp);
        var a0 = got(amps, t.kind);
        amps.set(t.kind, a0 + t.amplitude);
      }
      function alone(kind:String, unit:Float, name:String):String {
        var sumSq = got(kinds, kind), own:Float = Math.sqrt(sumSq), rest = total - own * own, left = room - rest;
        if (left <= 0) return name + " none";
        var first = got(amps, kind);
        return name + " <= " + Std.string(Math.round(first * Math.sqrt(left) / own / unit * 1000) / 1000);
      }
      var k = room > 0 ? Math.sqrt(room / total) : 0.0;
      lines.push("  velocity loop " + fv + " Hz (position " + r2(fv / inputs.cPositionRatio) + " Hz): Fp from the assumed smoothness set " + r2(Math.sqrt(total)) + " um; each alone: " + alone("cogging", 1, "cogging N m")
        + ", " + alone("ripple", tv(inputs, "continuousTorque") / 100 * 1.75, "ripple % of continuous torque at 1x rev") + ", " + alone("seal", 1, "seal ripple N m") + ", " + alone("interpolation", 1, "interpolation arcsec")
        + "; all together scaled x" + r2(k) + " (cogging " + r3(k * tv(inputs, "coggingTorque")) + " N m, ripple " + r3(k * inputs.cTorqueRipple * 100) + " %, seal " + r3(k * tv(inputs, "sealDragRipple")) + " N m, interpolation " + r2(k * tv(inputs, "interpolationError") / ARC) + "\")");
    }
    return lines;
  }

  static function got(m:Map<String, Float>, k:String):Float {
    var v = m.get(k);
    return v == null ? 0.0 : v;
  }

  /** The earlier view for comparison: the once-per-turn speed ripple integrated to angle, and what the position loop leaves of it. */
  public static function speedRippleCrossCheck(inputs:HobberAccuracyInputs):Array<String> {
    var rho = tv(inputs, "speedRipple"), r = inputs.spec.pitchRadius(), fC = cSpeedHz(inputs), kp = 2 * Math.PI * inputs.cLoopVelocityHz / inputs.cPositionRatio;
    var open = 2 * r * rho * 1000, w = 2 * Math.PI * fC, sp = w / Math.sqrt(w * w + kp * kp);
    var equivalentTorque = rho * w * cInertia(inputs) * w;
    return ["Cross-check (the earlier view): a " + rho * 100 + " % once-per-turn speed ripple at " + r2(fC * 60) + " rpm integrated to angle is " + r2(open) + " um of Fp (2 r x " + rho + " rad), the open-loop (velocity mode) figure.",
      "  In position mode with the B encoder as reference the position loop corrects it: sensitivity |s / (s + Kp)| = " + Std.string(Math.round(sp * 1e4) / 1e4) + " at " + r3(fC) + " Hz (Kp " + r2(kp) + " 1/s), leaving "
      + r2(open * sp) + " um; a vendor's speed ripple is measured in velocity mode, so it overstates what a position-controlled table does. The same torque, seen as a disturbance, is "
      + Std.string(Math.round(equivalentTorque * 1e6) / 1e6) + " N m (J w^2 x ripple) at that frequency, far below the cogging and ripple torques that now decide."];
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
