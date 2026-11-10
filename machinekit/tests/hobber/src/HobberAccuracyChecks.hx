import machinekit.hobbing.*;
import machinekit.hobbing.HobberAccuracyBudget;
import machinekit.motion.PrecisionErrors;
import materia.assembly.AssemblyFrames as F;

/**
 * Accuracy budget of the flexspline hobber (FRAME_PLAN, "Accuracy budget"): reads the machine, builds `HobberAccuracyBudget`'s rows, prints the
 * budget, and fails when the predicted ISO 1328 class (RSS) is worse than the target on any parameter.
 *
 * `haxeon.accuracy.json`: no argument prints the budget with the tilt table loaded and gates on it; `report` prints without gating;
 * `uncompensated` adds the budget before the tilt-compensation step.
 * Read-only use of `FlexsplineHobber`; values the model does not hold are set here or in `HobberAccuracyInputs` with their basis.
 */
class HobberAccuracyChecks {
  static var assertions = 0;
  static function check(ok:Bool, label:String):Void {
    assertions++;
    if (!ok) throw label;
  }
  static function near(a:Float, b:Float, e:Float, label:String):Void check(Math.abs(a - b) <= e, label + ": " + a + " vs " + b);

  /** Abbe offset (scale line to hob centre) and the thermal loop lengths of each scale, from the posed machine at the initial joint values. */
  static function axes(machine:FlexsplineHobber):Array<AccuracyAxis> {
    var poses = machine.solvedPoses(), back = F.inverse(machinekit.hobbing.HobberFrame.workToMachine()), out = [];
    var tool = F.compose(poses.get("hob"), machine.hob.connector("centre").frame);
    var toolWork = F.transformPoint(back, tool.x, tool.y, tool.z);
    for (a in HobberScales.all()) {
      var housing = poses.get(HobberScales.member(a.id)), head = poses.get(HobberScales.member(a.id,true));
      if (housing == null || head == null) throw 'axis ${a.id} has no scale members';
      var origin = F.transformPoint(back, housing.x, housing.y, housing.z);
      var worldAlong = F.transformVector(housing, 0, 0, 1), along = F.transformVector(back, worldAlong.x, worldAlong.y, worldAlong.z);
      var centre = F.transformPoint(back, head.x, head.y, head.z);
      var u = [along.x, along.y, along.z];
      // The Abbe offset is from the measuring line (the housing's `axis` connector: under the groove floor of the reading face), not the hole line the housing is bolted on.
      var ax = F.compose(housing, a.housing.connector("axis").frame), line = F.transformPoint(back, ax.x, ax.y, ax.z);
      var d = [toolWork.x - line.x, toolWork.y - line.y, toolWork.z - line.z];
      var along0 = d[0] * u[0] + d[1] * u[1] + d[2] * u[2];
      var across = [for (i in 0...3) d[i] - along0 * u[i]];
      var item = new AccuracyAxis();
      item.id = a.id;
      item.scale = a.housing.designation;
      item.accuracyUm = a.housing.accuracy;
      item.measuringLengthMm = a.housing.measuringLength;
      item.abbeMm = Math.sqrt(across[0] * across[0] + across[1] * across[1] + across[2] * across[2]);
      item.steelLoopMm = Math.abs((toolWork.x - centre.x) * u[0] + (toolWork.y - centre.y) * u[1] + (toolWork.z - centre.z) * u[2]);
      item.castLoopMm = Math.abs(origin.x * u[0] + origin.y * u[1] + origin.z * u[2]);
      out.push(item);
    }
    return out;
  }

  /** The model's inputs: spec, hob, scales and Abbe offsets, table fields, part runouts, swivel accuracy, mandrel. Everything else is `HobberAccuracyInputs`'s labelled assumption. */
  public static function inputs(machine:FlexsplineHobber):HobberAccuracyInputs {
    var i = new HobberAccuracyInputs(machine.spec);
    var design = machine.hob.design;
    i.pressureAngle = machine.hob.parameterNumber("pressureAngle");
    i.hobLeadAngle = design.leadAngle;
    i.hobStarts = design.starts;
    i.hobGashes = design.gashes;
    i.hobRpm = 300000 / (Math.PI * design.outsideDiameter);
    i.axes = axes(machine);
    // Stroke and height of the cut: the Z travel and the hob's height above the column sole (work z 280, FRAME_PLAN H1).
    for (t in machine.travel) if (t.id == "Z") i.strokeMm = t.upper - t.lower;
    var poses = machine.solvedPoses(), back = F.inverse(machinekit.hobbing.HobberFrame.workToMachine());
    var tool = F.transformPoint(F.compose(back, F.compose(poses.get("hob"), machine.hob.connector("centre").frame)), 0, 0, 0);
    i.hobHeightMm = Math.abs(tool.z - 280.0);
    i.aLeverMm = Math.sqrt(tool.y * tool.y + Math.pow(tool.z - HobberZLayout.SEAT_Z, 2)) + i.aEngagementMm;
    for (row in machine.tableSpec.all()) {
      i.table.set(row.name, row.value);
      i.tableBasis.set(row.name, Std.string(row.basis));
    }
    i.tableRadialStiffness = machine.tableSpec.number("radialStiffness") / 1000;
    i.tablePivotMm = machine.mandrel.parameterNumber("workSeat") + machine.tableSpec.number("pivotBelowFace");
    var outboard:OutboardSupport = cast machine.components().filter(m -> m.id == "outboard")[0].component;
    i.outboardStiffness = outboard.parameterNumber("radialStiffness") / 1e6;
    i.bCounts = machine.spindle.encoder.countsPerRevolution;
    i.mandrelEccentricityUm = machine.mandrel.parameterNumber("eccentricity") * 1000;
    i.mandrelRoundnessUm = machine.mandrel.parameterNumber("clampedRoundness") * 1000;
    i.mandrelBandVariationUm = machine.mandrel.parameterNumber("bandExpansionVariation") * 1000;
    for (e in PrecisionErrors.inAssembly(machine)) {
      var part = e.part, q = e.error.quantity;
      if (StringTools.startsWith(part, "swivel")) {
        if (q == "outputAbsoluteAccuracy") i.aAbsoluteArcsec = e.error.bound;
        if (q == "outputRepeatability") i.aRepeatArcsec = e.error.bound;
      } else if ((part == "hob" || StringTools.startsWith(part, "spindle") || part == "outboard") && (q == "radialRunout" || q == "axialRunout" || q == "profileError")) {
        var u = new AccuracyRunout();
        u.part = part;
        u.quantity = q;
        u.micrometres = e.error.bound * 1000;
        u.basis = Std.string(e.error.basis);
        i.runouts.push(u);
      }
    }
    return i;
  }

  static function machineTable(i:HobberAccuracyInputs, name:String):Float {
    var v = i.table.get(name);
    if (v == null) throw "table value missing: " + name;
    return v;
  }

  /** Unit checks on the formulas and mappings, against hand values. */
  static function selfTest(i:HobberAccuracyInputs):Void {
    var s = i.spec;
    near(gearkit.Iso1328.tolerance(s, "fp", 6, false), 7.3, 0.1, "class 6 fp at d 60 m 0.3 (PLAN.md reference numbers)");
    near(gearkit.Iso1328.tolerance(s, "Fp", 6, false), 23.5, 0.5, "class 6 Fp at d 60 m 0.3");
    near(2 * Math.tan(i.pressureAngle), 0.7279, 0.001, "radial error to tooth thickness 2 tan(alpha)");
    check(i.axes.length == 3, "scales X, Y, Z");
    check(i.runouts.length >= 3, "hob chain runouts read from the machine");
    // Known case: a C error of 1" over r 30 mm is 2 r x 4.848e-6 = 0.291 um pk-pk.
    var rows = HobberAccuracyBudget.build(i, true);
    var accuracy:AccuracyContributor = null;
    for (c in rows) if (c.id == "C.accuracy") accuracy = c;
    check(accuracy != null, "C.accuracy row");
    near(accuracy.impact[1], 2 * s.pitchRadius() * machineTable(i, "absoluteAccuracy") * 1000, 1e-9, "C accuracy Fp = 2 r angle");
    // C loop: low-frequency stiffness is huge, high-frequency stiffness tends to J w^2, the encoder error transmission to 1 and then 0.
    var kLow = HobberAccuracyBudget.loopStiffness(i, 100, 0.25), kHigh = HobberAccuracyBudget.loopStiffness(i, 100, 5000);
    check(kLow > 1e5 && kLow < 1e7, "C loop stiffness at the C speed: " + kLow);
    near(kHigh / (HobberAccuracyBudget.cInertia(i) * Math.pow(2 * Math.PI * 5000, 2)), 1, 0.02, "C loop stiffness at 5 kHz is the inertia");
    var terms = HobberAccuracyBudget.trackingTerms(i, 100);
    for (t in terms) if (t.id == "C.trk.cogging") near(t.angle, t.amplitude / t.stiffness, 1e-15, "cogging position error = torque / stiffness");
    check(HobberAccuracyBudget.trackingTerms(i, 200)[0].angle < terms[0].angle, "a faster loop rejects cogging better");
    // The compensated Abbe error is the residual share of the uncompensated one.
    var comp = HobberAccuracyBudget.build(i, true), raw = HobberAccuracyBudget.build(i, false), a = 0.0, b = 0.0;
    for (c in comp) if (c.id == "X.abbe(compensated)") a = c.value;
    for (c in raw) if (c.id == "X.abbe(uncompensated)") b = c.value;
    near(a, b * i.compensationResidual, 1e-9, "compensation residual");
    // Totals: RSS never exceeds the plain sum.
    var result = HobberAccuracyBudget.summarise(i, comp, true);
    for (p in 0...HobberAccuracyBudget.PARAMETERS.length) check(result.rss[p] <= result.worst[p] + 1e-9, "rss <= worst");
  }

  public static function main():Void {
    var args = Sys.args(), began = Sys.time();
    var machine = new FlexsplineHobber();
    Sys.println('machine built in ${Math.round(Sys.time() - began)} s');
    var i = inputs(machine);
    selfTest(i);
    Sys.println('self test ok ($assertions assertions)');
    Sys.println("ISO 1328-1:2013 class limits from the standard's formulas (gearkit.Iso1328: fp = 0.001 d + 0.4 m + 5, Fp = 0.002 d + 0.55 sqrt(d) + 0.7 m + 12, Fr = 0.9 Fp, "
      + "profile and helix terms from the standard, class 5 base times 2^((class-5)/2)). m 0.3 is below the standard's module range (m >= 0.5): extrapolated classes.");
    Sys.println('part errors read from the machine: ${[for (u in i.runouts) u.part + "." + u.quantity + "=" + u.micrometres + "um(" + u.basis + ")"].join(", ")}');
    if (args.indexOf("uncompensated") >= 0) {
      var raw = HobberAccuracyBudget.run(i, false);
      for (line in HobberAccuracyBudget.report(i, raw, false)) Sys.println(line);
      Sys.println("failures before compensation: " + HobberAccuracyBudget.failures(raw).join("; "));
    }
    var result = HobberAccuracyBudget.run(i, true);
    for (line in HobberAccuracyBudget.report(i, result)) Sys.println(line);
    Sys.println("top limiting contributors (share of the class " + HobberAccuracyBudget.TARGET_CLASS + " limit on the worst parameter):");
    for (line in HobberAccuracyBudget.limiting(result)) Sys.println("  " + line);
    for (line in HobberAccuracyBudget.speedRippleCrossCheck(i)) Sys.println(line);
    for (line in HobberAccuracyBudget.smoothnessSpec(i, result.contributors, result.limitTarget[1], [50, 100, 200])) Sys.println(line);
    var failures = HobberAccuracyBudget.failures(result);
    if (failures.length > 0) {
      for (f in failures) Sys.println("GATE FAIL " + f);
      if (args.indexOf("report") < 0) {
        Sys.println('accuracy budget: FAILED (${failures.length} parameter(s) worse than class ${HobberAccuracyBudget.TARGET_CLASS})');
        Sys.exit(1);
      }
    }
    Sys.println('accuracy budget: ${failures.length == 0 ? "passed" : "reported"} in ${Math.round(Sys.time() - began)} s');
  }
}
