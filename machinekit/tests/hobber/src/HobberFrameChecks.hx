import machinekit.assembly.AssemblyIntegrity;
import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineKitComponents;
import machinekit.hobbing.FoundationLayout;
import machinekit.hobbing.HobSpindle;
import machinekit.hobbing.HobberCastingPockets;
import machinekit.hobbing.HobberSaddle;
import machinekit.hobbing.HobberSpindleRelief;
import machinekit.hobbing.HobberXLayout;
import machinekit.hobbing.HobberScales;
import machinekit.hobbing.SpindleHousing;
import machinekit.hobbing.HobberSwivelLayout;
import machinekit.hobbing.HobberZLayout;
import materia.assembly.AssemblyFrames as F;

/** Frame, drive and integrity checks on the assembled hobber (FRAME_PLAN.md H1-H6). */
class HobberFrameChecks {
  /** `AssemblyIntegrity` in strict mode on the whole machine; `sweep` false checks the rest state only. */
  public static function integrity(machine:FlexsplineHobber, sweep:Bool, gap:Float = 2.0):Array<String> {
    // The column is lowered onto the carriage after the carriage's block and nut-block screws are fitted: those screws are reached before it is in place
    // (the assembly order, per fastener: the column still blocks every other screw's driver and counts for holes and heads).
    var options:IntegrityOptions = {strict: true, processContacts: machine.processContacts(), runningGap: gap,
      fittedBefore: [{member: "column/", fastened: ["Xbridge"]}]};
    if (!sweep) options.sweepStates = [];
    var report = AssemblyIntegrity.check(machine, options);
    return [for (item in report.errors()) item.code + " " + item.message];
  }

  /** Assumed guide pitch/yaw error over the travel, arcseconds (no measurement yet): sets the uncompensated Abbe error printed by `scales`. */
  static inline final GUIDE_ANGLE_ERROR_ARCSEC = 5.0;
  static inline final GUIDE_ANGLE_ERROR_RAD = GUIDE_ANGLE_ERROR_ARCSEC * Math.PI / 648000;

  /**
   * The members whose strict-integrity findings are open, with the reason for each. A finding on any other member fails the check, so the
   * frames, drives, guides, casting and anchors stay covered.
   * Empty: the last one, the spindle servo against the bed casting, is fixed by the relief in the casting (`HobberSpindleRelief`).
   */
  static final OPEN_MEMBERS:Array<String> = [];

  /** Integrity errors that do not touch an open member (see `OPEN_MEMBERS`). */
  public static function newErrors(machine:FlexsplineHobber, sweep:Bool):Array<String> {
    var found = integrity(machine, sweep);
    function open(error:String):Bool {
      for (member in OPEN_MEMBERS) if (error.indexOf(member) >= 0) return true;
      return false;
    }
    return [for (error in found) if (!open(error)) error];
  }

  /** The joint's positive direction in the work frame, per axis. */
  static function jointDirection(id:String):Array<Float> return id == "X" ? [1.0, 0, 0] : id == "Y" ? [0.0, -1, 0] : [0.0, 0, 1];

  /**
   * The sealed scales (FRAME_PLAN H4): the measuring length is the smallest standard one for the stroke, the head's centre stays inside
   * the measuring window over the whole stroke plus overtravel, and the Abbe offsets are reported. The Abbe offset of an axis is the
   * distance from the housing's line (the face toward the head, along the travel) to the hob centre, across the travel, in the work
   * frame at the initial joint values: two components, along the two work axes that are not the travel, and their length. Returns
   * one line per axis; throws when the head can leave the window or the scale is not where the machine says.
   */
  public static function scales(machine:FlexsplineHobber):Array<String> {
    var poses = machine.solvedPoses(), lines = [];
    var back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    var tool = F.compose(poses.get("hob"), machine.hob.connector("centre").frame);
    var toolWork = F.transformPoint(back, tool.x, tool.y, tool.z);
    for (a in HobberScales.all()) {
      var travel = machine.travel.filter(t -> t.id == a.id)[0];
      var housing = poses.get(a.id + "scale"), head = poses.get(a.id + "scaleHead");
      if (housing == null || head == null) throw 'axis ${a.id} has no scale members';
      var origin = F.transformPoint(back, housing.x, housing.y, housing.z);
      var worldAlong = F.transformVector(housing, 0, 0, 1), along = F.transformVector(back, worldAlong.x, worldAlong.y, worldAlong.z);
      var centre = F.transformPoint(back, head.x, head.y, head.z);
      var d = [toolWork.x - origin.x, toolWork.y - origin.y, toolWork.z - origin.z];
      var u = [along.x, along.y, along.z], along0 = d[0] * u[0] + d[1] * u[1] + d[2] * u[2];
      var across = [for (i in 0...3) d[i] - along0 * u[i]];
      var sense = jointDirection(a.id), sign = sense[0] * u[0] + sense[1] * u[1] + sense[2] * u[2];
      // Head position along the scale: at the initial joint value, then over the stroke with the guide overtravel (5 mm each end).
      var here = (centre.x - origin.x) * u[0] + (centre.y - origin.y) * u[1] + (centre.z - origin.z) * u[2];
      var from = here + sign * (travel.lower - 5 - travel.initial), to = here + sign * (travel.upper + 5 - travel.initial);
      var low = Math.min(from, to), high = Math.max(from, to), start = a.housing.startOffset(), end = start + a.housing.measuringLength;
      var names = ["x", "y", "z"], parts = [for (i in 0...3) if (Math.abs(u[i]) < 0.5) '${names[i]} ${Math.round(Math.abs(across[i]) * 10) / 10}'];
      var total = Math.sqrt(across[0] * across[0] + across[1] * across[1] + across[2] * across[2]);
      var line = '${a.id} scale ${a.housing.designation}: Abbe offset ${parts.join(", ")} mm (${Math.round(total * 10) / 10} mm), uncompensated Abbe error ${Math.round(total * GUIDE_ANGLE_ERROR_RAD * 1e4) / 10} um at an assumed ${GUIDE_ANGLE_ERROR_ARCSEC}" guide angular error (assumed; compensated by the tilt table, see the compensate-tilt procedures), head ' +
        'window ${Math.round(low * 10) / 10}..${Math.round(high * 10) / 10} of ${Math.round(start * 10) / 10}..${Math.round(end * 10) / 10}';
      lines.push(line);
      if (low < start - 1e-6 || high > end + 1e-6) throw "scale head leaves its measuring window: " + line;
    }
    return lines;
  }

  /**
   * The head on the A axis (FRAME_PLAN R3): the mass that turns with the actuator's output flange and its centre of mass, and what that
   * puts on the output bearing and the brake, at the middle position and at A +-15 deg. Work frame, N m. The actuator's bearing moment
   * rating and brake holding torque are not published (asked from ZeroErr); a missing one is reported as unknown, and an assumed one
   * is labelled. Gravity is -z; the output bearing's tilting moment is taken about the output face (the bearing lies behind it, which
   * adds `force x offset`, under 1 N m here). The cutting force is 100 N at the hob, in the direction that loads the bearing most.
   * Returns one line per quantity; throws when the demand exceeds a rating that is known.
   */
  public static function swivelLoad(machine:FlexsplineHobber):Array<String> {
    var def = machine.definition(), poses = machine.solvedPoses(), back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    var turning = new Map<String, Bool>();
    turning.set("swivel/output", true);
    var changed = true;
    while (changed) {
      changed = false;
      for (j in def.joints) if (turning.exists(j.parent) && !turning.exists(j.child)) {
        turning.set(j.child, true);
        changed = true;
      }
    }
    var mass = 0.0, mx = 0.0, my = 0.0, mz = 0.0, count = 0;
    for (m in machine.components()) if (turning.exists(m.id)) {
      var props = m.component.massProperties(), at = F.transformPoint(F.compose(back, poses.get(m.id)), props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      mass += props.mass;
      mx += props.mass * at.x;
      my += props.mass * at.y;
      mz += props.mass * at.z;
      count++;
    }
    var com = {x: mx / mass, y: my / mass, z: mz / mass - HobberZLayout.SEAT_Z};
    var g = 9.80665, face = HobberSwivelLayout.OUTPUT_X, overhang = (face - com.x) / 1000;
    var tilt = mass * g * overhang;
    // Gravity torque about A: the centre of mass is (dy, dz) from the axis; turned by theta its lever on the vertical force is dy cos - dz sin.
    function gravityTorque(theta:Float):Float return mass * g * (com.y * Math.cos(theta) - com.z * Math.sin(theta)) / 1000;
    var worst = 0.0, at0 = gravityTorque(0), limit = HobberSwivelLayout.RANGE;
    for (i in -15...16) worst = Math.max(worst, Math.abs(gravityTorque(i * Math.PI / 180)));
    var tool = F.transformPoint(F.compose(back, F.compose(poses.get("hob"), machine.hob.connector("centre").frame)), 0, 0, 0);
    var hobOverhang = (face - tool.x) / 1000, hobLever = Math.sqrt(Math.pow(tool.y, 2) + Math.pow(tool.z - HobberZLayout.SEAT_Z, 2)) / 1000;
    var cutting = 100.0, cutTilt = cutting * hobOverhang, cutTorque = cutting * hobLever;
    var demandTorque = worst + cutTorque, bearingMoment = tilt + cutTilt;
    var row = machine.swivel.housing.ratioRow, moment = machine.swivel.housing.momentRating(), holding = machine.swivel.housing.brakeHoldingTorque();
    function r(v:Float, digits:Int = 2):Float return Math.round(v * Math.pow(10, digits)) / Math.pow(10, digits);
    var old = new machinekit.hobbing.SwivelHead(new machinekit.motion.TorqueMotor(new machinekit.component.ComponentValues().setNumber("height", 20)
      .setNumber("ratedSpeed", Math.PI / 12).setNumber("maxSpeed", Math.PI / 6).setNumber("torqueConstant", 2)
      .setNumber("backEmfConstant", 2 / Math.sqrt(3))), null, Math.PI / 12);
    var oldMass = 0.0;
    for (m in old.components()) oldMass += m.component.massProperties().mass;
    var newMass = 0.0;
    for (id in ["swivel/housing", "swivel/output", "swivelAdapter"]) for (m in machine.components()) if (m.id == id) newMass += m.component.massProperties().mass;
    var lines = [
      'A head: ${r(mass)} kg on the output flange (${count} members), centre of mass x ${r(com.x, 1)} y ${r(com.y, 1)} z ${r(com.z, 1)} mm from the A axis; ' +
      'overhang ${r(overhang * 1000, 1)} mm in front of the output face',
      'A head change: swivel stack was ${r(oldMass)} kg (torque motor, crossed roller, clamp, housing, platen), now actuator ${r(machine.swivel.housing.massProperties().mass + machine.swivel.output.massProperties().mass)} kg ' +
      '+ adapter ring, ${r(newMass - machine.swivel.housing.massProperties().mass - machine.swivel.output.massProperties().mass)} kg',
      'A output bearing: gravity tilting moment ${r(tilt)} N m, cutting ${cutting} N at the hob (${r(hobOverhang * 1000, 1)} mm out) ${r(cutTilt)} N m, together ${r(bearingMoment)} N m; ' +
      (moment == null ? "moment rating: unknown (asked ZeroErr)" : 'moment rating ${moment} N m (assumed), margin ${r(moment / bearingMoment)}x') +
      '; reference: Honpine HPJM-RE80-110 static moment 260 N m (a different product), our ${r(bearingMoment)} N m is ${r(100 * bearingMoment / 260, 1)} % of it',
      'A torque about the axis: gravity ${r(at0)} N m at 0, ${r(worst)} N m at most over +-15 deg; cutting ${r(cutTorque)} N m (${cutting} N at a ${r(hobLever * 1000, 1)} mm lever); ' +
      'demand ${r(demandTorque)} N m against rated ${row.ratedTorque} / peak ${row.peakTorque} N m (${r(row.ratedTorque / demandTorque, 1)}x rated)',
      'A brake: must hold ${r(demandTorque)} N m with the power off; ' +
      (holding == null ? "holding torque: unknown (asked ZeroErr)" : 'holding torque ${holding} N m (ASSUMED, not vendor data), ${r(holding / demandTorque, 1)}x the demand')
    ];
    if (demandTorque > row.ratedTorque) throw "A head demand exceeds the actuator's rated torque: " + lines.join("; ");
    if (moment != null && bearingMoment > moment) throw "A head moment exceeds the output bearing rating: " + lines.join("; ");
    if (holding != null && demandTorque > holding) throw "A brake cannot hold the head: " + lines.join("; ");
    return lines;
  }


  /**
   * The B spindle belt drive (head redesign): the belt's pull on the cartridge's rear bearing, and the first mode of the servo on its
   * cradle against the hob's tooth-pass frequency over the hob speed range. ESTIMATES from the model's masses and sections with stated joint
   * assumptions (preloaded, dowelled bolted joints: axial stiffness 3 x the bolt's); they say where the mode is, not that the part is released.
   * Lines: belt tangential force and pretension (slack side kept at a quarter of the peak force, an assumed rule), the radial pull and
   * what it does to the 6205 tail bearing (catalogue ratings assumed from the maker), then the servo group's mass and centre, each stiffness
   * and mode, and the tooth-pass band (hob rpm x gashes / 60, from 100 m/min to the cartridge's top speed) with +-20 %. Throws when the first
   * mode lies inside that band.
   */
  public static function spindleDrive(machine:FlexsplineHobber):Array<String> {
    var spindle = machine.spindle, rating = spindle.motor.rating, poses = machine.solvedPoses(), back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    function r(v:Float, digits:Int = 2):Float return Math.round(v * Math.pow(10, digits)) / Math.pow(10, digits);
    var lines = [];
    // Belt pull on the cartridge pulley.
    var rm = spindle.motorPulley.pitchDiameter / 2000, rs = spindle.spindlePulley.pitchDiameter / 2000, c = HobSpindle.centres() / 1000;
    var fRated = rating.ratedTorque / rm, fPeak = rating.peakTorque / rm, t0 = HobSpindle.pretension(rating.peakTorque, spindle.motorPulley.pitchDiameter);
    var radial = 2 * t0 * Math.cos(Math.asin((rs - rm) / c));
    var pulleyZ = HobSpindle.pulleyFront(rating.shaftLength) - HobSpindle.PULLEY_WIDTH / 2, tailZ = 7.5, pairZ = HobSpindle.FRONT_BEARING_Z;
    var overhang = tailZ - pulleyZ, span = pairZ - tailZ, tail = radial * (1 + overhang / span), pair = radial * overhang / span;
    var c0 = 7800.0, dyn = 14800.0, nTop = spindle.maxSpeed() * 60 / (2 * Math.PI);
    var l10 = 1e6 / (60 * nTop) * Math.pow(dyn / tail, 3);
    lines.push('B belt: HTD5M ${spindle.motorPulley.teeth}T/${spindle.spindlePulley.teeth}T, ${r(spindle.belt.length, 1)} mm belt on ${r(c * 1000, 1)} mm centres, motor turns ${r(spindle.ratio, 3)}x the hob; tangential force ${r(fRated)} N rated / ${r(fPeak)} N motor peak, '
      + 'pretension ${r(t0)} N per strand (slack side kept at 25 % of the peak force: assumed rule, belt maker to confirm), tight side ${r(t0 + fPeak / 2)} N at peak; '
      + 'set by the jack screw: free span ${r(c * 1000, 0)} mm rings at ${r(HobSpindle.spanFrequency(t0), 0)} Hz (${HobSpindle.BELT_MASS_PER_M} kg/m assumed), needs ${r(HobSpindle.tensioningTravel(t0, spindle.belt.length), 2)} mm more centres than slack, slots give +-${HobSpindle.TENSION_TRAVEL} mm');
    lines.push('Cartridge rear radial load (belt pull): ${r(radial)} N on the pulley, ${r(overhang, 1)} mm behind the 6205 tail bearing: tail bearing ${r(tail)} N (${r(100 * tail / c0, 1)} % of C0 ${c0} N, L10 ${Math.round(l10 / 1000) * 1000 / 1000}k h at ${Math.round(nTop)} rpm; SKF 6205 ratings C ${dyn} N, C0 ${c0} N assumed from the maker), '
      + 'opposite ${r(pair)} N on the angular-contact pair; the cartridge must be a belt-drive cartridge rated for at least ${Math.ceil(radial * 1.5 / 50) * 50} N radial at the pulley (1.5 x the pull)');
    // The servo group on its cradle.
    var ids = ["spindle/motor", "spindle/motorShaft", "spindle/motorPulley", "spindle/motorEncoder"], mass = 0.0, mx = 0.0, my = 0.0, mz = 0.0, cradle = 0.0;
    for (m in machine.components()) {
      var weight = ids.indexOf(m.id) >= 0 ? 1.0 : m.id == "motorCradle" ? 1.0 / 3 : 0.0;
      if (weight == 0) continue;
      var props = m.component.massProperties(), at = F.transformPoint(F.compose(back, poses.get(m.id)), props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      mass += weight * props.mass;
      mx += weight * props.mass * at.x;
      my += weight * props.mass * at.y;
      mz += weight * props.mass * (at.z - HobberZLayout.SEAT_Z);
      if (m.id == "motorCradle") cradle = props.mass;
    }
    var com = {x: mx / mass, y: my / mass, z: mz / mass}, sec = HobberSaddle.cradleSection();
    var e = 210000.0, kJoint = 3 * e * 36.6 / 28.0;
    // Root section about x (bending in y-z): the wall at the rear of the section and the rib ahead of it; about y the rib stands at its x.
    var width = sec.width, t = sec.wall, tr = sec.ribThickness, d = sec.ribDepth;
    var aw = width * t, ar = tr * d, yw = d - t / 2, yr = d / 2, yc = (aw * yw + ar * yr) / (aw + ar);
    var iX = width * Math.pow(t, 3) / 12 + aw * Math.pow(yw - yc, 2) + tr * Math.pow(d, 3) / 12 + ar * Math.pow(yr - yc, 2);
    var xMid = sec.x0 + width / 2, xc = (aw * xMid + ar * sec.ribX) / (aw + ar);
    var iY = t * Math.pow(width, 3) / 12 + aw * Math.pow(xMid - xc, 2) + d * Math.pow(tr, 3) / 12 + ar * Math.pow(sec.ribX - xc, 2);
    var height = com.z - (sec.floorTop), ahead = 239.0 - com.y;
    var sumY2 = 0.0, sumX2 = 0.0, yMean = 0.0;
    for (b in sec.bolts) yMean += b.y / sec.bolts.length;
    for (b in sec.bolts) {
      sumY2 += (b.y - yMean) * (b.y - yMean);
      sumX2 += b.x * b.x;
    }
    // N/mm at the group's centre of mass.
    function hz(k:Float):Float return Math.sqrt(k * 1000 / mass) / (2 * Math.PI);
    // Load-path elements of the axial mode (bending in y-z): the wall (and rib) as a cantilever from the foot top, the foot (the floor under the wall) as a
    // cantilever of the wall's width from the nearest bolt row to the wall's mid-plane, and the floor joint (the bolt rows rocking). The foot is taken
    // as a free cantilever though the seat's tongue carries it in compression: no credit for that support.
    var rearBolt = 0.0;
    for (b in sec.bolts) rearBolt = Math.max(rearBolt, b.y);
    var dims = HobberSaddle.cradleDimensions(), footLen = HobberSaddle.cradleRearY() - dims.wall / 2 - rearBolt, iFoot = width * Math.pow(dims.foot, 3) / 12;
    function axialElements(joint:Float) {
      var kWall = 3 * e * iX / Math.pow(height, 3), kJointRock = joint * sumY2 / Math.pow(height, 2), kFoot = e * iFoot / (height * height * footLen);
      return [{name: "wall bending", f: hz(kWall), k: kWall}, {name: "foot", f: hz(kFoot), k: kFoot}, {name: "floor joint", f: hz(kJointRock), k: kJointRock}];
    }
    function modesFor(joint:Float) {
      var kz = e * iX / (ahead * ahead * height), el = axialElements(joint);
      var kxWall = 3 * e * iY / Math.pow(height, 3), kxRoot = joint * sumX2 / Math.pow(height, 2);
      return [{name: "vertical (flange rocking on the wall)", f: hz(kz)}, {name: "axial (wall, foot and floor bolts)", f: hz(1 / (1 / el[0].k + 1 / el[1].k + 1 / el[2].k))},
        {name: "lateral (wall and floor bolts)", f: hz(1 / (1 / kxWall + 1 / kxRoot))}];
    }
    var modes = modesFor(kJoint), loose = modesFor(kJoint / 10), elements = axialElements(kJoint);
    var first = modes[0], firstLoose = loose[0];
    for (m in modes) if (m.f < first.f) first = m;
    for (m in loose) if (m.f < firstLoose.f) firstLoose = m;
    // Tooth-pass band.
    var gashes = machine.hob.parameterCount("gashes"), diameter = machine.hob.design.outsideDiameter;
    var nLow = 100000 / (Math.PI * diameter), fLow = nLow * gashes / 60, fHigh = nTop * gashes / 60;
    var lo = 0.8 * fLow, hi = 1.2 * fHigh;
    lines.push('B servo group: ${r(mass)} kg (motor, shaft, pulley, encoder and a third of the ${r(cradle)} kg cradle), centre ${r(ahead, 1)} mm ahead of the flange, ${r(height, 1)} mm above the floor; '
      + 'cradle section I about x ${Math.round(iX)} mm4, about y ${Math.round(iY)} mm4; floor bolts ${sec.bolts.length} M8 (joint ${Math.round(kJoint / 1000)} N/um each)');
    lines.push('B servo first-mode estimate: ' + [for (m in modes) m.name + " " + Math.round(m.f) + " Hz"].join("; ") + '; first ${Math.round(first.f)} Hz ' + first.name
      + ' against tooth-pass ${Math.round(fLow)}..${Math.round(fHigh)} Hz (hob Ø${r(diameter, 1)} at 100 m/min = ${Math.round(nLow)} rpm up to ${Math.round(nTop)} rpm, ${gashes} gashes): '
      + 'avoid ${Math.round(lo)}..${Math.round(hi)} Hz (+-20 %), ${first.f > hi ? Math.round(100 * first.f / hi - 100) + " % above" : first.f < lo ? "below" : "INSIDE"}; '
      + 'sensitivity: with the floor joint ten times softer (not preloaded) the first mode is ${Math.round(firstLoose.f)} Hz ${firstLoose.name}');
    if (first.f >= lo && first.f <= hi) throw "the servo's first mode lies within 20 % of the tooth-pass band: " + lines.join("; ");
    var margin = 1.3;
    lines.push('B servo first-mode margin: ${r(first.f / hi, 2)} x the top of the avoid band (${Math.round(hi)} Hz), required at least ${margin} x (${Math.round(margin * hi)} Hz); '
      + 'axial load-path elements, each alone with the servo group on it: ' + [for (el in elements) el.name + " " + Math.round(el.f) + " Hz"].join(", ")
      + ' (foot: ${r(footLen, 1)} mm from the rear bolt row to the wall mid-plane as a ${dims.foot} mm cantilever of the wall width)');
    var bad = [for (el in elements) if (el.f < margin * hi) el.name + " " + Math.round(el.f) + " Hz"];
    if (first.f < margin * hi || bad.length > 0) throw "the servo's first mode (" + Math.round(first.f) + " Hz) or a load-path element (" + bad.join(", ") + ") is less than 1.3 x the top of the tooth-pass band: " + lines.join("; ");
    // Cradle mass and the lightening.
    lines.push('Motor cradle: ${r(cradle, 3)} kg (was 2.036 kg with a 15 mm wall, 12 mm floor and solid rib, 1.285 kg before the foot); wall ${dims.wall}, floor ${dims.floor} (${dims.floorThin} ahead of y 195, foot ${dims.foot} under the wall), windowed rib, jack lug ${dims.lug} mm above the foot');
    // The servo's speed and torque over the hob's speed range (1:1 belt).
    var ratio = spindle.ratio, motorRatedRpm = rating.ratedSpeed * 60 / (2 * Math.PI), hobRpmTop = 3 * nLow, motorRpmTop = hobRpmTop * ratio;
    var cuttingBound = 100.0, efficiency = 0.97, reserve = 1.2;
    var cuttingTorque = cuttingBound * diameter / 2000, motorTorque = cuttingTorque / ratio / efficiency, motorPower = motorTorque * motorRpmTop * 2 * Math.PI / 60;
    lines.push('B speed and torque over the hob range (Ø${r(diameter, 1)}, 100..300 m/min = ${Math.round(nLow)}..${Math.round(hobRpmTop)} rpm): servo ${Math.round(nLow * ratio)}..${Math.round(motorRpmTop)} rpm against ${Math.round(motorRatedRpm)} rpm rated (${r(100 * motorRpmTop / motorRatedRpm, 1)} %), '
      + 'the cartridge limit ${Math.round(nTop)} rpm is above the rated speed (torque falls there: not for continuous cutting); cutting torque bound ${r(cuttingTorque, 2)} N m at the hob '
      + '(tangential force ${cuttingBound} N: ASSUMED upper bound, above the repo\'s load studies: 46.7 N peak on the reference part, 72.1 N on the m0.3 z40 1 mm smoke; at 16 mm radius those are 0.75 and 1.15 N m) = ${r(motorTorque, 2)} N m at the servo through the belt '
      + '(${efficiency} efficiency, ratio ${r(ratio, 2)}); servo rated ${r(rating.ratedTorque, 2)} N m (${r(rating.ratedTorque / motorTorque, 2)} x, ${reserve} x required), peak ${r(rating.peakTorque, 2)} N m; power ${Math.round(motorPower)} W against ${Math.round(rating.ratedTorque * rating.ratedSpeed)} W');
    if (motorRpmTop > motorRatedRpm) throw "the servo would run above its rated speed at the top of the hob speed range: " + lines.join("; ");
    if (motorTorque * reserve > rating.ratedTorque || motorPower > rating.ratedTorque * rating.ratedSpeed) throw "the servo's rated torque or power does not cover the hob's cutting load: " + lines.join("; ");
    // The flange bolts hold the servo against the belt (slots in the wall): friction of four M6 8.8 at 60 % of proof (assumed), steel on steel.
    var preload = 0.6 * 580 * 20.1, grip = 4 * preload * 0.12, pull = 2 * t0;
    lines.push('B servo flange grip: 4 M6 8.8 at 60 % of proof (${Math.round(preload)} N each, assumed), friction 0.12 steel on steel (assumed) holds ${Math.round(grip)} N along the slots against the belt\'s net pull ${Math.round(pull)} N (2 x pretension): ${r(grip / pull, 1)} x; '
      + 'the jack screw also bears on the flange from below against that pull');
    if (grip < 3 * pull) throw "the flange bolts' friction does not hold the belt pull with a factor of 3: " + lines.join("; ");
    return lines;
  }

  /** The members that move with the Z joint (the Z saddle and everything carried on it), and their mass and centre of mass in the work frame. */
  static function zMoving(machine:FlexsplineHobber):{mass:Float, x:Float, y:Float, z:Float, ids:Array<String>} {
    var def = machine.definition(), poses = machine.solvedPoses(), back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    var moving = new Map<String, Bool>();
    moving.set("blockZLeft", true);
    var changed = true;
    while (changed) {
      changed = false;
      for (j in def.joints) if (moving.exists(j.parent) && !moving.exists(j.child)) {
        moving.set(j.child, true);
        changed = true;
      }
    }
    var mass = 0.0, mx = 0.0, my = 0.0, mz = 0.0, ids = [];
    for (m in machine.components()) if (moving.exists(m.id)) {
      var props = m.component.massProperties(), at = F.transformPoint(F.compose(back, poses.get(m.id)), props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      mass += props.mass;
      mx += props.mass * at.x;
      my += props.mass * at.y;
      mz += props.mass * at.z;
      ids.push(m.id);
    }
    return {mass: mass, x: mx / mass, y: my / mass, z: mz / mass, ids: ids};
  }

  /**
   * The Z screw stands off the rail pair's middle (`HobberZOffset`), so its thrust and the moving mass's weight, which acts at the mass's
   * centre of mass, are `e` apart across the machine and form a moment about the rail normal (x), the blocks' yaw. The sign of the offset must be
   * the side of the centre of mass. The moment is shared by the four blocks and set against the block row's rated yaw moment (static: the
   * weight, held by the brake; dynamic: the sizing's peak thrust, the rated moment scaled by C/C0 because the catalogue lists static moments
   * only). Returns the lines; throws when the sign is wrong or a block is over its rating.
   */
  public static function zOffsetMoments(machine:FlexsplineHobber):Array<String> {
    var z = zMoving(machine), g = 9.80665, offset = HobberZLayout.axisY(), result = machinekit.hobbing.HobberZOffset.result();
    if (offset * (z.y - 0) <= 0) throw 'Z screw at y ${offset} is on the opposite side of the moving mass centre (y ${z.y})';
    var report:Dynamic = FlexsplineHobberLayoutReport.measure(machine), mass = z.mass, q = 0.005 / (2 * Math.PI);
    var nut:machinekit.motion.BallNut = cast machine.components().filter(m -> m.id == "Znut")[0].component;
    var motor:machinekit.motion.ServoMotor = cast machine.components().filter(m -> m.id == "Zmotor")[0].component;
    var demand = machinekit.sizing.LinearDriveSizing.demand({
      massKg: mass, motors: 1, metresPerRadian: q, efficiency: nut.efficiency(), rotaryInertiaKgM2: 0.4e-4, acceleration: 0.5, velocity: 0.2, strokeM: 0.2,
      cuttingForceN: 100, dragForceN: 100, gravityForceN: mass * g, rotaryDragNm: 0.1, reserve: 1.5
    }, motor.rating);
    var peakThrust = demand.peakNm * nut.efficiency() / q, weight = mass * g;
    var arm = Math.abs(offset - z.y) / 1000, mirrored = Math.abs(-offset - z.y) / 1000;
    var block:machinekit.motion.LinearRailBlock = cast machine.components().filter(m -> m.id == "blockZLeft")[0].component;
    var rating:Null<machinekit.motion.LinearRailProfile.LinearRailRating> = null;
    for (item in block.spec.ratings) if (item.yawMoment != null && rating == null) rating = item;
    if (rating == null) throw "Z block row " + block.spec.designation + " has no rated yaw moment";
    var blocks = 4, staticMoment = weight * arm / blocks, dynamicMoment = peakThrust * arm / blocks;
    var yawBase:Float = rating.yawMoment ?? 0.0, yawStatic:Float = yawBase, yawDynamic:Float = yawBase * rating.dynamicLoad / rating.staticLoad;
    function r(v:Float, d:Int = 2):Float return Math.round(v * Math.pow(10, d)) / Math.pow(10, d);
    var lines = [
      'Z screw offset: y ${offset} (minimum ${result.minimum} mm set by ${result.governing}; casting allows at most ${result.maximum} mm); Z moving mass ${r(mass)} kg, centre of mass y ${r(z.y, 1)}',
      'Z offset moments on ${blocks} ${block.spec.designation} blocks (${rating.source}): arm ${r(arm * 1000, 1)} mm (on the other side it would be ${r(mirrored * 1000, 1)} mm); ' +
      'static weight ${r(weight)} N: ${r(staticMoment, 3)} N m per block against yaw ${yawStatic} N m (safety factor ${r(yawStatic / Math.max(staticMoment, 1e-9), 1)}); ' +
      'peak thrust ${r(peakThrust)} N: ${r(dynamicMoment, 3)} N m per block against ${r(yawDynamic, 1)} N m (dynamic, scaled by C/C0; safety factor ${r(yawDynamic / Math.max(dynamicMoment, 1e-9), 1)}); roll about the rail axis: none (thrust and weight are along it)'
    ];
    if (staticMoment > yawStatic || dynamicMoment > yawDynamic) throw "Z blocks exceed their yaw rating: " + lines.join("; ");
    return lines;
  }

  /**
   * The spindle seat against the cartridge (FRAME_PLAN R2): in the housing's own z (tool face at 180, encoder stator at 0..-14), the seat's
   * bore must lie inside the housing's clamping length (`SpindleHousing.CLAMP_FROM..CLAMP_TO`), cover the angular-contact pair's outer rings
   * (129..163), and carry its clamp screws inside that length. Returns the line; throws otherwise.
   */
  public static function spindleSeat(machine:FlexsplineHobber):Array<String> {
    var poses = machine.solvedPoses(), housing = poses.get("spindle/housing"), back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    var seat = HobberSaddle.spindleSeatRange(), toHousing = F.inverse(F.compose(back, housing));
    function zOf(y:Float):Float return F.transformPoint(toHousing, 0, y, HobberZLayout.SEAT_Z).z;
    var a = zOf(seat.y0), b = zOf(seat.y1), from = Math.min(a, b), to = Math.max(a, b);
    var screws = [for (y in seat.clampYs) zOf(y)];
    var lines = ['Spindle seat grips housing z ${Math.round(from * 10) / 10}..${Math.round(to * 10) / 10} (${Math.round((to - from) * 10) / 10} mm) of the clamping length ${SpindleHousing.CLAMP_FROM}..${SpindleHousing.CLAMP_TO} (assumed); ' +
      'bearing pair 129..163; clamp screws at z ${screws.join(", ")}; rear face and the encoder stator (z 0..-14) and the nose and gland flange 170..186 are not gripped'];
    if (from < SpindleHousing.CLAMP_FROM - 1e-6 || to > SpindleHousing.CLAMP_TO + 1e-6) throw "spindle seat leaves the housing's clamping length: " + lines.join("; ");
    if (from > 129 || to < 163) throw "spindle seat does not cover the bearing pair: " + lines.join("; ");
    for (z in screws) if (z < from || z > to) throw "spindle seat clamp screw outside the seat: " + lines.join("; ");
    return lines;
  }

  /**
   * The relief in the bed casting under the spindle servo (`HobberSpindleRelief`). Checks (1) the relief's pose chain against the solved
   * machine at three joint states (servo frame within 1e-6 mm and radians), (2) the servo, with its connector envelopes, keeps the running
   * gap from the finished casting at the extreme poses including the overtravel on every axis, and (3) the casting keeps `WALL` of solid
   * material under the pocket's floor. Returns the report lines; throws with the numbers otherwise.
   */
  public static function spindleRelief(machine:FlexsplineHobber):Array<String> {
    var relief = HobberSpindleRelief.result(), lines = [];
    function r(v:Float, d:Int = 2):Float return Math.round(v * Math.pow(10, d)) / Math.pow(10, d);
    var model = new cadkit.modeling.AssemblyModel("mm");
    machine.addTo(model, "");
    var state = model.initialState();
    var toMachine = FlexsplineHobber.MACHINE_FROM_WORK, zero = HobberSpindleRelief.motorZeroPose(HobSpindle.MOTOR_CLOCKING);
    function analytic(x:Float, y:Float, z:Float, a:Float) return F.compose(toMachine, F.compose(HobberSpindleRelief.motion(x, y, z, a), zero));
    // (1) the chain.
    var xl = machine.travel.filter(t -> t.id == "X")[0].lower, xu = machine.travel.filter(t -> t.id == "X")[0].upper;
    for (s in [{x: xl, y: 0.0, z: 0.0, a: 0.0}, {x: xl, y: -10.0, z: -10.0, a: 15 * Math.PI / 180}, {x: (xl + xu) / 2, y: 7.0, z: 120.0, a: -9 * Math.PI / 180}]) {
      state.setJoint("X", s.x);
      state.setJoint("Y", s.y);
      state.setJoint("Z", s.z);
      state.setJoint("swivel/A", s.a);
      state.forwardKinematics();
      var actual = state.worldPose("spindle/motor"), expected = analytic(s.x, s.y, s.z, s.a), worst = 0.0;
      worst = Math.max(worst, Math.max(Math.abs(actual.x - expected.x), Math.max(Math.abs(actual.y - expected.y), Math.abs(actual.z - expected.z))));
      var ma = F.toRotationMatrix(actual), me = F.toRotationMatrix(expected);
      for (i in 0...9) worst = Math.max(worst, Math.abs(ma[i] - me[i]));
      if (worst > 1e-6) throw 'spindle relief pose chain differs from the machine by ${worst} at X ${s.x}, Y ${s.y}, Z ${s.z}, A ${r(s.a * 180 / Math.PI, 1)} deg';
    }
    for (t in machine.travel) if (t.id == "X" && t.lower < HobberScales.stroke("X").lower - 1e-9) throw 'X lower limit ${t.lower} is below the design stroke ${HobberScales.stroke("X").lower} the relief is derived for';
    state.setJoint("X", xl);
    state.forwardKinematics();
    var cache = new machinekit.assembly.PosedParts(), members = machine.components();
    var castingPose = state.worldPose("bed/casting");
    var expectedCasting = F.compose(toMachine, FoundationLayout.worldPose("bed"));
    var cp = F.toRotationMatrix(castingPose), ce = F.toRotationMatrix(expectedCasting);
    var offset = Math.max(Math.abs(castingPose.x - expectedCasting.x), Math.max(Math.abs(castingPose.y - expectedCasting.y), Math.abs(castingPose.z - expectedCasting.z)));
    for (i in 0...9) offset = Math.max(offset, Math.abs(cp[i] - ce[i]));
    if (offset > 1e-6) throw 'the bed casting is not at the bed frame the relief assumes (off by ${offset})';
    var casting = cache.posed(members.filter(m -> m.id == "bed/casting")[0].component, castingPose);
    var motor = members.filter(m -> m.id == "spindle/motor")[0].component;
    // (2) the gap at the extremes with the overtravel.
    var ot = HobberXLayout.OVERTRAVEL, worstGap = 1e9, worstAt = "";
    var x = HobberScales.stroke("X"), y = HobberScales.stroke("Y"), z = HobberScales.stroke("Z");
    for (xv in [x.lower - ot, x.upper + ot]) for (yv in [y.lower - ot, y.upper + ot]) for (zv in [z.lower - ot, z.lower, 0.0]) for (a in [-HobberSwivelLayout.RANGE, 0.0, HobberSwivelLayout.RANGE]) {
      var posed = cache.posed(motor, analytic(xv, yv, zv, a)), gap = posed.distance(casting);
      posed.close();
      if (gap < worstGap) {
        worstGap = gap;
        worstAt = 'X ${xv}, Y ${yv}, Z ${zv}, A ${r(a * 180 / Math.PI, 1)} deg';
      }
    }
    if (worstGap < HobberSpindleRelief.RUNNING_GAP - 1e-6)
      throw 'spindle servo comes ${r(worstGap, 3)} mm from the bed casting at ${worstAt} (running gap ${HobberSpindleRelief.RUNNING_GAP} mm) with the relief in place';
    // (3) the wall under the floor, in the casting's frame (the casting is posed at the bed frame). With the servo standing above the
    // cartridge it never reaches the casting and there is no relief.
    if (relief == null) {
      cache.close();
      lines.push('Spindle servo (${HobSpindle.MOTOR}, connectors to LH assumed positions) stands above the cartridge and never comes within the ${HobberSpindleRelief.RUNNING_GAP} mm running gap of the bed casting: '
        + 'no relief in the casting; least gap to the finished casting over the extremes with overtravel ${r(worstGap, 2)} mm at ${worstAt}');
      return lines;
    }
    var wall = HobberCastingPockets.WALL;
    var d = FoundationLayout.dimensions("bed");
    var slab = cadkit.modeling.Part.box(relief.x1 - relief.x0, relief.y1 - relief.y0, wall, cadkit.modeling.Align.Min, cadkit.modeling.Align.Min, cadkit.modeling.Align.Min)
      .translated(new cadkit.modeling.Vector(relief.x0, relief.y0, relief.z0 - wall));
    var slabPosed = machinekit.assembly.PosedParts.moved(slab, castingPose);
    slab.close();
    var solid = slabPosed.intersect(casting), missing = (relief.x1 - relief.x0) * (relief.y1 - relief.y0) * wall - solid.volume();
    solid.close();
    slabPosed.close();
    cache.close();
    if (missing > 1.0) throw 'the bed casting is not solid for ${wall} mm under the spindle relief floor (z ${relief.z0}): ${r(missing)} mm3 missing in the footprint x ${relief.x0}..${relief.x1}, y ${relief.y0}..${relief.y1}';
    lines.push('Spindle servo relief in the bed casting (${HobSpindle.MOTOR}, connectors to LH assumed positions): pocket x ${relief.x0}..${relief.x1}, y ${relief.y0}..${relief.y1} in the casting frame '
      + '(${r(relief.width, 1)} x ${r(relief.depth, 1)} mm), floor z ${relief.z0} (${r(d[2] - relief.z0, 1)} mm below the top), '
      + 'box volume ${r(relief.volume / 1000, 1)} cm3, opens through ${relief.faces.join(", ")}; set by ${relief.governing} (${relief.poses} poses); ' +
      'least gap to the finished casting over the extremes with overtravel ${r(worstGap, 2)} mm at ${worstAt}; ${wall} mm of material under the floor');
    return lines;
  }

  /** Procedures, drive sizing and strict integrity of the assembled machine; returns the number of checks. */
  public static function run(machine:FlexsplineHobber, sweep:Bool = false):Int {
    var presentation = materia.presentation.Presentations.assembled(machine.definition(), -1.1, 0.35, "hobber");
    presentation.procedures = HobberAssemblyProcedures.build(machine);
    materia.presentation.PresentationCodec.validate(machine.definition(), presentation);
    var lines = sizing(machine).concat(scales(machine)).concat(swivelLoad(machine)).concat(zOffsetMoments(machine)).concat(spindleSeat(machine)).concat(spindleDrive(machine)).concat(spindleRelief(machine)), errors = newErrors(machine, sweep);
    for (line in lines) Sys.println(line);
    if (errors.length > 0) throw 'hobber strict integrity: ${errors.length} findings\n  ' + errors.join("\n  ");
    return presentation.procedures.length + lines.length + 1;
  }

  /** The machine survives encode and decode with its member recipes and collision hulls. */
  public static function roundTrip(machine:FlexsplineHobber):Void {
    var rebuilt = MachineAssembly.decode(machine.encode(), MachineKitComponents.standard());
    if (rebuilt.components().length != machine.components().length) throw "the machine lost members in persistence";
  }

  /**
   * Drive sizing on the assembled machine at 400 W: the layout report's continuous thrust margin (rated torque, 20 % reserve,
   * 0.5 m/s2, 100 N process + 100 N friction, gravity on Z) and the full duty envelope of `LinearDriveSizing` (peak and RMS
   * torque with a 1.5 reserve, speed, inertia ratio). Returns one line per axis; throws when an axis fails.
   */
  public static function sizing(machine:FlexsplineHobber):Array<String> {
    var report:Dynamic = FlexsplineHobberLayoutReport.measure(machine), lines:Array<String> = [];
    for (entry in (report.drives : Array<Dynamic>)) {
      var id:String = entry.axis, members = machine.components();
      var motor:machinekit.motion.ServoMotor = cast members.filter(m -> m.id == id + "motor")[0].component;
      var nut:machinekit.motion.BallNut = cast members.filter(m -> m.id == id + "nut")[0].component;
      var lead = 0.005, q = lead / (2 * Math.PI), mass:Float = entry.movingMassKg;
      var demand = machinekit.sizing.LinearDriveSizing.demand({
        massKg: mass, motors: 1, metresPerRadian: q, efficiency: nut.efficiency(), rotaryInertiaKgM2: 0.4e-4,
        acceleration: 0.5, velocity: 0.2, strokeM: id == "Y" ? 0.03 : id == "X" ? 0.095 : 0.2,
        cuttingForceN: 100, dragForceN: 100, gravityForceN: id == "Z" ? mass * 9.80665 : id == "Y" ? mass * 9.80665 * Math.sin(Math.PI / 12) : 0,
        rotaryDragNm: 0.1, reserve: 1.5
      }, motor.rating);
      var line = '$id ${motor.rating.designation}: moving ${Math.round(mass * 100) / 100} kg, thrust margin ${Math.round(entry.thrustMarginN)} N, '
        + 'peak ${Math.round(demand.peakNm * 1000) / 1000} N m (peak rating ${motor.rating.peakTorque}), rms ${Math.round(demand.rmsNm * 1000) / 1000} N m '
        + '(rated ${motor.rating.ratedTorque}), inertia ratio ${Math.round(demand.inertiaRatio * 100) / 100}, failures: ${demand.failures.join("; ")}';
      lines.push(line);
      if (demand.failures.length > 0) throw "drive sizing fails: " + line;
      if (id == "Z") {
        // The braked servo holds the axis with the power off: the brake is bound to the motor's drive and holds the screw torque of the weight.
        var brake = machine.actuatorFor("Zdrive").brake, load = mass * 9.80665 * q;
        if (brake == null) throw "Z drive has no brake bound";
        if (brake.holdingEffort < 1.5 * load) throw 'Z brake holds ${brake.holdingEffort} N m against ${Math.round(load * 1000) / 1000} N m of weight';
        lines.push('Z brake: holds ${Math.round(brake.holdingEffort * 100) / 100} N m (assumed) against ${Math.round(load * 1000) / 1000} N m of weight, release ${brake.releaseChannel} read back on ${brake.releasedSensor}');
      }
    }
    return lines;
  }

  public static function main():Void {
    var args = Sys.args(), began = Sys.time();
    if (args.indexOf("spindlepose") >= 0) {
      var names = [], spindles:Array<HobSpindle> = [];
      for (k in 0...4) {
        names.push("k" + k);
        spindles.push(new HobSpindle(null, null, k));
      }
      names.push("default");
      spindles.push(new HobSpindle());
      for (i in 0...names.length) {
        var poses = spindles[i].solvedPoses();
        var m:materia.assembly.AssemblyRecord.AssemblyFrame = poses.get("motor");
        Sys.println(names[i] + " motor " + [m.x, m.y, m.z] + " rot " + F.toRotationMatrix(m).map(v -> Math.round(v * 1000) / 1000));
      }
      var spin = HobberSpindleRelief.motorZeroPose(HobSpindle.MOTOR_CLOCKING);
      Sys.println("zero pose " + [spin.x, spin.y, spin.z] + " rot " + F.toRotationMatrix(spin).map(v -> Math.round(v * 1000) / 1000));
      if (args.length == 1) return;
    }
    if (args.indexOf("clocking") >= 0) {
      for (k in 0...4) {
        var r = HobberSpindleRelief.reliefFor(k);
        Sys.println(r == null ? 'clocking $k: no relief' : 'clocking $k: x ${r.x0}..${r.x1} y ${r.y0}..${r.y1} floor ${r.z0} (box ${Math.round(r.volume / 1000)} cm3) faces ${r.faces.join(",")} governed by ${r.governing}');
      }
      if (args.length == 1) return;
    }
    var machine = new FlexsplineHobber();
    Sys.println('machine built: ${machine.components().length} members in ${Math.round(Sys.time() - began)} s (${Math.round(Sys.cpuTime())} s CPU)');
    if (args.indexOf("procedures") >= 0) {
      var presentation = materia.presentation.Presentations.assembled(machine.definition(), -1.1, 0.35, "hobber");
      presentation.procedures = HobberAssemblyProcedures.build(machine);
      materia.presentation.PresentationCodec.validate(machine.definition(), presentation);
      Sys.println('procedures valid: ${[for (p in presentation.procedures) p.id].join(", ")}');
    }
    if (args.indexOf("sizing") >= 0) for (line in sizing(machine)) Sys.println(line);
    if (args.indexOf("scales") >= 0) for (line in scales(machine)) Sys.println(line);
    if (args.indexOf("swivel") >= 0) for (line in swivelLoad(machine)) Sys.println(line);
    if (args.indexOf("zoffset") >= 0) for (line in zOffsetMoments(machine)) Sys.println(line);
    if (args.indexOf("spindleseat") >= 0) for (line in spindleSeat(machine)) Sys.println(line);
    if (args.indexOf("roundtrip") >= 0) {
      roundTrip(machine);
      Sys.println("round trip ok");
    }
    if (args.indexOf("spindledrive") >= 0) for (line in spindleDrive(machine)) Sys.println(line);
    if (args.indexOf("spindlerelief") >= 0) for (line in spindleRelief(machine)) Sys.println(line);
    if (args.indexOf("fasteners") >= 0) for (line in HobberFastenerChecks.report(machine)) Sys.println(line);
    if (args.indexOf("integrity") >= 0) {
      var errors = integrity(machine, args.indexOf("sweep") >= 0);
      var counts = new Map<String, Int>();
      for (error in errors) {
        var code = error.split(" ")[0];
        counts.set(code, (counts.exists(code) ? counts.get(code) : 0) + 1);
      }
      for (code => count in counts) Sys.println('$code: $count');
      for (error in errors) Sys.println("  " + error);
      Sys.println('integrity: ${errors.length} errors in ${Math.round(Sys.time() - began)} s');
      if (errors.length > 0) Sys.exit(1);
    }
  }
}
