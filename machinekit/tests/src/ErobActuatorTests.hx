import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.component.MachineKitComponents;
import machinekit.motion.ErobActuator;
import machinekit.motion.ErobActuatorCad;
import machinekit.motion.ErobActuatorData as D;
import machinekit.motion.ErobActuatorPart;
import machinekit.motion.PowerSupply;
import machinekit.motion.PrecisionErrors;
import machinekit.power.DcCurrent;
import machinekit.power.DcVoltage;

/** eRob 110 V6 rev D: drawing dimensions, bolt patterns, ratio rows, joint limits and effort, encoder, 48 V bus. */
class ErobActuatorTests {
  static function check(condition:Bool, message:String):Void if (!condition) throw 'eRob: $message';
  static function near(a:Null<Float>, b:Float, message:String, tolerance:Float = 1e-6):Void check(
    a != null && Math.isFinite(a) && Math.abs(a - b) <= tolerance,
    '$message: $a != $b'
  );
  static function fails(run:() -> Void, message:String):Void {
    var failed = false;
    try run() catch (_:Dynamic) failed = true;
    check(failed, message);
  }

  public static function main():Void {
    MachineKitComponents.standard();
    ratioRows();
    dimensions();
    boltPatterns();
    cutouts();
    jointAndEffort();
    encoderAndErrors();
    electrical();
    unpublished();
    hostFaceAndDrive();
    recipeRoundTrip();
    cadFallback();
    Sys.println("eRob: ratio rows, dimensions, bolt patterns, cut-outs, joint, encoder, 48 V bus and recipes passed");
  }

  static function ratioRows():Void {
    var expected:Array<Array<Float>> = [[50, 127, 72, 51, 242, 60], [80, 178, 113, 82, 332, 37.5], [100, 204, 140, 87, 369, 30],
      [120, 217, 140, 87, 395, 25], [160, 229, 140, 87, 408, 18.75]];
    check(D.ratios().length == 5, "five ratios");
    for (e in expected) {
      var r = D.row(Std.int(e[0]));
      near(r.peakTorque, e[1], "peak start-stop");
      near(r.averageTorque, e[2], "average load");
      near(r.ratedTorque, e[3], "rated");
      near(r.momentaryTorque, e[4], "momentary");
      near(r.speedRpm, e[5], "speed");
      // Rated motor speed through the ratio is the table's output speed.
      near(D.MOTOR_RATED_RPM / r.ratio, r.speedRpm, "3000 rpm through the ratio");
    }
    fails(() -> D.row(101), "unknown ratio rejected");
    near(D.encoderGrade(20).repeatability, 7, "20-bit repeatability");
    near(D.encoderGrade(20).absolute, 15, "20-bit absolute");
    near(D.encoderGrade(19).repeatability, 10, "19-bit repeatability");
    near(D.encoderGrade(19).absolute, 25, "19-bit absolute");
    fails(() -> D.encoderGrade(21), "unknown encoder grade rejected");
    // Motor: 723 W at 2.3 N m and 3000 rpm.
    near(D.MOTOR_RATED_TORQUE * D.MOTOR_RATED_RPM * 2 * Math.PI / 60, D.RATED_POWER, "rated power", 1);
  }

  static function dimensions():Void {
    var plain = new ErobActuatorPart(new ComponentValues().setBoolean("brake", false));
    var braked = new ErobActuatorPart();
    var out = new ErobActuatorPart(new ComponentValues().setToken("variant", ErobActuatorPart.OUTPUT).setBoolean("brake", false));
    var housing = plain.geometry(), pilot = out.geometry(), envelope = plain.geometry(Envelope);
    var hb = housing.shape.bounds(), ob = pilot.shape.bounds();
    near(hb.get_max().get_x(), 55, "body radius");
    near(hb.get_max().get_z(), ErobActuatorPart.SEAT_Z, "housing front face");
    near(hb.get_min().get_z(), -D.BODY_LENGTH, "rear boss end");
    near(ob.get_max().get_x(), 42.5, "pilot radius");
    near(ob.get_max().get_z(), 0, "output face");
    near(ob.get_min().get_z(), -D.PILOT_HEIGHT, "pilot height");
    check(housing.solidCount() == 1 && pilot.solidCount() == 1, "one solid each");
    check(housing.volume() < envelope.volume(), "preview inside envelope");
    // The hollow bore is Ø18 through both.
    near(plain.geometry(Envelope).volume(), Math.PI * (55 * 55 - 9 * 9) * (D.BODY_LENGTH - D.REAR_SPIGOT_HEIGHT + ErobActuatorPart.SEAT_Z)
      + Math.PI * (42.5 * 42.5 - 81) * D.REAR_SPIGOT_HEIGHT, "envelope volume", 1);
    near(D.BODY_LENGTH, 80.2, "body length");
    near(D.OVERALL_LENGTH, 83.6, "length with connectors");
    // Mass: 2.57 kg, 2.68 kg with brake, split across the joint without counting twice.
    var plainOut = out.massProperties().mass + plain.massProperties().mass;
    near(plainOut, 2.57, "unbraked mass", 1e-9);
    near(braked.massProperties().mass + new ErobActuatorPart(new ComponentValues().setToken("variant",
      ErobActuatorPart.OUTPUT)).massProperties().mass, 2.68, "braked mass", 1e-9);
    near(new ErobActuator(100, 20, false).massProperties().mass, 2.57, "assembly mass", 1e-9);
    housing.close();
    pilot.close();
    envelope.close();
  }

  static function boltPatterns():Void {
    var housing = new ErobActuatorPart(), out = new ErobActuatorPart(new ComponentValues().setToken("variant", ErobActuatorPart.OUTPUT));
    var fixed = [for (c in housing.connectors()) if (StringTools.startsWith(c.name, "fixedBolt")) c];
    var output = [for (c in out.connectors()) if (StringTools.startsWith(c.name, "outputBolt")) c];
    check(fixed.length == 16 && output.length == 12, "16 fixed and 12 output bolts");
    for (i in 0...16) {
      var p = ErobActuatorPart.fixedBolt(i);
      near(Math.sqrt(p.x * p.x + p.y * p.y), 51, "fixed bolt radius (Ø102)");
      var q = ErobActuatorPart.fixedBolt((i + 1) % 16);
      near(Math.abs(Math.atan2(p.x * q.y - p.y * q.x, p.x * q.x + p.y * q.y)) * 180 / Math.PI, 22.5, "fixed spacing");
    }
    for (i in 0...12) {
      var p = ErobActuatorPart.outputBolt(i);
      near(Math.sqrt(p.x * p.x + p.y * p.y), 38.5, "output bolt radius (Ø77)");
      var q = ErobActuatorPart.outputBolt((i + 1) % 12);
      near(Math.abs(Math.atan2(p.x * q.y - p.y * q.x, p.x * q.x + p.y * q.y)) * 180 / Math.PI, 30, "output spacing");
    }
    near(housing.connector("seat").frame.z, ErobActuatorPart.SEAT_Z, "seat connector z");
    near(out.connector("face").frame.z, 0, "output face connector z");
    near(housing.connector("fixedBolt1").frame.x, 51, "bolt 1 on +X");
  }

  static function cutouts():Void {
    var housing = new ErobActuatorPart(), out = new ErobActuatorPart(new ComponentValues().setToken("variant", ErobActuatorPart.OUTPUT));
    check(housing.toolSpecs().length == 1 && housing.toolSpecs()[0].name == "mountingCutout", "housing exposes mountingCutout");
    check(out.toolSpecs().length == 1 && out.toolSpecs()[0].name == "mountingCutout", "output exposes mountingCutout");
    var seat = housing.mountingCutout(), b = seat.shape.bounds();
    check(seat.solidCount() == 1, "seat tool is one solid");
    near(b.get_min().get_z() + 0.1, ErobActuatorPart.SEAT_Z - D.POCKET_DEPTH_MIN, "pocket is 8 deep behind the front face");
    near(b.get_max().get_z() - 0.1, -1, "bore stops 1 mm below the output face (running gap)");
    check(b.get_max().get_x() > 55 && b.get_max().get_x() < 55.5, "pocket Ø110 plus clearance");
    fails(() -> housing.mountingCutout(7), "a pocket under 8 mm is rejected");
    var viaSpec = housing.tool("mountingCutout", housing.toolSpecs()[0].defaults());
    near(viaSpec.volume(), seat.volume(), "tool from its spec", 1e-6);
    var plate = out.mountingCutout(), expected = 12 * Math.PI * 2.3 * 2.3 * 10.2 + Math.PI * 9.2 * 9.2 * 10.2;
    near(plate.volume(), expected, "output holes and bore", 1e-3);
    seat.close();
    viaSpec.close();
    plate.close();
  }

  static function jointAndEffort():Void {
    for (ratio in [50, 80, 100, 120, 160]) {
      var a = new ErobActuator(ratio, 20, true);
      var row = D.row(ratio), joints = a.definition().joints;
      check(joints.length == 1 && joints[0].id == "A", "one joint A");
      check(joints[0].type == Continuous, "continuous revolute");
      near(joints[0].limits.effort, row.peakTorque, 'joint effort at $ratio:1');
      near(joints[0].limits.velocity, row.speedRpm * 2 * Math.PI / 60, 'joint speed at $ratio:1');
      var act = a.actuator("A-drive");
      near(act.maxEffort, row.peakTorque, "actuator effort");
      near(act.ratedTorque, row.ratedTorque, "actuator rated torque");
      near(act.peakTorque, row.peakTorque, "actuator peak torque");
      near(act.ratedSpeed, row.speedRpm * 2 * Math.PI / 60, "actuator speed");
      near(act.rotorInertia, 316600e-9 * ratio * ratio, "rotor inertia through the ratio");
      near(act.encoderCounts, 1048576, "20-bit counts");
      var curve = act.torqueSpeed, assumed = act.assumed;
      check(act.drive == "servo" && curve != null && curve.length == 4, "servo curve");
      check(assumed != null && assumed.length == 1, "curve interpolation is declared assumed");
      check(act.holdingTorque == null, "no holding torque is invented");
    }
    var a = new ErobActuator(100, 19, false);
    near(a.actuator("x").rotorInertia, 308632e-9 * 1e4, "unbraked inertia");
    fails(() -> a.actuator("x", 24), "24 V bus rejected");
    fails(() -> new ErobActuator(90), "unknown ratio rejected");
    var restored = MachineAssembly.decode(a.encode());
    check(restored.definition().joints[0].id == "A", "joint survives encode/decode");
    check(restored.components().length == 2, "both sides survive encode/decode");
  }

  static function encoderAndErrors():Void {
    var a = new ErobActuator(100, 20, true);
    var export = new cadkit.modeling.AssemblyModel();
    a.addTo(export, "");
    var encoders = export.definition().encoders;
    if (encoders == null || encoders.length != 1) throw "eRob: encoder recorded on the joint: " + (encoders == null ? "null" : Std.string(encoders.length));
    check(encoders[0].joint == "A" && encoders[0].kind == "absolute", "absolute output encoder on A");
    near(encoders[0].counts, 1048576, "20-bit output encoder");
    var errors = PrecisionErrors.of(a.housing).all();
    check(errors.length == 2 && errors[0].bound == 7 && errors[0].unit == "arcsec", "repeatability ±7 arcsec");
    check(errors[1].bound == 15, "absolute ±15 arcsec");
    var e19 = new ErobActuatorPart(new ComponentValues().setToken("encoderBits", "19"));
    near(e19.encoder("e", "A").counts, 524288, "19-bit counts");
    check(PrecisionErrors.of(e19).all()[1].bound == 25, "19-bit absolute ±25 arcsec");
    var metadata = machinekit.component.EngineeringMetadata.of(a.housing);
    check(metadata != null && metadata.ratingsBasis == Catalog, "ratings are Catalog");
  }

  static function electrical():Void {
    var a = new ErobActuator();
    check(a.hasPort("power") && a.hasPort("bus"), "power and bus ports exposed");
    var need = a.housing.busRequirement("power");
    near(need.nominalVolts, 48, "48 V");
    near(need.minimumVolts, 43.2, "-10 %");
    near(need.maximumVolts, 52.8, "+10 %");
    near(need.ratedPower, 723, "rated power");
    check(need.regenerates, "regenerates to the bus");
    a.housing.checkSupply(new PowerSupply(new DcVoltage(48), new DcCurrent(20)).outputRating("power1"));
    fails(() -> a.housing.checkSupply(new PowerSupply(new DcVoltage(24), new DcCurrent(40)).outputRating("power1")), "24 V supply rejected");
    fails(() -> a.housing.checkSupply(new PowerSupply(new DcVoltage(48), new DcCurrent(5)).outputRating("power1")), "240 W supply rejected");
    fails(() -> new ErobActuatorPart(new ComponentValues().setToken("variant", ErobActuatorPart.OUTPUT)).busRequirement("power"), "output side has no bus");
    check(new ErobActuatorPart(new ComponentValues().setToken("bus", "CANopen")).port("bus").name == "bus", "CANopen variant has the bus port");
  }

  /** The flange as a host sees it, the output-referred winding, the integrated amplifier, the optional stop and acceleration, the brake once entered. */
  static function hostFaceAndDrive():Void {
    var housing = new ErobActuatorPart(), output = new ErobActuatorPart(new ComponentValues().setToken("variant", ErobActuatorPart.OUTPUT));
    var face = housing.mountingFace(), outFace = output.mountingFace();
    if (face == null || outFace == null) throw "eRob: both sides present a mounting face";
    var cut = face.tool(8);
    check(cut.volume() > 0, "housing face carries holes");
    cut.close();
    var rating = housing.electricalRating();
    if (rating == null) throw "eRob: housing has a winding rating";
    near(rating.torqueConstant, D.TORQUE_CONSTANT * 100, "torque constant through the ratio");
    check(output.electricalRating() == null && output.motorDriver() == null, "output side has no winding or amplifier");
    var amplifier = housing.motorDriver();
    check(amplifier != null, "integrated amplifier");
    check(housing.motorBrakeSpec() == null, "no brake torque, no brake spec");
    var held = new ErobActuatorPart(new ComponentValues().setNumber("brakeHoldingTorque", 40));
    var spec = held.motorBrakeSpec();
    if (spec == null) throw "eRob: brake spec once a torque is entered";
    near(spec.holdingTorque(), 40, "entered holding torque");
    var stopped = new ErobActuator(100, 20, true, "A", null, 0.2, 1.5);
    var joint = [for (j in stopped.definition().joints) if (j.id == "A") j][0];
    near(joint.limits.upper, 0.2, "stop");
    near(joint.limits.acceleration, 1.5, "acceleration");
  }

  static function unpublished():Void {
    var bare = new ErobActuatorPart();
    check(bare.momentRating() == null && bare.bearingStiffness() == null && bare.brakeHoldingTorque() == null, "unpublished values stay unset");
    var list = bare.unpublishedInputs();
    check(list.length == 3, "three unpublished inputs with a brake");
    for (i in list) check(i.basis == Assumed && i.value == null, "unpublished input is Assumed and empty: " + i.name);
    var set = new ErobActuatorPart(new ComponentValues().setNumber("momentRating", 120).setNumber("brakeHoldingTorque", 60));
    near(set.momentRating(), 120, "assumed moment rating");
    near(set.brakeHoldingTorque(), 60, "assumed holding torque");
    check(set.unpublishedInputs()[0].reason.indexOf("Assumed") == 0, "assumption carries its label");
    near(new ErobActuator(100, 20, true, "A", new ComponentValues().setNumber("brakeHoldingTorque", 60)).actuator("x").holdingTorque, 60, "holding torque reaches the actuator");
    check(new ErobActuatorPart(new ComponentValues().setBoolean("brake", false)).unpublishedInputs().length == 2, "no brake, no holding torque input");
    fails(() -> new ErobActuatorPart(new ComponentValues().setBoolean("brake", false).setNumber("brakeHoldingTorque", 5)), "holding torque without brake rejected");
  }

  static function recipeRoundTrip():Void {
    var part = new ErobActuatorPart(new ComponentValues().setToken("ratio", "120").setToken("encoderBits", "19").setBoolean("brake", false));
    check(part.designation == "EROB110I-V6D-120-E19-EC", "designation " + part.designation);
    var rebuilt:ErobActuatorPart = cast part.componentType().create(part.values());
    check(rebuilt.designation == part.designation && rebuilt.ratio == 120, "recipe rebuilds the part");
    var bom = part.bom;
    check(bom.procurement != null && bom.typeId == "machinekit.motion.erob-actuator-part", "bought part with a recipe");
    var out = new ErobActuatorPart(new ComponentValues().setToken("variant", ErobActuatorPart.OUTPUT));
    check(machinekit.component.IncludedInPurchase.of(out) != null, "output flange is part of the actuator purchase");
  }

  static function cadFallback():Void {
    if (!ErobActuatorCad.available()) {
      check(ErobActuatorCad.load() == null, "absent CAD gives null, and nominal geometry stands in");
    } else {
      var cad = ErobActuatorCad.load();
      check(cad != null && cad.sha256 == D.STEP_SHA256, "present CAD matches its pin");
      cad.close();
    }
    var part = new ErobActuatorPart(), g = part.geometry();
    check(g.volume() > 0, "nominal geometry without the file");
    g.close();
  }
}
