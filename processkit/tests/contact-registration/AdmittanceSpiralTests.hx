import motionkit.robot.CartesianAdmittance;
import processkit.perception.ContactPoseEnvelope.ContactRegionBounds;
import processkit.perception.SpiralSearchTrajectory;
import processkit.ContactAdmittanceRunner.SpiralEntryCriterion;
import processkit.ContactAdmittanceRunner;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;
import robotkit.spatial.Wrench3;

class AdmittanceSpiralTests {
  static var checks = 0;
  static function check(value:Bool, message:String):Void {
    checks++;
    if (!value) throw message;
  }
  public static function run():Void {
    var sensorPose = new Transform3(Vec3.zero(), Quat.identity());
    var tcpPose = new Transform3(new Vec3(0, 0, 0.03), Quat.identity());
    var tcpFromSensor = ContactAdmittanceRunner.sensorTransformAtTcp(sensorPose, tcpPose);
    var shiftedWrench = tcpFromSensor.transformWrench(new Wrench3(new Vec3(10, 0, 0), Vec3.zero()));
    check(Math.abs(tcpFromSensor.translation.z + 0.03) < 1e-12 &&
      Math.abs(shiftedWrench.torque.y + 0.3) < 1e-12,
      "A sensor wrench is shifted from the mounted sensor origin to the screwdriver TCP");
    var turnedSensor = new Transform3(Vec3.zero(), Quat.fromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2));
    var rotatedWrench = ContactAdmittanceRunner.sensorTransformAtTcp(turnedSensor, sensorPose)
      .transformWrench(new Wrench3(new Vec3(1, 0, 0), Vec3.zero()));
    check(Math.abs(rotatedWrench.force.x) < 1e-12 && Math.abs(rotatedWrench.force.y - 1) < 1e-12,
      "A sensor wrench is rotated into the screwdriver TCP frame in the correct direction");
	var trackingTolerance = ContactAdmittanceRunner.sampleTrackingToleranceMeters(
		0.00015, 0.00014, 0.01, 0.02, 0.0000274, 6.0);
	check(Math.abs(trackingTolerance - (0.00029 * 0.03 + 0.0000274 / 6.0)) < 1e-12,
		"Spiral tracking tolerance covers relative sensor/control motion and proportional sample lag");
	check(ContactAdmittanceRunner.forceHeadroomScale(0, 25) == 1 &&
		Math.abs(ContactAdmittanceRunner.forceHeadroomScale(10, 25) - 0.6) < 1e-12 &&
		ContactAdmittanceRunner.forceHeadroomScale(25, 25) == 0 &&
		ContactAdmittanceRunner.forceHeadroomScale(30, 25) == 0,
		"Trajectory feedforward consumes only the remaining commissioned force headroom");
	var trackingAtHighLoad = ContactAdmittanceRunner.forceLimitedSearchVelocity(new Vec3(0.003, 0.004, 0),
		0.00014, 20, 25);
	check(Math.abs(trackingAtHighLoad.norm() - 0.000028) < 1e-12 &&
		Math.abs(trackingAtHighLoad.x / trackingAtHighLoad.y - 0.75) < 1e-12,
		"A large position-tracking error cannot saturate away the measured force-headroom taper");
	check(ContactAdmittanceRunner.forceLimitedSearchVelocity(new Vec3(1, 0, 0), 0, 0, 25).norm() == 0 &&
		ContactAdmittanceRunner.forceLimitedSearchVelocity(new Vec3(1, 0, 0), 0.00014, 25, 25).norm() == 0,
		"An exhausted speed or force budget admits no position-search motion");
	check(Math.abs(ContactAdmittanceRunner.axialForceHeadroomScale(10, 10, 25) - 0.6) < 1e-12 &&
		Math.abs(ContactAdmittanceRunner.axialForceHeadroomScale(24, 0, 25) - 0.28) < 1e-12 &&
		ContactAdmittanceRunner.axialForceHeadroomScale(25, 0, 25) == 0,
		"Axial feed reserves vector-force headroom after separating lateral contact load");
	check(Math.abs(ContactAdmittanceRunner.candidateProbeSpeed(0.00012, 0.001, 20) - 0.0001) < 1e-12 &&
		ContactAdmittanceRunner.candidateProbeSpeed(0.00012, 0.0005, 20) < 0.00012,
		"An unvalidated candidate uses only half its progress timeout for the commissioned lead advance");

    var controller = new CartesianAdmittance(new Wrench3(new Vec3(10, 0, 0), Vec3.zero()),
      [1, 1, 1, 1, 1, 1], [10, 10, 10, 10, 10, 10], 0.05, 0.1, 0.01);
    var first = controller.step(Wrench3.zero(), 0.01);
    check(Math.abs(first.linearX + 0.05) < 1e-12 && first.linearY == 0 && first.angularZ == 0,
      "A force deficit moves against the measured external wrench at a bounded speed");
    var settled = controller.step(new Wrench3(new Vec3(10, 0, 0), Vec3.zero()), 0.01);
    check(Math.abs(settled.linearX + 0.045) < 1e-12,
      "Measured target force damps the prior correction instead of accumulating motion");
    controller.stop();
    var disturbed = controller.step(new Wrench3(new Vec3(-1000, 0, 0), new Vec3(0, 0, 1000)), 0.01);
    check(disturbed.linearX < 0 && Math.abs(disturbed.linearX) <= controller.maxLinearSpeed + 1e-12 &&
      Math.abs(disturbed.angularZ) <= controller.maxAngularSpeed + 1e-12,
      "Large force disturbances remain inside explicit linear and angular speed bounds");
    controller.stop();
    var exactPeriod = controller.step(Wrench3.zero(), 0.01);
    controller.stop();
    var quantizedPeriod = controller.step(Wrench3.zero(), 0.010000001);
    check(Math.abs(exactPeriod.linearX - quantizedPeriod.linearX) < 1e-12,
      "A one-nanosecond clock quantization excess integrates only the commissioned period");
    var rejected = false;
    try controller.step(Wrench3.zero(), 0.010000003) catch (_:Dynamic) rejected = true;
    check(rejected, "An integration period beyond the commissioned bound is rejected");
    rejected = false;
    try new CartesianAdmittance(Wrench3.zero(), [1, 0, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1], 0.1, 0.1, 0.01)
      catch (_:Dynamic) rejected = true;
    check(rejected, "Missing or zero commissioned gains cannot silently enable force control");

    var region = new ContactRegionBounds(0.01, 0.01, 0.02, 0.001, 0.001);
    var spiral = new SpiralSearchTrajectory(new Vec3(1, 0, 0), new Vec3(0, 1, 0), region,
      0.009, 2, 5, 0.03);
    var start = spiral.sample(0), middle = spiral.sample(2.5), end = spiral.sample(5);
    check(start.offset.norm() == 0 && middle.offset.x > 0 && Math.abs(middle.offset.y) < 1e-10 &&
      middle.offset.norm() <= region.halfU && middle.velocity.norm() <= spiral.maxLateralSpeed,
      "The spiral remains within the registered region and its commanded lateral speed bound");
    check(end.complete && end.velocity.norm() == 0 && end.offset.norm() <= spiral.maxRadius,
      "Search exhaustion holds its bounded endpoint with zero lateral command");
    var ringRadius = 0.001;
    var ringDuration = 2.0 * Math.PI * ringRadius * 2.0 / 0.00014;
    var ring = new SpiralSearchTrajectory(new Vec3(1, 0, 0), new Vec3(0, 1, 0), region,
      ringRadius, 2, ringDuration, 0.00014, 0, ringRadius);
    var ringStart = ring.sample(0), ringHalf = ring.sample(ringDuration / 2.0);
    check(Math.abs(ringStart.offset.norm() - ringRadius) < 1e-12 &&
      Math.abs(ringHalf.offset.norm() - ringRadius) < 1e-12 &&
      ringStart.velocity.norm() <= ring.maxLateralSpeed && ringHalf.velocity.norm() <= ring.maxLateralSpeed,
      "The bounded local entry search repeats its commissioned circumference at the lateral speed limit");
    var refinementTurns = 16.0;
    var refinementDuration = ringRadius * (1.0 + 2.0 * Math.PI * refinementTurns) / 0.00014;
    var refinement = new SpiralSearchTrajectory(new Vec3(1, 0, 0), new Vec3(0, 1, 0), region,
      ringRadius, refinementTurns, refinementDuration, 0.00014, 0, ringRadius, true);
    var refinementStart = refinement.sample(0), refinementHalf = refinement.sample(refinementDuration / 2.0);
    var refinementEnd = refinement.sample(refinementDuration);
    check(Math.abs(refinementStart.offset.norm() - ringRadius) < 1e-12 &&
      Math.abs(refinementHalf.offset.norm() - ringRadius / 2.0) < 1e-12 && refinementEnd.complete &&
      refinementEnd.offset.norm() < 1e-12 && refinementStart.velocity.norm() <= refinement.maxLateralSpeed &&
      refinementHalf.velocity.norm() <= refinement.maxLateralSpeed,
      "Dense inward refinement crosses every radius around a candidate within its lateral speed bound");
    var fallbackRadius = 0.0016, fallbackTurns = 14.0, fallbackSpeed = 0.00014;
    var fallbackDuration = fallbackRadius * (1.0 + 2.0 * Math.PI * fallbackTurns) / fallbackSpeed;
    var fallback = new SpiralSearchTrajectory(new Vec3(1, 0, 0), new Vec3(0, 1, 0), region,
      fallbackRadius, fallbackTurns, fallbackDuration, fallbackSpeed);
    check(fallback.sample(0).velocity.norm() <= fallbackSpeed + 1e-12,
      "A derived whole-region fallback accepts its commissioned speed within floating-point rounding");
    var seededSpiral = new SpiralSearchTrajectory(new Vec3(1, 0, 0), new Vec3(0, 1, 0), region,
      0.009, 2, 5, 0.03, 0.0005);
    check(seededSpiral.contactSeedOffsetUMeters == 0.0005 && seededSpiral.sample(0).offset.norm() == 0,
      "A commissioned contact seed is kept distinct from the spiral centered on its measured preload");
    var seedReturn = new SpiralSearchTrajectory(new Vec3(1, 0, 0), new Vec3(0, 1, 0), region,
      0.0005, 2, 5, 0.03, 0, 0.0005, true);
    check(Math.abs(seedReturn.sample(0).offset.x - 0.0005) < 1e-12 &&
      Math.abs(seedReturn.sample(2.5).offset.norm() - 0.00025) < 1e-12 &&
      seedReturn.sample(5).offset.norm() < 1e-12,
      "A seeded entry pass contracts from the measured preload toward the CAD nominal axis");
    rejected = false;
    try new SpiralSearchTrajectory(new Vec3(1, 0, 0), new Vec3(0, 1, 0), region, 0.011, 1, 5, 0.03)
      catch (_:Dynamic) rejected = true;
    check(rejected, "A search radius beyond the registered hole region is rejected");
    rejected = false;
    try new SpiralSearchTrajectory(new Vec3(1, 0, 0), new Vec3(1, 0, 0), region, 0.005, 1, 5, 0.03)
      catch (_:Dynamic) rejected = true;
    check(rejected, "Non-orthogonal search axes cannot distort the commissioned radius");
    rejected = false;
    try new SpiralSearchTrajectory(new Vec3(1, 0, 0), new Vec3(0, 1, 0), region,
      0.009, 1, 5, 0.03, 0.0011) catch (_:Dynamic) rejected = true;
    check(rejected, "The contact seed and search radius cannot exceed the registered lateral region");
    rejected = false;
    try spiral.sample(4) catch (_:Dynamic) rejected = true;
    check(rejected, "A backwards sensor/control time cannot replay an earlier search offset");
    var entry = new SpiralEntryCriterion(new Vec3(0, 0, 1), 4, 2, 2);
    check(!entry.observe(new Wrench3(new Vec3(0, 0, 3), Vec3.zero()), haxe.Int64.ofInt(1), true) &&
      entry.searchStartedNs == null, "A commanded target cannot substitute for measured probe contact");
    check(!entry.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(2), true) &&
      entry.searchStartedNs == null &&
      !entry.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(3), true) &&
      entry.searchStartedNs == haxe.Int64.ofInt(3), "Spiral timing starts from a stable measured compression baseline");
    check(!entry.observe(new Wrench3(new Vec3(0, 0, 2), Vec3.zero()), haxe.Int64.ofInt(4), true) &&
      entry.observe(new Wrench3(new Vec3(0, 0, 2), Vec3.zero()), haxe.Int64.ofInt(5), true),
      "A force drop requires the commissioned number of fresh confirming samples");
    var relieved = new SpiralEntryCriterion(new Vec3(0, 0, 1), 4, 2, 2);
    relieved.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(1), false);
    relieved.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(2), false);
    relieved.observe(Wrench3.zero(), haxe.Int64.ofInt(3), false);
    relieved.resetContactBaseline();
    check(relieved.searchStartedNs == null && !relieved.confirmedLoadDrop &&
      !relieved.observe(Wrench3.zero(), haxe.Int64.ofInt(4), true) &&
      !relieved.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(5), true) &&
      !relieved.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(6), true) &&
      !relieved.observe(new Wrench3(new Vec3(0, 0, 2), Vec3.zero()), haxe.Int64.ofInt(7), true) &&
      relieved.observe(new Wrench3(new Vec3(0, 0, 2), Vec3.zero()), haxe.Int64.ofInt(8), true),
      "A contact relief clears its retreat-induced load drop and requires a new compression baseline");
    var earlyDrop = new SpiralEntryCriterion(new Vec3(0, 0, 1), 4, 2, 2);
    check(!earlyDrop.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(5), false) &&
      !earlyDrop.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(6), false) &&
      !earlyDrop.observe(new Wrench3(new Vec3(0, 0, 2), Vec3.zero()), haxe.Int64.ofInt(7), false) &&
      !earlyDrop.observe(new Wrench3(new Vec3(0, 0, 2), Vec3.zero()), haxe.Int64.ofInt(8), false) &&
      earlyDrop.searchStartedNs == haxe.Int64.ofInt(6) && earlyDrop.confirmedLoadDrop,
      "An early stable force drop can guide the search without confirming CAD entry");
    check(!earlyDrop.observe(new Wrench3(new Vec3(0, 0, 8), Vec3.zero()), haxe.Int64.ofInt(9), true) &&
      !earlyDrop.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(10), true) &&
      earlyDrop.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(11), true),
      "Entry needs a fresh load drop inside the CAD entry-depth window");
		var resetDrop = new SpiralEntryCriterion(new Vec3(0, 0, 1), 4, 2, 2);
		resetDrop.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(1), false);
		resetDrop.observe(new Wrench3(new Vec3(0, 0, 5), Vec3.zero()), haxe.Int64.ofInt(2), false);
		resetDrop.observe(new Wrench3(new Vec3(0, 0, 2), Vec3.zero()), haxe.Int64.ofInt(3), false);
		resetDrop.observe(new Wrench3(new Vec3(0, 0, 2), Vec3.zero()), haxe.Int64.ofInt(4), false);
		check(resetDrop.baselineLoadN == 5 && resetDrop.confirmedLoadDrop,
			"A lateral sweep can identify an early drop while retaining its measured compression baseline");
		resetDrop.resetLoadDropConfirmation();
		check(!resetDrop.confirmedLoadDrop && resetDrop.baselineLoadN == 5 &&
			!resetDrop.observe(new Wrench3(new Vec3(0, 0, 2), Vec3.zero()), haxe.Int64.ofInt(5), true) &&
			resetDrop.observe(new Wrench3(new Vec3(0, 0, 2), Vec3.zero()), haxe.Int64.ofInt(6), true),
			"Final CAD-plane verification requires a new force-drop confirmation against the fresh baseline");
    var settling = new SpiralEntryCriterion(new Vec3(0, 0, 1), 4, 1, 2);
    check(!settling.observe(new Wrench3(new Vec3(0, 0, 11.66), Vec3.zero()), haxe.Int64.ofInt(1), false) &&
      !settling.observe(new Wrench3(new Vec3(0, 0, 6.93), Vec3.zero()), haxe.Int64.ofInt(2), false) &&
      !settling.observe(new Wrench3(new Vec3(0, 0, 6.90), Vec3.zero()), haxe.Int64.ofInt(3), false) &&
      settling.searchStartedNs == haxe.Int64.ofInt(3) && settling.baselineLoadN != null &&
      Math.abs(cast(settling.baselineLoadN, Float) - 6.915) < 1e-9 && !settling.confirmedLoadDrop,
      "Recontact settling becomes the fresh baseline instead of a false entry drop");
    var missingContact = new SpiralEntryCriterion(new Vec3(0, 0, 1), 4, 2, 2);
    check(!missingContact.observe(Wrench3.zero(), haxe.Int64.ofInt(1), true) &&
      !missingContact.observe(Wrench3.zero(), haxe.Int64.ofInt(2), true) &&
      !missingContact.observe(Wrench3.zero(), haxe.Int64.ofInt(3), true) && missingContact.searchStartedNs == null,
      "Low-force axial travel without a measured compression baseline cannot report entry");
    Sys.println('Bounded admittance and spiral policy: $checks assertions passed');
  }
}
