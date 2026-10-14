package processkit;

import haxe.Int64;
import motionkit.kinematics.Twist6;
import motionkit.robot.CartesianAdmittance;
import motionkit.robot.ServoSession;
import robotkit.core.ForceTorqueMeasurement;
import robotkit.core.SensorFrame;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Wrench3;
import robotkit.time.ClockMappings;
import processkit.screwdriving.ScrewEntrySearch;
import materia.project.ScrewEntrySearchBounds;
import robotkit.spatial.Twist3;

/** Commissioned contact-load drop used to recognize that the screw entered its hole. */
class SpiralEntryCriterion {
  public final loadAxis:Vec3;
  public final minimumContactLoadN:Float;
  public final requiredDropN:Float;
  public final confirmationSamples:Int;
  public var searchStartedNs(default, null):Null<Int64> = null;
  public var confirmedLoadDrop(default, null):Bool = false;
  public var baselineLoadN(default, null):Null<Float> = null;
  public var lastLoadN(default, null):Float = 0.0;
  var baseline:Null<Float> = null;
  var pendingBaselineLoad:Null<Float> = null;
  var pendingBaselineSamples = 0;
  var confirmations = 0;
  var previousNs:Null<Int64> = null;

  /** `loadAxis` points along positive environmental compression in the robot root frame. */
  public function new(loadAxis:Vec3, minimumContactLoadN:Float, requiredDropN:Float,
		confirmationSamples:Int) {
    if (loadAxis == null || !Math.isFinite(loadAxis.norm()) || Math.abs(loadAxis.norm() - 1.0) > 1e-8 ||
      !Math.isFinite(minimumContactLoadN) || !(minimumContactLoadN > 0) ||
		!Math.isFinite(requiredDropN) || !(requiredDropN > 0) || confirmationSamples < 1 ||
		minimumContactLoadN <= requiredDropN)
      throw "Spiral entry needs an explicit compression axis, load drop and confirmation count";
    this.loadAxis = loadAxis;
    this.minimumContactLoadN = minimumContactLoadN;
    this.requiredDropN = requiredDropN;
    this.confirmationSamples = confirmationSamples;
  }

  /** Uses only fresh measured environmental load; target force is not mistaken for contact. */
  public function observe(wrench:Wrench3, nowNs:Int64, entryWindowReached:Bool):Bool {
    if (wrench == null || Int64.compare(nowNs, Int64.ofInt(0)) < 0)
      throw "Spiral entry needs a measured wrench and robot-clock acquisition time";
    if (previousNs != null && nowNs <= cast(previousNs, Int64))
      throw "Spiral entry sample clock did not advance";
    previousNs = nowNs;
    var load = wrench.force.dot(loadAxis);
    lastLoadN = load;
    if (baseline == null) {
      if (load < minimumContactLoadN) {
        pendingBaselineLoad = null;
        pendingBaselineSamples = 0;
      } else {
        var pending = pendingBaselineLoad;
        if (pending == null || Math.abs(load - cast(pending, Float)) > requiredDropN) {
          pendingBaselineLoad = load;
          pendingBaselineSamples = 1;
        } else {
          pendingBaselineSamples++;
          pendingBaselineLoad = cast(pending, Float) + (load - cast(pending, Float)) / pendingBaselineSamples;
        }
        if (pendingBaselineSamples >= confirmationSamples) {
          baseline = pendingBaselineLoad;
          baselineLoadN = pendingBaselineLoad;
          searchStartedNs = nowNs;
          confirmations = 0;
          pendingBaselineLoad = null;
          pendingBaselineSamples = 0;
        }
      }
		confirmedLoadDrop = false;
		return false;
    }
    if (load > cast(baseline, Float)) {
      baseline = load;
      baselineLoadN = load;
    }
    if (cast(baseline, Float) - load >= requiredDropN) confirmations++ else confirmations = 0;
		confirmedLoadDrop = confirmations >= confirmationSamples;
		return entryWindowReached && confirmedLoadDrop;
  }

	/** Discard a retreat-induced load drop before the next fresh contact attempt. */
	public function resetContactBaseline():Void {
		baseline = null;
		baselineLoadN = null;
		pendingBaselineLoad = null;
		pendingBaselineSamples = 0;
		confirmations = 0;
		searchStartedNs = null;
		confirmedLoadDrop = false;
	}

	/** Require a new measured force-drop confirmation without discarding the fresh compression baseline. */
	public function resetLoadDropConfirmation():Void {
		confirmations = 0;
		confirmedLoadDrop = false;
	}
}

/**
 * Freshness-bounded six-axis admittance over ServoSession. The spiral begins
 * only after measured compressive contact, fits inside the registered contact
 * region, and completes only when the measured force drop is confirmed.
 */
class ContactAdmittanceRunner implements ScrewEntrySearch {
  public final servo:ServoSession;
  public final controller:CartesianAdmittance;
  public final sensorId:String;
  public final sensorFrame:String;
  public final tcpFromSensor:Transform3;
  public final maximumAgeNs:Int64;
  public final maximumForceN:Float;
  public final maximumTorqueNm:Float;
  public final maximumSearchAxialSpeed:Float;
  /** Commissioned axial relief before searching after contact before the CAD entry plane. */
  public final contactReliefMeters:Float;
  public final maximumTravelMeters:Float;
  public final maximumTravelRadians:Float;
  public final maximumRunSeconds:Float;
  public final spiralTrackingGainPerSecond:Float;
  public final expectedEntry:Transform3;
  public final entryDepthToleranceMeters:Float;
  public final candidateProgressTimeoutSeconds:Float;
  public final candidateAdvanceMeters:Float;
  public var complete(default, null):Bool = false;
  public var failure(default, null):Null<String> = null;
  public var entryFound(default, null):Bool = false;
  /** Peak measured force seen during this bounded search, excluding later fastening loads. */
  public var peakForceN(default, null):Float = 0.0;
  public var lastMeasurement(default, null):Null<ForceTorqueMeasurement> = null;
  public var measuredEntry(default, null):Null<Transform3> = null;
  public var confirmedEntryDepthMeters(default, null):Null<Float> = null;
  final mappings:Null<ClockMappings>;
  final originalSpiral:processkit.perception.SpiralSearchTrajectory;
  var spiral:processkit.perception.SpiralSearchTrajectory;
  var initialSeedSearchActive = false;
  var candidateReliefActive = false;
  var suppressCandidateCapture = false;
  final entry:SpiralEntryCriterion;
  public var expectedEntryDepthMeters(default, null):Null<Float> = null;
  var active = false;
  var stopping = false;
  var sourceClock = "";
  var sensorClock:Null<String> = null;
  var startNs:Int64 = Int64.ofInt(0);
  var previousUpdateNs:Int64 = Int64.ofInt(0);
  var previousSequence:Null<Int64> = null;
  var previousSampleNs:Null<Int64> = null;
  var previousControlNs:Null<Int64> = null;
  var spiralElapsedSeconds = 0.0;
  var spiralCatchupElapsedSeconds = 0.0;
	var spiralCatchupRecoveryElapsedSeconds = 0.0;
	var spiralCatchupRecoveryCount = 0;
	var spiralCatchupRecoveryActive = false;
	var spiralCatchupRecoveryStartOffset:Null<Vec3> = null;
  var spiralSearchActive = false;
  var contactReliefAttempted = false;
  var contactReliefTargetDepth:Null<Float> = null;
  var candidateHoleOffset:Null<Vec3> = null;
  var rejectedCandidateOffset:Null<Vec3> = null;
  var candidateStartDepth = 0.0;
  var candidateMaximumDepth = 0.0;
  var candidateProgressDepth = 0.0;
  var candidateElapsedSeconds = 0.0;
  var finalEntrySearchActive = false;
  var entryPlaneSearchActive = false;
  var entryPlaneAdvanceActive = false;
  var entryPlaneAdvanceElapsedSeconds = 0.0;
  var entryPlaneAdvanceLastDepth = 0.0;
  var entryPlaneAdvanceWindowActive = false;
  var searchCenterOffset = Vec3.zero();
  var lastCommanded:Null<Twist6> = null;
  var lastServoVelocity:Array<Float> = [];
  var candidateValidated = false;
  var commandSequence = 0;
  var initialTcp:Null<Transform3> = null;

  /** Convert frame poses in one link into `tcp_T_sensor` for sensor-wrench transport. */
  public static function sensorTransformAtTcp(linkFromSensor:Transform3, linkFromTcp:Transform3):Transform3 {
    if (linkFromSensor == null || linkFromTcp == null)
      throw "Force/torque conversion needs the mounted sensor and screwdriver TCP frames";
    return linkFromTcp.inverse().compose(linkFromSensor);
  }

	/** Bound the tracking error from relative-motion latency and proportional sample lag. */
	public static function sampleTrackingToleranceMeters(maximumTcpSpeed:Float, maximumLateralSpeed:Float,
		controlStepSeconds:Float, maximumAgeSeconds:Float, sampleLateralSpeed:Float,
		trackingGainPerSecond:Float):Float {
		if (!Math.isFinite(maximumTcpSpeed) || !(maximumTcpSpeed > 0) ||
			!Math.isFinite(maximumLateralSpeed) || !(maximumLateralSpeed > 0) ||
			!Math.isFinite(controlStepSeconds) || !(controlStepSeconds > 0) ||
			!Math.isFinite(maximumAgeSeconds) || maximumAgeSeconds < 0 ||
			!Math.isFinite(sampleLateralSpeed) || sampleLateralSpeed < 0 || sampleLateralSpeed > maximumLateralSpeed ||
			!Math.isFinite(trackingGainPerSecond) || !(trackingGainPerSecond > 0))
			throw "Spiral tracking tolerance needs finite commissioned speeds, timing and gain";
		return (maximumTcpSpeed + maximumLateralSpeed) * (controlStepSeconds + maximumAgeSeconds) +
			sampleLateralSpeed / trackingGainPerSecond;
	}

	/** Reserve the remaining commissioned force bound for contact feedback, not trajectory feedforward. */
	public static function forceHeadroomScale(forceMagnitude:Float, maximumForce:Float):Float {
		if (!Math.isFinite(forceMagnitude) || forceMagnitude < 0 || !Math.isFinite(maximumForce) || !(maximumForce > 0))
			throw "Force-limited feedforward needs a finite measured force and positive commissioned bound";
		if (forceMagnitude >= maximumForce) return 0.0;
		return (maximumForce - forceMagnitude) / maximumForce;
	}

	/** Taper all position-search motion after its speed cap, preserving force-response priority. */
	public static function forceLimitedSearchVelocity(velocity:Vec3, maximumSpeed:Float,
		forceMagnitude:Float, maximumForce:Float):Vec3 {
		if (velocity == null || !Math.isFinite(maximumSpeed) || maximumSpeed < 0)
			throw "Force-limited search needs a velocity and a finite non-negative speed budget";
		var speed = velocity.norm();
		if (!Math.isFinite(speed)) throw "Force-limited search velocity must have a finite norm";
		var scale = forceHeadroomScale(forceMagnitude, maximumForce);
		if (speed > maximumSpeed && speed > 0) scale *= maximumSpeed / speed;
		return velocity.scale(scale);
	}

	/** Reserve axial feed from the vector force limit after accounting for measured lateral load. */
	public static function axialForceHeadroomScale(forceMagnitude:Float, axialForceMagnitude:Float,
		maximumForce:Float):Float {
		if (!Math.isFinite(forceMagnitude) || forceMagnitude < 0 || !Math.isFinite(axialForceMagnitude) ||
			axialForceMagnitude < 0 || axialForceMagnitude > forceMagnitude + 1e-9 ||
			!Math.isFinite(maximumForce) || !(maximumForce > 0))
			throw "Axial feed needs finite measured force components inside a positive commissioned bound";
		if (forceMagnitude >= maximumForce || axialForceMagnitude >= maximumForce) return 0.0;
		var lateralSquared = Math.max(0.0, forceMagnitude * forceMagnitude - axialForceMagnitude * axialForceMagnitude);
		var maximumAxialForce = Math.sqrt(Math.max(0.0, maximumForce * maximumForce - lateralSquared));
		return Math.min(1.0, Math.max(0.0, maximumAxialForce - axialForceMagnitude) / maximumForce);
	}

	/** Give an unvalidated candidate half its commissioned timeout to traverse the lead advance. */
	public static function candidateProbeSpeed(maximumSpeed:Float, advanceMeters:Float,
		progressTimeoutSeconds:Float):Float {
		if (!Math.isFinite(maximumSpeed) || !(maximumSpeed > 0) || !Math.isFinite(advanceMeters) ||
			!(advanceMeters > 0) || !Math.isFinite(progressTimeoutSeconds) || !(progressTimeoutSeconds > 0) ||
			advanceMeters > maximumSpeed * progressTimeoutSeconds)
			throw "Candidate probe needs a reachable lead advance and positive speed/time bounds";
		return Math.min(maximumSpeed, 2.0 * advanceMeters / progressTimeoutSeconds);
	}

  public function new(servo:ServoSession, controller:CartesianAdmittance, sensorId:String,
    sensorFrame:String, tcpFromSensor:Transform3, maximumAgeNs:Int64, maximumForceN:Float,
    maximumTorqueNm:Float, maximumSearchAxialSpeed:Float, contactReliefMeters:Float,
    maximumTravelMeters:Float, maximumTravelRadians:Float,
    maximumRunSeconds:Float, spiralTrackingGainPerSecond:Float, mappings:Null<ClockMappings>,
    spiral:processkit.perception.SpiralSearchTrajectory, entry:SpiralEntryCriterion,
    expectedEntry:Transform3, entryDepthToleranceMeters:Float, candidateProgressTimeoutSeconds:Float,
    candidateAdvanceMeters:Float) {
    if (servo == null || controller == null || sensorId == null || sensorId.length == 0 ||
      sensorFrame == null || sensorFrame.length == 0 || tcpFromSensor == null ||
      Int64.compare(maximumAgeNs, Int64.ofInt(0)) <= 0 || !Math.isFinite(maximumForceN) || !(maximumForceN > 0) ||
      !Math.isFinite(maximumTorqueNm) || !(maximumTorqueNm > 0) || !Math.isFinite(maximumTravelMeters) ||
      !Math.isFinite(maximumSearchAxialSpeed) || !(maximumSearchAxialSpeed > 0) ||
      !Math.isFinite(contactReliefMeters) || !(contactReliefMeters > 0) ||
      maximumSearchAxialSpeed > controller.maxLinearSpeed ||
      !(maximumTravelMeters > 0) || !Math.isFinite(maximumTravelRadians) || !(maximumTravelRadians > 0) ||
      controller.maxAngularSpeed * maximumRunSeconds > maximumTravelRadians + 1e-12 ||
      !Math.isFinite(maximumRunSeconds) || !(maximumRunSeconds > 0) ||
      !Math.isFinite(spiralTrackingGainPerSecond) || !(spiralTrackingGainPerSecond > 0) ||
      spiral == null || entry == null || expectedEntry == null || !Math.isFinite(entryDepthToleranceMeters) ||
      entryDepthToleranceMeters < 0 || !Math.isFinite(candidateProgressTimeoutSeconds) ||
      !(candidateProgressTimeoutSeconds > 0) || candidateProgressTimeoutSeconds > maximumRunSeconds ||
      !Math.isFinite(candidateAdvanceMeters) || !(candidateAdvanceMeters > 0) ||
      candidateAdvanceMeters > maximumSearchAxialSpeed * candidateProgressTimeoutSeconds ||
      Math.abs(spiral.contactSeedOffsetUMeters) + spiral.maxRadius > maximumTravelMeters ||
      Math.abs(spiral.axisU.dot(entry.loadAxis)) > 1e-8 || Math.abs(spiral.axisV.dot(entry.loadAxis)) > 1e-8 ||
      Math.abs(spiral.contactSeedOffsetUMeters) + spiral.maxRadius +
        spiral.maxLateralSpeed * Int64.toFloat(maximumAgeNs) * 1e-9 > spiral.region.halfU ||
      spiral.maxRadius + spiral.maxLateralSpeed * Int64.toFloat(maximumAgeNs) * 1e-9 > spiral.region.halfV)
      throw "Contact admittance needs explicit source, frame, force, travel, time and matching search bounds";
    if (servo.robot.snapshot().trajectoryActive)
      throw "Contact admittance cannot take over an active motion program";
    this.servo = servo;
    this.controller = controller;
    this.sensorId = sensorId;
    this.sensorFrame = sensorFrame;
    this.tcpFromSensor = tcpFromSensor;
    this.maximumAgeNs = maximumAgeNs;
    this.maximumForceN = maximumForceN;
    this.maximumTorqueNm = maximumTorqueNm;
    this.maximumSearchAxialSpeed = maximumSearchAxialSpeed;
    this.contactReliefMeters = contactReliefMeters;
    this.maximumTravelMeters = maximumTravelMeters;
    this.maximumTravelRadians = maximumTravelRadians;
    this.maximumRunSeconds = maximumRunSeconds;
    this.spiralTrackingGainPerSecond = spiralTrackingGainPerSecond;
    this.mappings = mappings;
    this.originalSpiral = spiral;
    this.spiral = createSeedSearch();
    initialSeedSearchActive = Math.abs(spiral.contactSeedOffsetUMeters) > 0;
    this.entry = entry;
    this.expectedEntry = expectedEntry;
    this.entryDepthToleranceMeters = entryDepthToleranceMeters;
    this.candidateProgressTimeoutSeconds = candidateProgressTimeoutSeconds;
    this.candidateAdvanceMeters = candidateAdvanceMeters;
  }

  /** Start on the commissioned contact seed and contract toward the CAD nominal axis. */
  function createSeedSearch():processkit.perception.SpiralSearchTrajectory {
    var seed = originalSpiral.contactSeedOffsetUMeters;
    var radius = Math.abs(seed);
    if (radius == 0) return originalSpiral;
    var axisU = seed < 0 ? originalSpiral.axisU.scale(-1) : originalSpiral.axisU;
    return new processkit.perception.SpiralSearchTrajectory(axisU, originalSpiral.axisV,
      originalSpiral.region, radius, originalSpiral.turns, originalSpiral.durationSeconds,
      originalSpiral.maxLateralSpeed, 0, radius, true);
  }

	/** Clear a rejected screw from the bore before resuming the same bounded spiral. */
	function rejectCandidate():Void {
		if (candidateHoleOffset == null)
			throw "Cannot retract a missing force-search candidate";
		rejectedCandidateOffset = candidateHoleOffset;
		candidateHoleOffset = null;
		candidateValidated = false;
		candidateElapsedSeconds = 0;
		suppressCandidateCapture = true;
		candidateReliefActive = true;
		contactReliefTargetDepth = -contactReliefMeters;
		spiralSearchActive = false;
		entryPlaneAdvanceActive = false;
		searchCenterOffset = Vec3.zero();
		entry.resetContactBaseline();
	}

  public function start():Void {
    if (active || complete || failure != null) throw "Contact admittance runner cannot be reused";
    var snapshot = servo.robot.snapshot();
    if (snapshot.sourceClockId == null || snapshot.sourceClockId.length == 0 || snapshot.sourceClockId == "unspecified")
      throw "Contact admittance needs the robot's identified monotonic source clock";
    sourceClock = snapshot.sourceClockId;
    startNs = snapshot.sourceTimestampNs;
    previousUpdateNs = startNs;
    initialTcp = servo.manipulator.tcpPose([for (index in servo.manipulator.jointIndices()) snapshot.positions.get(index)]);
    var origin = initialTcp;
    if (origin == null) throw "Contact admittance lost its initial TCP frame";
    var advanceAxis = entry.loadAxis.scale(-1);
    var expectedDepth = expectedEntry.translation.sub(origin.translation).dot(advanceAxis);
    if (!Math.isFinite(expectedDepth) || !(expectedDepth > entryDepthToleranceMeters))
      throw "Contact admittance CAD entry plane is not ahead of its measured approach pose";
    var freshnessDrift = originalSpiral.maxLateralSpeed * Int64.toFloat(maximumAgeNs) * 1e-9;
    var requiredTravel = ScrewEntrySearchBounds.requiredTravelMeters(expectedDepth, contactReliefMeters,
      originalSpiral.region.halfU, originalSpiral.region.halfV, originalSpiral.contactSeedOffsetUMeters, originalSpiral.maxRadius,
      freshnessDrift, controller.maxLinearSpeed * controller.maxStepSeconds);
    if (maximumTravelMeters < requiredTravel)
      throw 'Contact admittance travel bound ${maximumTravelMeters} m cannot cover the CAD entry pose, ' +
        'contact relief and registered spiral displacement (${requiredTravel} m required)';
    var minimumSpiralSeconds = ScrewEntrySearchBounds.minimumSpiralSeconds(
      expectedDepth, contactReliefMeters, maximumSearchAxialSpeed);
    var minimumRunSeconds = ScrewEntrySearchBounds.minimumRunSeconds(expectedDepth, contactReliefMeters,
      maximumSearchAxialSpeed, originalSpiral.durationSeconds, originalSpiral.maxRadius, originalSpiral.turns,
      originalSpiral.maxLateralSpeed, candidateProgressTimeoutSeconds);
    if (originalSpiral.durationSeconds < minimumSpiralSeconds || maximumRunSeconds < minimumRunSeconds)
      throw 'Contact admittance bounds cannot cover CAD entry depth ${expectedDepth} m: ' +
        'spiral ${originalSpiral.durationSeconds} s < ${minimumSpiralSeconds} s or run ${maximumRunSeconds} s < ${minimumRunSeconds} s';
    expectedEntryDepthMeters = expectedDepth;
    active = true;
  }

  public function running():Bool return active;

  /** Read one robot snapshot, consume only a fresh source acquisition and update the bounded servo. */
  public function update(screwHeld:Bool):Void {
    if (!active) return;
    var snapshot = servo.robot.snapshot();
    var now = snapshot.sourceTimestampNs;
    if (snapshot.sourceClockId != sourceClock) {
      requestStop("force/torque joint clock changed epoch");
      try servo.robot.stop(robotkit.core.StopMode.Normal) catch (_:Dynamic) {}
      var atRest = true;
      for (index in servo.manipulator.jointIndices()) if (Math.abs(snapshot.velocities.get(index)) > 1e-5) atRest = false;
      if (atRest) active = false;
      return;
    }
    if (now < previousUpdateNs) { requestStop("force/torque robot clock moved backwards"); updateBrake(snapshot); return; }
    if (now == previousUpdateNs) return;
    var dt = Int64.toFloat(Int64.sub(now, previousUpdateNs)) * 1e-9;
    previousUpdateNs = now;
    if (!Math.isFinite(dt) || !(dt > 0) || Int64.toFloat(Int64.sub(now, startNs)) * 1e-9 > maximumRunSeconds) {
      requestStop("force/torque run exceeded its commissioned time bound"); updateBrake(snapshot); return;
    }
    if (stopping) { updateBrake(snapshot); return; }
    if (!screwHeld) {
      requestStop("screwdriver lost the held screw during force search");
      updateBrake(snapshot);
      return;
    }
    try {
      var frame:Null<SensorFrame> = null;
      for (index in 0...snapshot.sensors.length) {
        var candidate = snapshot.sensors.get(index);
        if (candidate.sensorId == sensorId) frame = candidate;
      }
      var measurement = ForceTorqueMeasurement.fromFrame(frame, sensorId, sensorFrame, now,
        sourceClock, maximumAgeNs, mappings);
      if (sensorClock != null && measurement.sourceClockId != sensorClock)
        throw 'Force/torque sensor "$sensorId" changed source clock during the active search';
      sensorClock = measurement.sourceClockId;
      if (previousSequence != null) {
        if (measurement.sequence < cast(previousSequence, Int64) ||
          (measurement.sequence > cast(previousSequence, Int64) && measurement.sourceTimestampNs <= cast(previousSampleNs, Int64)))
          throw 'Force/torque sensor "$sensorId" sequence or source time moved backwards';
        if (measurement.sequence == cast(previousSequence, Int64)) {
          var previous = lastMeasurement;
          if (previous == null || measurement.sourceTimestampNs != previous.sourceTimestampNs ||
            !sameWrench(measurement.wrench, previous.wrench))
            throw 'Force/torque sensor "$sensorId" repeated a sequence with conflicting acquisition data';
          servo.update();
          return;
        }
      }
      previousSequence = measurement.sequence;
      previousSampleNs = measurement.sourceTimestampNs;
      lastMeasurement = measurement;
      var joints = [for (index in servo.manipulator.jointIndices()) snapshot.positions.get(index)];
      var tcp = servo.manipulator.tcpPose(joints);
      var origin = initialTcp;
      if (origin == null) throw "Contact admittance lost its initial TCP frame";
      var measuredTravel = tcp.translation.sub(origin.translation).norm();
      if (measuredTravel > maximumTravelMeters)
        throw 'Contact admittance exhausted its measured TCP travel bound: ${measuredTravel} m > ${maximumTravelMeters} m';
      var measuredAngle = tcp.rotation.angularDistance(origin.rotation);
      if (measuredAngle > maximumTravelRadians)
        throw 'Contact admittance exhausted its measured TCP angular travel bound: ${measuredAngle} rad > ${maximumTravelRadians} rad';
      var tcpWrench = tcpFromSensor.transformWrench(measurement.wrench);
      var rootWrench = new Transform3(Vec3.zero(), tcp.rotation).transformWrench(tcpWrench);
      var forceMagnitude = rootWrench.force.norm(), torqueMagnitude = rootWrench.torque.norm();
      if (forceMagnitude > peakForceN) peakForceN = forceMagnitude;
      if (forceMagnitude > maximumForceN)
        throw 'Contact admittance measured ${forceMagnitude} N, above its ${maximumForceN} N force bound';
		var axialForceMagnitude = Math.abs(rootWrench.force.dot(entry.loadAxis));
		var axialForceHeadroomScale = ContactAdmittanceRunner.axialForceHeadroomScale(
			forceMagnitude, axialForceMagnitude, maximumForceN);
      if (torqueMagnitude > maximumTorqueNm)
        throw 'Contact admittance measured ${torqueMagnitude} N m, above its ${maximumTorqueNm} N m torque bound';
      var controlRotation = tcpFromSensor.rotation;
      var controlFrame = new Transform3(Vec3.zero(), tcp.rotation.multiply(controlRotation));
      var localWrench = new Transform3(Vec3.zero(), controlFrame.rotation.conjugate()).transformWrench(rootWrench);
		var controlNs = measurement.robotTimestampNs;
		var integrationNs = previousControlNs == null || Int64.compare(controlNs, startNs) <= 0
			? Int64.fromFloat(Math.round(dt * 1e9)) : Int64.sub(controlNs, cast(previousControlNs, Int64));
		var integrationDt = Int64.toFloat(integrationNs) * 1e-9;
		if (!Math.isFinite(integrationDt) || !(integrationDt > 0)) integrationDt = dt;
		previousControlNs = controlNs;
		var localCorrection = controller.step(localWrench, integrationDt);
      var rootCorrection = controlFrame.transformTwist(new Twist3(
        new Vec3(localCorrection.linearX, localCorrection.linearY, localCorrection.linearZ),
        new Vec3(localCorrection.angularX, localCorrection.angularY, localCorrection.angularZ)));
      var correction = new Twist6(rootCorrection.linear.x, rootCorrection.linear.y, rootCorrection.linear.z,
        rootCorrection.angular.x, rootCorrection.angular.y, rootCorrection.angular.z);
		var expectedDepth = expectedEntryDepthMeters;
		if (expectedDepth == null) throw "Contact admittance lost its CAD entry depth";
		var advanceAxis = entry.loadAxis.scale(-1);
		var axialDepth = tcp.translation.sub(origin.translation).dot(advanceAxis);
		var entryWindowReached = axialDepth >= expectedDepth - entryDepthToleranceMeters &&
			axialDepth <= expectedDepth + entryDepthToleranceMeters;
		if (entryPlaneAdvanceActive && entryWindowReached && !entryPlaneAdvanceWindowActive) {
			entryPlaneAdvanceWindowActive = true;
			entryPlaneAdvanceElapsedSeconds = 0.0;
			entry.resetLoadDropConfirmation();
		}
        var confirmedEntry = entry.observe(rootWrench, measurement.robotTimestampNs, entryWindowReached);
		if (entryPlaneAdvanceActive && !confirmedEntry) {
			if (entryWindowReached) {
				entryPlaneAdvanceElapsedSeconds += dt;
				if (entryPlaneAdvanceElapsedSeconds >= candidateProgressTimeoutSeconds)
					throw "CAD entry plane had no fresh confirmed force drop";
			} else if (axialDepth >= entryPlaneAdvanceLastDepth + entryDepthToleranceMeters) {
				entryPlaneAdvanceLastDepth = axialDepth;
				entryPlaneAdvanceElapsedSeconds = 0.0;
			} else {
				entryPlaneAdvanceElapsedSeconds += dt;
				if (entryPlaneAdvanceElapsedSeconds >= candidateProgressTimeoutSeconds)
					throw "CAD entry verification stalled before plane";
			}
		}
		if (entry.searchStartedNs != null && !contactReliefAttempted && !entryWindowReached) {
			contactReliefAttempted = true;
			contactReliefTargetDepth = -contactReliefMeters;
		}
		if (confirmedEntry) {
        entryFound = true;
        confirmedEntryDepthMeters = axialDepth;
        requestStop(null);
        updateBrake(snapshot);
        return;
      }
		var displacement = tcp.translation.sub(origin.translation);
		var measuredCandidate = originalSpiral.axisU.scale(displacement.dot(originalSpiral.axisU))
			.add(originalSpiral.axisV.scale(displacement.dot(originalSpiral.axisV)));
		if (candidateHoleOffset != null && !finalEntrySearchActive && !entryPlaneSearchActive &&
			!entryPlaneAdvanceActive &&
			expectedDepth != null && (entryWindowReached ||
			Math.abs(cast(expectedDepth, Float) - axialDepth) <= candidateAdvanceMeters + entryDepthToleranceMeters)) {
			// A false candidate can be wedged in the housing bore. Pull the screw back
			// to the safe approach before applying any new lateral search motion.
			rejectCandidate();
		}
		if (candidateHoleOffset != null && !finalEntrySearchActive && !entryPlaneSearchActive) {
			if (axialDepth > candidateMaximumDepth) candidateMaximumDepth = axialDepth;
			if (!candidateValidated && candidateMaximumDepth >= candidateStartDepth + candidateAdvanceMeters) {
				candidateValidated = true;
				candidateProgressDepth = candidateMaximumDepth;
				candidateElapsedSeconds = 0;
			} else if (candidateMaximumDepth >= candidateProgressDepth + candidateAdvanceMeters) {
				// A validated candidate still has to make axial progress toward the CAD entry plane.
				// Retract before the bounded spiral searches elsewhere.
				candidateProgressDepth = candidateMaximumDepth;
				candidateElapsedSeconds = 0;
			} else {
				candidateElapsedSeconds += dt;
				if (candidateElapsedSeconds >= candidateProgressTimeoutSeconds) {
					rejectCandidate();
				}
			}
		}
		var excludedCandidate = rejectedCandidateOffset;
		var candidateExclusionRadius = originalSpiral.maxRadius /
			(2.0 * ScrewEntrySearchBounds.fallbackTurns(originalSpiral.turns));
		if (suppressCandidateCapture && excludedCandidate != null &&
			measuredCandidate.sub(cast(excludedCandidate, Vec3)).norm() > candidateExclusionRadius)
			suppressCandidateCapture = false;
		if (!suppressCandidateCapture && !entryPlaneSearchActive && !entryPlaneAdvanceActive &&
			contactReliefTargetDepth == null && candidateHoleOffset == null &&
			entry.confirmedLoadDrop && !entryWindowReached) {
			var candidateOffset = spiral.axisU.scale(displacement.dot(spiral.axisU))
				.add(spiral.axisV.scale(displacement.dot(spiral.axisV)));
			if (excludedCandidate == null || candidateOffset.sub(cast(excludedCandidate, Vec3)).norm() > candidateExclusionRadius) {
				candidateHoleOffset = candidateOffset;
				candidateStartDepth = axialDepth;
				candidateMaximumDepth = axialDepth;
				candidateProgressDepth = axialDepth;
				candidateElapsedSeconds = 0;
				candidateValidated = false;
			}
		}
      var center = origin.translation;
		var target = center.add(searchCenterOffset);
      if (!spiralSearchActive && !candidateReliefActive && !finalEntrySearchActive && !entryPlaneSearchActive && !entryPlaneAdvanceActive)
        target = target.add(originalSpiral.axisU.scale(originalSpiral.contactSeedOffsetUMeters));
		var searchVelocity = Vec3.zero();
		var prioritizeSearchLateral = false;
		var spiralTargetCatchingUp = false;
		if (spiralSearchActive && !entryPlaneAdvanceActive && (candidateHoleOffset == null || finalEntrySearchActive)) {
			prioritizeSearchLateral = true;
			var sample = spiral.sample(spiralElapsedSeconds);
			var sampleTarget = target.add(sample.offset);
			var sampleError = sampleTarget.sub(tcp.translation);
			var sampleErrorLateral = spiral.axisU.scale(sampleError.dot(spiral.axisU))
				.add(spiral.axisV.scale(sampleError.dot(spiral.axisV)));
			var sampleTrackingTolerance = ContactAdmittanceRunner.sampleTrackingToleranceMeters(
				controller.maxLinearSpeed, spiral.maxLateralSpeed, controller.maxStepSeconds,
				Int64.toFloat(maximumAgeNs) * 1e-9, sample.velocity.norm(), spiralTrackingGainPerSecond);
			var sampleLateralError = sampleErrorLateral.norm();
			var spiralCatchingUp = sampleLateralError > sampleTrackingTolerance;
			if (spiralCatchupRecoveryActive) {
				spiralCatchupRecoveryElapsedSeconds += dt;
				var recoveryLimit = ScrewEntrySearchBounds.catchupRecoverySeconds(contactReliefMeters,
					maximumSearchAxialSpeed, candidateProgressTimeoutSeconds);
				if (spiralCatchupRecoveryElapsedSeconds >= recoveryLimit)
					throw 'Spiral catch-up relief recovery exceeded its ${recoveryLimit} s bound after ' +
						'${spiralCatchupRecoveryElapsedSeconds} s at depth ${axialDepth} m, lateral error ' +
						'${sampleLateralError} m (tolerance ${sampleTrackingTolerance} m)';
			}
			if (spiralCatchingUp) {
				// A briefly reachable sample does not refresh the total catch-up allowance.
				if (!spiralCatchupRecoveryActive && contactReliefTargetDepth == null)
					spiralCatchupElapsedSeconds += dt;
				if (!spiralCatchupRecoveryActive && contactReliefTargetDepth == null &&
					spiralCatchupElapsedSeconds >= candidateProgressTimeoutSeconds) {
					if (spiralCatchupRecoveryCount == 0) {
						spiralCatchupRecoveryCount++;
						spiralCatchupRecoveryActive = true;
						spiralCatchupRecoveryStartOffset = sample.offset;
						spiralCatchupRecoveryElapsedSeconds = 0.0;
						contactReliefTargetDepth = ScrewEntrySearchBounds.catchupReliefDepthMeters(
							axialDepth, contactReliefMeters);
						entry.resetContactBaseline();
						spiralCatchupElapsedSeconds = 0.0;
					} else {
						throw 'Spiral search could not reach its bounded measured target after one relief retry: ' +
							'elapsed ${spiralCatchupElapsedSeconds} s, lateral error ${sampleLateralError} m ' +
							'(tolerance ${sampleTrackingTolerance} m), measured ${tcp.translation.x},' +
							'${tcp.translation.y},${tcp.translation.z}, target ${sampleTarget.x},' +
							'${sampleTarget.y},${sampleTarget.z}, sample ${sample.offset.x},' +
							'${sample.offset.y},${sample.offset.z} at ${spiralElapsedSeconds} s';
					}
				}
			} else {
				// Force headroom limits position-search commands, not the trajectory clock.
				// Account for measured tracking lag through the bounded recovery instead.
				spiralElapsedSeconds += dt;
				sample = spiral.sample(spiralElapsedSeconds);
				searchVelocity = sample.velocity;
			}
			target = target.add(sample.offset);
			if (sample.complete && !spiralCatchingUp && finalEntrySearchActive) {
				// Local refinement found no entry. Finish tracking its measured endpoint before
				// searching the full registered region around the CAD axis.
				rejectedCandidateOffset = candidateHoleOffset;
				candidateHoleOffset = null;
				finalEntrySearchActive = false;
				entryPlaneSearchActive = true;
				searchCenterOffset = Vec3.zero();
				var fallbackTurns = ScrewEntrySearchBounds.fallbackTurns(originalSpiral.turns);
				spiral = new processkit.perception.SpiralSearchTrajectory(originalSpiral.axisU, originalSpiral.axisV,
					originalSpiral.region, originalSpiral.maxRadius, fallbackTurns,
					ScrewEntrySearchBounds.fallbackDurationSeconds(
						originalSpiral.maxRadius, originalSpiral.turns, originalSpiral.maxLateralSpeed), originalSpiral.maxLateralSpeed);
				spiralElapsedSeconds = dt;
				entry.resetContactBaseline();
				target = center.add(spiral.axisU.scale(spiral.contactSeedOffsetUMeters));
				sample = spiral.sample(spiralElapsedSeconds);
				searchVelocity = sample.velocity;
			}
			if (sample.complete && !spiralCatchingUp && entryPlaneSearchActive) {
				// The full registered lateral sweep found no entry. Hold its measured lateral
				// endpoint, reacquire compression, and verify the exact CAD plane under axial feed.
				var displacement = tcp.translation.sub(origin.translation);
				searchCenterOffset = spiral.axisU.scale(displacement.dot(spiral.axisU))
					.add(spiral.axisV.scale(displacement.dot(spiral.axisV)));
				target = center.add(searchCenterOffset);
				entryPlaneSearchActive = false;
				entryPlaneAdvanceActive = true;
				entryPlaneAdvanceElapsedSeconds = 0.0;
				entryPlaneAdvanceLastDepth = axialDepth;
				entryPlaneAdvanceWindowActive = false;
				spiralSearchActive = false;
				candidateHoleOffset = null;
				candidateValidated = false;
				candidateElapsedSeconds = 0.0;
				entry.resetContactBaseline();
			}
			if (sample.complete && !spiralCatchingUp && initialSeedSearchActive && !entryPlaneAdvanceActive) {
				// Search from the measured contact seed back toward the CAD axis first. If that
				// inward pass misses, cover the registered region around the CAD axis itself.
				initialSeedSearchActive = false;
				entryPlaneSearchActive = true;
				searchCenterOffset = Vec3.zero();
				var fallbackTurns = ScrewEntrySearchBounds.fallbackTurns(originalSpiral.turns);
				spiral = new processkit.perception.SpiralSearchTrajectory(originalSpiral.axisU, originalSpiral.axisV,
					originalSpiral.region, originalSpiral.maxRadius, fallbackTurns,
					ScrewEntrySearchBounds.fallbackDurationSeconds(
						originalSpiral.maxRadius, originalSpiral.turns, originalSpiral.maxLateralSpeed),
					originalSpiral.maxLateralSpeed);
				spiralElapsedSeconds = dt;
				entry.resetContactBaseline();
				target = center;
				sample = spiral.sample(spiralElapsedSeconds);
				searchVelocity = sample.velocity;
			}
			if (sample.complete && !spiralCatchingUp && !entryPlaneAdvanceActive) throw entryPlaneSearchActive
				? "CAD entry-plane registered spiral exhausted without a confirmed force drop"
				: "Spiral hole search exhausted its registered radius and time bound";
			var targetError = target.sub(tcp.translation);
			var targetErrorLateral = spiral.axisU.scale(targetError.dot(spiral.axisU))
				.add(spiral.axisV.scale(targetError.dot(spiral.axisV)));
			spiralTargetCatchingUp = targetErrorLateral.norm() > sampleTrackingTolerance;
		}
		var candidate = candidateHoleOffset;
		if (candidate != null && !finalEntrySearchActive) {
			target = center.add(cast(candidate, Vec3));
      }
      {
        var displacement = tcp.translation.sub(center);
        var offsetU = displacement.dot(spiral.axisU), offsetV = displacement.dot(spiral.axisV);
        if (Math.abs(offsetU) > spiral.region.halfU || Math.abs(offsetV) > spiral.region.halfV)
          throw "Measured screwdriver TCP left the registered spiral-search region";
		// Track only within the hole plane. Axial tracking would cancel the admittance
		// displacement and hold the TCP at its pre-contact height forever.
		var trackingError = target.sub(tcp.translation);
		var tracking = spiral.axisU.scale(trackingError.dot(spiral.axisU))
			.add(spiral.axisV.scale(trackingError.dot(spiral.axisV))).scale(spiralTrackingGainPerSecond);
		var correctedLinear = new Vec3(correction.linearX, correction.linearY, correction.linearZ);
		var feedbackLateral = spiral.axisU.scale(correctedLinear.dot(spiral.axisU))
			.add(spiral.axisV.scale(correctedLinear.dot(spiral.axisV)));
		var feedbackSpeed = feedbackLateral.norm();
		// Measured contact response takes precedence over every position-search term.
		var feedbackLimit = prioritizeSearchLateral ? controller.maxLinearSpeed : spiral.maxLateralSpeed;
		if (feedbackSpeed > feedbackLimit)
			feedbackLateral = feedbackLateral.scale(feedbackLimit / feedbackSpeed);
		var searchBudget = Math.min(spiral.maxLateralSpeed,
			Math.max(0.0, feedbackLimit - feedbackLateral.norm()));
		var searchLateral = ContactAdmittanceRunner.forceLimitedSearchVelocity(searchVelocity.add(tracking),
			searchBudget, forceMagnitude, maximumForceN);
		var lateral = feedbackLateral.add(searchLateral);
		var linear = correctedLinear.sub(spiral.axisU.scale(correctedLinear.dot(spiral.axisU)))
			.sub(spiral.axisV.scale(correctedLinear.dot(spiral.axisV))).add(lateral);
		var axial = entry.loadAxis.scale(linear.dot(entry.loadAxis));
		var reliefTarget = contactReliefTargetDepth;
		if (reliefTarget != null && axialDepth > cast(reliefTarget, Float)) {
			// Reach the selected axial clearance before resuming contact search.
			axial = entry.loadAxis.scale(maximumSearchAxialSpeed);
		} else if (reliefTarget != null) {
			contactReliefTargetDepth = null;
			entry.resetContactBaseline();
			var resumeRejectedCandidateSearch = candidateReliefActive;
			if (resumeRejectedCandidateSearch) {
				candidateReliefActive = false;
			}
			spiralSearchActive = true;
			if (spiralCatchupRecoveryActive) {
				// Stay clear of the work while the spiral moves beyond the stalled sample.
				axial = entry.loadAxis.scale(linear.dot(entry.loadAxis));
			} else {
				if (!resumeRejectedCandidateSearch) spiralElapsedSeconds = 0.0;
				axial = entry.loadAxis.scale(-maximumSearchAxialSpeed);
			}
		} else if (spiralCatchupRecoveryActive) {
			var stalledOffset = spiralCatchupRecoveryStartOffset;
			var currentOffset = spiral.sample(spiralElapsedSeconds).offset;
			var recoveryDistance = originalSpiral.maxRadius /
				(2.0 * ScrewEntrySearchBounds.fallbackTurns(originalSpiral.turns));
			if (stalledOffset == null || currentOffset.sub(cast(stalledOffset, Vec3)).norm() < recoveryDistance ||
				spiralTargetCatchingUp) {
				axial = entry.loadAxis.scale(linear.dot(entry.loadAxis));
			} else {
				spiralCatchupRecoveryActive = false;
				spiralCatchupRecoveryStartOffset = null;
				spiralCatchupRecoveryElapsedSeconds = 0.0;
				spiralCatchupElapsedSeconds = 0.0;
				entry.resetContactBaseline();
				axial = axial.add(entry.loadAxis.scale(-maximumSearchAxialSpeed * axialForceHeadroomScale));
			}
		} else if (finalEntrySearchActive && !entryWindowReached) {
				// Do not advance during any retained local candidate refinement.
			axial = entry.loadAxis.scale(linear.dot(entry.loadAxis));
		} else if (spiralSearchActive && candidateHoleOffset == null && entry.searchStartedNs != null) {
			// Maintain measured compression while sweeping laterally. Feeding deeper
			// before a candidate is captured can wedge the screw in a clearance bore.
			axial = entry.loadAxis.scale(linear.dot(entry.loadAxis));
		} else if (spiralTargetCatchingUp && !entryWindowReached) {
			// Do not feed axially while the measured TCP is still catching up to a bounded search sample.
			axial = entry.loadAxis.scale(linear.dot(entry.loadAxis));
		} else if ((finalEntrySearchActive || entryPlaneSearchActive || entryPlaneAdvanceActive) && entryWindowReached) {
			// Hold the CAD entry depth while a bounded search or final verification checks measured load.
			axial = entry.loadAxis.scale(linear.dot(entry.loadAxis));
		} else {
			// Advance at the commissioned feed while admittance corrects measured load.
			var axialFeedSpeed = maximumSearchAxialSpeed;
			if (candidateHoleOffset != null && !candidateValidated)
				axialFeedSpeed = ContactAdmittanceRunner.candidateProbeSpeed(maximumSearchAxialSpeed,
					candidateAdvanceMeters, candidateProgressTimeoutSeconds);
			axial = axial.add(entry.loadAxis.scale(-axialFeedSpeed * axialForceHeadroomScale));
			var axialSpeed = axial.norm();
			if (axialSpeed > maximumSearchAxialSpeed)
				axial = axial.scale(maximumSearchAxialSpeed / axialSpeed);
		}
		if (entryPlaneAdvanceActive && !entryWindowReached) {
			// During final verification, keep a bounded minimum approach speed so axial
			// admittance cannot exactly cancel the feed before the CAD entry plane.
			var minimumProgressSpeed = candidateAdvanceMeters / candidateProgressTimeoutSeconds * axialForceHeadroomScale;
			var commandedProgressSpeed = axial.dot(advanceAxis);
			if (commandedProgressSpeed < minimumProgressSpeed)
				axial = axial.add(advanceAxis.scale(minimumProgressSpeed - commandedProgressSpeed));
			var lateralLimit = Math.sqrt(Math.max(0.0,
				controller.maxLinearSpeed * controller.maxLinearSpeed - axial.norm() * axial.norm()));
			var lateralSpeed = lateral.norm();
			if (lateralSpeed > lateralLimit && lateralSpeed > 0)
				lateral = lateral.scale(lateralLimit / lateralSpeed);
			linear = lateral.add(axial);
		} else {
			linear = linear.sub(entry.loadAxis.scale(linear.dot(entry.loadAxis))).add(axial);
		}
        var linearSpeed = linear.norm();
        if (linearSpeed > controller.maxLinearSpeed) linear = linear.scale(controller.maxLinearSpeed / linearSpeed);
        correction = new Twist6(linear.x, linear.y, linear.z, correction.angularX,
          correction.angularY, correction.angularZ);
      }
      commandSequence++;
      if (commandSequence <= 0) throw "Contact admittance command sequence exhausted";
      lastCommanded = correction;
      var rejected = servo.command(correction, commandSequence, now + maximumAgeNs);
      if (rejected != null) throw 'Contact admittance servo command rejected: $rejected';
      var tick = servo.update();
      lastServoVelocity = tick.velocity.copy();
    } catch (error:Dynamic) {
      requestStop(Std.string(error));
      updateBrake(snapshot);
    }
  }

  public function cancel():Void if (active) {
    requestStop("contact admittance cancelled");
    try servo.robot.stop(robotkit.core.StopMode.Normal) catch (_:Dynamic) {}
  }

  public function isComplete():Bool return complete;
  public function failureReason():Null<String> return failure;
  public function entryPose():Null<Transform3> return measuredEntry;

  function requestStop(reason:Null<String>):Void {
    if (reason != null && failure == null) failure = reason;
    stopping = true;
    controller.stop();
    servo.stop();
  }

  static function sameWrench(left:Wrench3, right:Wrench3):Bool
    return left.force.x == right.force.x && left.force.y == right.force.y && left.force.z == right.force.z &&
      left.torque.x == right.torque.x && left.torque.y == right.torque.y && left.torque.z == right.torque.z;

  function updateBrake(snapshot:robotkit.core.RobotSnapshot):Void {
    var tick = servo.update();
    var observedRest = !snapshot.trajectoryActive;
    for (index in servo.manipulator.jointIndices())
      if (Math.abs(snapshot.velocities.get(index)) > 1e-5) observedRest = false;
    if (tick.atRest && observedRest) {
      active = false;
      var joints = [for (index in servo.manipulator.jointIndices()) snapshot.positions.get(index)];
      measuredEntry = servo.manipulator.tcpPose(joints);
      if (failure == null && entryFound) complete = true;
      else if (failure == null) failure = "contact admittance stopped without a confirmed hole entry";
    }
  }
}
