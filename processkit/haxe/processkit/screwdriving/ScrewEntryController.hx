package processkit.screwdriving;

import motionkit.kinematics.Twist6;
import motionkit.robot.CartesianAdmittance;
import robotkit.spatial.Vec3;
import robotkit.spatial.Wrench3;

enum ScrewEntryPhase {
  /** Free descent at the approach speed, holding the lateral target. */
  Approach;
  /** Contact held at the commissioned force while the spiral looks for a hole in the touched surface. */
  Search;
  /** Through a found hole: guarded descent, laterally compliant so the bore centres the screw. */
  Descend;
  /** Past the CAD entry plane by the entry advance at the registered hole: the tightening handover. */
  Seated;
  Failed;
}

/** Commissioned numbers of one force-guided entry. Lengths are metres, speeds m/s, forces N. */
class ScrewEntryLimits {
  /** Axial force the admittance regulates to while the screw touches a surface. */
  public final contactForceN:Float;
  /** Compression that, held for `confirmationSamples`, confirms contact. */
  public final contactThresholdN:Float;
  public final confirmationSamples:Int;
  /** Free-space descent speed: the guarded move before and between contacts. */
  public final approachSpeed:Float;
  public final maximumLinearSpeed:Float;
  public final spiralRadius:Float;
  /** Radial spacing of successive spiral turns; below the hole's capture diameter. */
  public final spiralPitch:Float;
  /** Path speed of the spiral reference while the tool keeps up with it. */
  public final spiralSpeed:Float;
  public final trackingGainPerSecond:Float;
  /** Lateral error inside which the spiral reference advances. */
  public final trackingTolerance:Float;
  /** Depth past a touched surface, or past the CAD entry plane, that shows the screw is in a hole. */
  public final entryAdvance:Float;
  /** Time the spiral may make no progress before the search fails. */
  public final stallSeconds:Float;

  public function new(contactForceN:Float, contactThresholdN:Float, confirmationSamples:Int, approachSpeed:Float,
      maximumLinearSpeed:Float, spiralRadius:Float, spiralPitch:Float, spiralSpeed:Float,
      trackingGainPerSecond:Float, trackingTolerance:Float, entryAdvance:Float, stallSeconds:Float) {
    function positive(value:Float):Bool return Math.isFinite(value) && value > 0;
    if (!positive(contactForceN) || !positive(contactThresholdN) || contactThresholdN >= contactForceN ||
        confirmationSamples < 1 || !positive(approachSpeed) || !positive(maximumLinearSpeed) ||
        approachSpeed > maximumLinearSpeed || !positive(spiralRadius) || !positive(spiralPitch) ||
        spiralPitch > spiralRadius || !positive(spiralSpeed) || spiralSpeed > maximumLinearSpeed ||
        !positive(trackingGainPerSecond) || !positive(trackingTolerance) || !positive(entryAdvance) ||
        !positive(stallSeconds))
      throw "Screw entry needs explicit positive forces, speeds, spiral, entry and stall bounds that agree";
    this.contactForceN = contactForceN;
    this.contactThresholdN = contactThresholdN;
    this.confirmationSamples = confirmationSamples;
    this.approachSpeed = approachSpeed;
    this.maximumLinearSpeed = maximumLinearSpeed;
    this.spiralRadius = spiralRadius;
    this.spiralPitch = spiralPitch;
    this.spiralSpeed = spiralSpeed;
    this.trackingGainPerSecond = trackingGainPerSecond;
    this.trackingTolerance = trackingTolerance;
    this.entryAdvance = entryAdvance;
    this.stallSeconds = stallSeconds;
  }
}

/**
 * Hybrid force/position screw entry, free of clocks, sensors and servos: measured TCP displacement and wrench in,
 * a TCP twist out. The advance axis `n` is force-controlled on a surface (the admittance regulates the compression
 * to the commissioned force) and a guarded descent while unloaded between surfaces; the hole plane (`u`, `v`) is
 * position-controlled.
 *
 * Whenever the screw touches a surface it searches that surface with an Archimedean spiral around the lateral target,
 * whose parameter advances only while the tool tracks it. Dropping `entryAdvance` below the touched depth means a
 * hole: the screw descends again with no lateral target of its own, so the bore it is in centres it. Reaching the
 * CAD entry plane plus `entryAdvance`, touching or not, puts the screw in the registered hole: the tightening
 * handover. Whether a thread is there to engage is the tightening cycle's to measure.
 */
class ScrewEntryController {
  public final limits:ScrewEntryLimits;
  public final axisU:Vec3;
  public final axisV:Vec3;
  /** The advance axis, into the hole. */
  public final axisN:Vec3;
  /** Depth of the CAD entry plane below the approach origin, and its measurement tolerance. */
  public final entryDepth:Float;
  public final depthTolerance:Float;
  final admittance:CartesianAdmittance;
  public var phase(default, null):ScrewEntryPhase = Approach;
  public var failure(default, null):Null<String> = null;
  /** Depth at which the current or last search touched its surface. */
  public var contactDepth(default, null):Null<Float> = null;
  /** Measured compression along the advance axis at the last step. */
  public var compressionN(default, null):Float = 0.0;
  /** Surfaces searched so far: the flange face and the tapped plate are two. */
  public var searches(default, null):Int = 0;
  var targetU = 0.0;
  var targetV = 0.0;
  var theta = 0.0;
  var stalledSeconds = 0.0;
  var contactSamples = 0;

  /** `admittance` works in the search axes (u, v, n, then rotations about them), with a target of `-contactForceN` on n. */
  public function new(limits:ScrewEntryLimits, axisU:Vec3, axisV:Vec3, entryDepth:Float, depthTolerance:Float,
      admittance:CartesianAdmittance) {
    if (limits == null || axisU == null || axisV == null || admittance == null ||
        Math.abs(axisU.norm() - 1.0) > 1e-8 || Math.abs(axisV.norm() - 1.0) > 1e-8 || Math.abs(axisU.dot(axisV)) > 1e-8)
      throw "Screw entry needs its limits, orthonormal hole-plane axes and an admittance";
    if (!Math.isFinite(depthTolerance) || depthTolerance < 0 || depthTolerance >= limits.entryAdvance)
      throw "Screw entry depth tolerance must be below the entry advance, or a flat surface would pass for a hole";
    if (!Math.isFinite(entryDepth) || !(entryDepth > depthTolerance))
      throw "Screw entry plane must lie ahead of the approach origin";
    var target = admittance.target;
    if (target.force.x != 0 || target.force.y != 0 || Math.abs(target.force.z + limits.contactForceN) > 1e-12 ||
        target.torque.norm() != 0)
      throw "Screw entry admittance must regulate only the commissioned axial contact force";
    if (admittance.maxLinearSpeed > limits.maximumLinearSpeed)
      throw "Screw entry admittance exceeds the commissioned linear speed";
    this.limits = limits;
    this.axisU = axisU;
    this.axisV = axisV;
    axisN = axisU.cross(axisV);
    this.entryDepth = entryDepth;
    this.depthTolerance = depthTolerance;
    this.admittance = admittance;
  }

  public function finished():Bool return phase == Seated || phase == Failed;

  /** Radius of the spiral reference; zero outside a search. */
  public function spiralRadius():Float return phase == Search ? spiralScale() * theta : 0.0;

  /**
   * One control step. `displacement` is the measured TCP position relative to the approach origin and `wrench` the
   * environment-on-tool wrench at the TCP, both in the root frame; `dt` is the period since the previous step.
   */
  public function step(displacement:Vec3, wrench:Wrench3, dt:Float):Twist6 {
    if (finished()) return new Twist6(0, 0, 0, 0, 0, 0);
    if (displacement == null || wrench == null || !Math.isFinite(dt) || !(dt > 0))
      throw "Screw entry needs a measured displacement, wrench and positive period";
    var u = displacement.dot(axisU), v = displacement.dot(axisV), depth = displacement.dot(axisN);
    var local = new Wrench3(new Vec3(wrench.force.dot(axisU), wrench.force.dot(axisV), wrench.force.dot(axisN)),
      new Vec3(wrench.torque.dot(axisU), wrench.torque.dot(axisV), wrench.torque.dot(axisN)));
    compressionN = -local.force.z;
    contactSamples = compressionN >= limits.contactThresholdN ? contactSamples + 1 : 0;
    var touching = contactSamples >= limits.confirmationSamples;
    var pastPlane = depth - entryDepth;
    switch (phase) {
      case Approach, Descend:
        if (pastPlane >= limits.entryAdvance - depthTolerance) {
          phase = Seated;
          return new Twist6(0, 0, 0, 0, 0, 0);
        }
        if (touching) {
          phase = Search;
          searches++;
          contactDepth = depth;
          theta = 0.0;
          stalledSeconds = 0.0;
        }
      case Search:
        if (depth >= (contactDepth : Float) + limits.entryAdvance) phase = Descend;
      case Seated, Failed:
    }
    if (phase == Descend) {
      // Only the measured contact moves the screw sideways: its lateral target is wherever it is.
      targetU = u;
      targetV = v;
    }
    var correction = admittance.step(local, dt);
    var lateralU = correction.linearX, lateralV = correction.linearY, axial = correction.linearZ;
    // Unloaded before or between surfaces, the advance axis is a guarded move, and any measured load hands it to the
    // force regulation. On a searched surface it stays regulated: a slip on a chamfer must not become a plunge.
    if (phase != Search && compressionN < limits.contactThresholdN) axial = limits.approachSpeed;
    var referenceU = targetU, referenceV = targetV;
    if (phase == Search) {
      var scale = spiralScale();
      var radius = scale * theta;
      var errorU = targetU + radius * Math.cos(theta) - u, errorV = targetV + radius * Math.sin(theta) - v;
      // A lateral contact force deflects the tool from its reference by design: tracking and admittance settle at
      // force / (damping * gain). Progress is judged on the lag beyond that, so a hole's chamfer pushing the screw
      // towards the hole is not mistaken for a stall. A chamfer pushes sideways by about the regulated axial force;
      // only that much is excused, so a wall pressing harder still stalls the spiral.
      var lateralForce = Math.sqrt(local.force.x * local.force.x + local.force.y * local.force.y);
      var excused = lateralForce > limits.contactForceN ? limits.contactForceN / lateralForce : 1.0;
      var lagU = errorU + excused * local.force.x / (admittance.dampingOf(0) * limits.trackingGainPerSecond);
      var lagV = errorV + excused * local.force.y / (admittance.dampingOf(1) * limits.trackingGainPerSecond);
      if (compressionN < limits.contactThresholdN) {
        // Falling off the touched surface, likely into the hole: hold the reference rather than drive into a wall.
      } else if (Math.sqrt(lagU * lagU + lagV * lagV) <= limits.trackingTolerance) {
        // The reference moves only while the tool keeps up with it.
        var tangent = scale * Math.sqrt(1.0 + theta * theta);
        theta += Math.min(limits.spiralSpeed * dt / tangent, Math.PI / 8.0);
        stalledSeconds = 0.0;
        radius = scale * theta;
        if (radius > limits.spiralRadius)
          return fail('spiral exhausted its ${millimetres(limits.spiralRadius)} mm radius at ' +
            '${millimetres(depth)} mm depth (${millimetres(depth - entryDepth)} mm from the CAD entry plane) ' +
            'without an entry');
        var speed = limits.spiralSpeed;
        var tangentU = (Math.cos(theta) - theta * Math.sin(theta)) / Math.sqrt(1.0 + theta * theta);
        var tangentV = (Math.sin(theta) + theta * Math.cos(theta)) / Math.sqrt(1.0 + theta * theta);
        lateralU += speed * tangentU;
        lateralV += speed * tangentV;
      } else {
        stalledSeconds += dt;
        if (stalledSeconds >= limits.stallSeconds)
          return fail('spiral made no progress for ${limits.stallSeconds} s: ${millimetres(Math.sqrt(lagU * lagU +
            lagV * lagV))} mm behind its reference at radius ${millimetres(radius)} mm, ' +
            '${Math.round(compressionN * 100) / 100} N compression');
      }
      referenceU = targetU + radius * Math.cos(theta);
      referenceV = targetV + radius * Math.sin(theta);
    }
    lateralU += limits.trackingGainPerSecond * (referenceU - u);
    lateralV += limits.trackingGainPerSecond * (referenceV - v);
    var speed = Math.sqrt(lateralU * lateralU + lateralV * lateralV + axial * axial);
    var scale = speed > limits.maximumLinearSpeed ? limits.maximumLinearSpeed / speed : 1.0;
    var linear = axisU.scale(lateralU * scale).add(axisV.scale(lateralV * scale)).add(axisN.scale(axial * scale));
    var angular = axisU.scale(correction.angularX).add(axisV.scale(correction.angularY))
      .add(axisN.scale(correction.angularZ));
    return new Twist6(linear.x, linear.y, linear.z, angular.x, angular.y, angular.z);
  }

  /** Radius gained per radian of spiral angle. */
  inline function spiralScale():Float return limits.spiralPitch / (2.0 * Math.PI);

  function fail(reason:String):Twist6 {
    phase = Failed;
    failure = reason;
    admittance.stop();
    return new Twist6(0, 0, 0, 0, 0, 0);
  }

  static function millimetres(metres:Float):Float return Math.round(metres * 1e6) / 1e3;
}
