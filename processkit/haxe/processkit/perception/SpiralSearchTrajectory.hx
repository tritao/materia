package processkit.perception;

import robotkit.spatial.Vec3;

/** One sample of a bounded Archimedean search in the registered hole plane. */
class SpiralSearchPoint {
  public final offset:Vec3;
  public final velocity:Vec3;
  public final complete:Bool;

  public function new(offset:Vec3, velocity:Vec3, complete:Bool) {
    this.offset = offset;
    this.velocity = velocity;
    this.complete = complete;
  }
}

/**
 * Bounded lateral search trajectory. The commissioned radius must fit inside
 * both axes of the registration region, and its conservative speed bound must
 * fit the explicit lateral speed limit.
 */
class SpiralSearchTrajectory {
  public final axisU:Vec3;
  public final axisV:Vec3;
  public final region:ContactRegionBounds;
  /** Initial lateral preload target, measured from the nominal TCP approach. */
  public final contactSeedOffsetUMeters:Float;
  public final maxRadius:Float;
  public final turns:Float;
  public final durationSeconds:Float;
  public final maxLateralSpeed:Float;
  /** Starting radius; equal to maxRadius produces a bounded circumferential sweep. */
  public final startRadius:Float;
  /** Decreases radius from maxRadius to the centre when refining a located candidate. */
  public final inward:Bool;
  var lastTime = -1.0;

  public function new(axisU:Vec3, axisV:Vec3, region:ContactRegionBounds, maxRadius:Float,
    turns:Float, durationSeconds:Float, maxLateralSpeed:Float, contactSeedOffsetUMeters:Float = 0,
    startRadius:Float = 0, inward:Bool = false) {
    if (axisU == null || axisV == null || region == null || !Math.isFinite(axisU.norm()) ||
      !Math.isFinite(axisV.norm()) || Math.abs(axisU.norm() - 1.0) > 1e-8 ||
      Math.abs(axisV.norm() - 1.0) > 1e-8 || Math.abs(axisU.dot(axisV)) > 1e-8 ||
      !Math.isFinite(maxRadius) || !(maxRadius > 0) || maxRadius > region.halfU || maxRadius > region.halfV ||
      !Math.isFinite(contactSeedOffsetUMeters) || Math.abs(contactSeedOffsetUMeters) + maxRadius > region.halfU ||
      !Math.isFinite(startRadius) || startRadius < 0 || startRadius > maxRadius ||
      !Math.isFinite(turns) || !(turns > 0) || !Math.isFinite(durationSeconds) || !(durationSeconds > 0) ||
      !Math.isFinite(maxLateralSpeed) || !(maxLateralSpeed > 0))
      throw "Spiral search needs orthogonal plane axes, a matching registration region and explicit positive bounds";
    var radialSpeed = inward ? -startRadius / durationSeconds : (maxRadius - startRadius) / durationSeconds;
    var angularSpeed = 2.0 * Math.PI * turns / durationSeconds;
		if (Math.abs(radialSpeed) + maxRadius * angularSpeed > maxLateralSpeed + 1e-12)
      throw "Spiral search exceeds its commissioned lateral speed limit";
    this.axisU = axisU;
    this.axisV = axisV;
    this.region = region;
    this.contactSeedOffsetUMeters = contactSeedOffsetUMeters;
    this.maxRadius = maxRadius;
    this.turns = turns;
    this.durationSeconds = durationSeconds;
    this.maxLateralSpeed = maxLateralSpeed;
    this.startRadius = startRadius;
    this.inward = inward;
  }

  /** Times must be monotonic; after the commissioned duration the endpoint is held with zero lateral speed. */
  public function sample(timeSeconds:Float):SpiralSearchPoint {
    if (!Math.isFinite(timeSeconds) || timeSeconds < 0 || timeSeconds < lastTime)
      throw "Spiral search time must be finite, non-negative and monotonic";
    lastTime = timeSeconds;
    var time = Math.min(timeSeconds, durationSeconds);
    var complete = time >= durationSeconds;
    var progress = time / durationSeconds;
    var radialSpeed = inward ? -startRadius / durationSeconds : (maxRadius - startRadius) / durationSeconds;
    var radius = inward ? startRadius * (1.0 - progress) : startRadius + (maxRadius - startRadius) * progress;
    var angle = 2.0 * Math.PI * turns * progress;
    var cosine = Math.cos(angle), sine = Math.sin(angle);
    var offset = axisU.scale(radius * cosine).add(axisV.scale(radius * sine));
    if (complete) return new SpiralSearchPoint(offset, Vec3.zero(), true);
    var angularSpeed = 2.0 * Math.PI * turns / durationSeconds;
    var velocity = axisU.scale(radialSpeed * cosine - radius * angularSpeed * sine)
      .add(axisV.scale(radialSpeed * sine + radius * angularSpeed * cosine));
    if (velocity.norm() > maxLateralSpeed + 1e-12)
      throw "Spiral search exceeded its validated lateral speed bound";
    return new SpiralSearchPoint(offset, velocity, false);
  }
}
