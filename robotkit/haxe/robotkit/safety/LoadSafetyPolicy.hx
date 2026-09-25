package robotkit.safety;

import robotkit.material.ForkState;
import robotkit.material.Forks;
import robotkit.material.LoadState;
import robotkit.mobile.Footprint;
import robotkit.mobile.FootprintPoint;
import robotkit.mobile.MobileBase;
import robotkit.mobile.MotionLimits;
import robotkit.safety.SafetyRestriction;
import robotkit.world.StopMode;

/** Applies load- and fork-height-dependent soft limits to one mobile forklift. */
class LoadSafetyPolicy implements Safety {
  public final base:MobileBase;
  public final forks:Forks;
  public final configuration:LoadSafetyConfiguration;
  var currentState:Null<SafetyState> = null;

  public function new(base:MobileBase, forks:Forks,
      ?configuration:LoadSafetyConfiguration) {
    if (base == null || forks == null || base.robot.id() != forks.robot.id())
      throw "Load safety requires a MobileBase and Forks view over the same robot";
    this.base = base;
    this.forks = forks;
    this.configuration = configuration == null
      ? new LoadSafetyConfiguration()
      : configuration;
    refresh();
  }

  /** Reads the latest fork and load state, updates MobileBase caps, and returns a snapshot. */
  public function refresh():SafetyState {
    var forkState:ForkState = forks.state();
    var load:LoadState = forks.loadState;
    var limits = forks.config.loadLimits;
    var liftHeight = forkState.lift.position;
    var massRatio = load.payload == null ? 0.0 : clamp01(
      load.payload.massKg / limits.maxMassKg);
    var liftRatio = raisedLiftRatio(liftHeight, limits.maxLiftHeightMeters,
      configuration.raisedForkThresholdMeters);
    var speedScale = load.observed ? 1.0 : configuration.unknownLoadSpeedScale;
    var accelerationScale = load.observed
      ? 1.0
      : configuration.unknownLoadAccelerationScale;
    speedScale *= 1.0 - configuration.payloadSpeedReduction * massRatio;
    speedScale *= 1.0 - configuration.raisedForkSpeedReduction * liftRatio;
    accelerationScale *= 1.0 - configuration.payloadAccelerationReduction * massRatio;
    accelerationScale *= 1.0 - configuration.raisedForkAccelerationReduction * liftRatio;

    var violation = limits.violation(load.payload, liftHeight);
    var speedViolation = load.payload != null && load.payload.massKg > limits.maxMassKg;
    var momentViolation = load.payload != null &&
      load.payload.massKg * load.payload.centerOfMassForwardMeters >
        limits.maxLoadMomentKgMeters;
    var heightViolation = liftHeight > limits.maxLiftHeightMeters;
    if (violation != null) {
      speedScale = Math.min(speedScale, 0.1);
      accelerationScale = Math.min(accelerationScale, 0.1);
    }

    var nominal:MotionLimits = base.nominalMotionLimits;
    var effective = new MotionLimits(nominal.maxLinearSpeed * speedScale,
      nominal.maxAngularSpeed * speedScale,
      nominal.maxLinearAcceleration * accelerationScale,
      nominal.maxAngularAcceleration * accelerationScale);
    var movingSpeed = Math.abs(base.currentCommand().linear);
    base.applySafetyLimits(effective);
    var wasStopped = base.safetyStopRequired;
    base.applySafetyStop(violation != null);
    if (violation != null && !wasStopped) base.stop(StopMode.Normal);

    var restrictions:Array<SafetyRestriction> = [];
    if (!load.observed) restrictions.push(LoadStateUnknown);
    if (speedViolation || momentViolation)
      restrictions.push(PayloadLimited(limits.maxMassKg));
    if (heightViolation || liftHeight > configuration.raisedForkThresholdMeters)
      restrictions.push(ForkHeightLimited(limits.maxLiftHeightMeters));
    if (speedScale < 0.999)
      restrictions.push(SpeedLimited(violation == null ? effective.maxLinearSpeed : 0.0));
    if (violation != null) restrictions.push(StopRequired(violation));

    var phase = violation != null ? ProtectiveStop :
      (restrictions.length == 0 ? Normal : Restricted);
    var stopping = new StoppingEnvelope(movingSpeed,
      configuration.reactionTimeSeconds, effective.maxLinearAcceleration);
    currentState = new SafetyState(phase,
      violation == null ? effective.maxLinearSpeed : 0.0,
      stopping, restrictions, forkState.sourceTimestampNs,
      forkState.receivedTimestampNs, forkState.sourceClockId,
      forkState.receivedClockId, effective,
      loadFootprint(base.footprint, load, configuration.unknownPayloadMarginMeters));
    return currentState;
  }

  /** Returns the last policy result; call refresh at the application's control frequency. */
  public function state():SafetyState {
    if (currentState == null) return refresh();
    return cast currentState;
  }

  static function raisedLiftRatio(height:Float, maximum:Float, threshold:Float):Float {
    if (height <= threshold) return 0.0;
    if (maximum <= threshold) return 1.0;
    return clamp01((height - threshold) / (maximum - threshold));
  }

  static function clamp01(value:Float):Float return Math.max(0.0, Math.min(1.0, value));

  static function loadFootprint(baseFootprint:Null<Footprint>, load:LoadState,
      unknownMargin:Float):Null<Footprint> {
    if (load.payload == null && load.observed) return baseFootprint;
    if (baseFootprint == null && load.payload == null) return null;
    var minX = 1.0e300;
    var maxX = -1.0e300;
    var minY = 1.0e300;
    var maxY = -1.0e300;
    if (baseFootprint != null) {
      for (point in baseFootprint.vertices()) {
        minX = Math.min(minX, point.x);
        maxX = Math.max(maxX, point.x);
        minY = Math.min(minY, point.y);
        maxY = Math.max(maxY, point.y);
      }
    }
    if (!load.observed) {
      minX -= unknownMargin;
      maxX += unknownMargin;
      minY -= unknownMargin;
      maxY += unknownMargin;
    }
    if (load.payload != null) {
      var payload = load.payload;
      minX = Math.min(minX, payload.centerOfMassForwardMeters - payload.lengthMeters * 0.5);
      maxX = Math.max(maxX, payload.centerOfMassForwardMeters + payload.lengthMeters * 0.5);
      minY = Math.min(minY, payload.centerOfMassLateralMeters - payload.widthMeters * 0.5);
      maxY = Math.max(maxY, payload.centerOfMassLateralMeters + payload.widthMeters * 0.5);
    }
    return new Footprint([
      new FootprintPoint(minX, minY), new FootprintPoint(maxX, minY),
      new FootprintPoint(maxX, maxY), new FootprintPoint(minX, maxY)
    ]);
  }
}
