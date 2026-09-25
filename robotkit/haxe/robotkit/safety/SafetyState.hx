package robotkit.safety;

import haxe.Int64;
import robotkit.mobile.Footprint;
import robotkit.mobile.MotionLimits;

/** Immutable operator-facing state and restrictions reported by safety policy. */
class SafetyState {
  public final phase:SafetyPhase;
  public final speedLimitMetersPerSecond:Float;
  public final stoppingEnvelope:StoppingEnvelope;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final sourceClockId:String;
  public final receivedClockId:String;
  /** Dynamic user-level motion cap when this state comes from a policy. */
  public final effectiveMotionLimits:Null<MotionLimits>;
  /** Conservative planar robot-and-load footprint when geometry is known. */
  public final footprint:Null<Footprint>;
  final values:Array<SafetyRestriction>;

  public function new(phase:SafetyPhase, speedLimitMetersPerSecond:Float,
      stoppingEnvelope:StoppingEnvelope, restrictions:Array<SafetyRestriction>,
      sourceTimestampNs:Int64, receivedTimestampNs:Int64,
      sourceClockId:String, receivedClockId:String,
      ?effectiveMotionLimits:MotionLimits, ?footprint:Footprint) {
    if (phase == null || stoppingEnvelope == null ||
        !Math.isFinite(speedLimitMetersPerSecond) || speedLimitMetersPerSecond < 0.0 ||
        sourceClockId == null || sourceClockId.length == 0 ||
        receivedClockId == null || receivedClockId.length == 0)
      throw "Safety state requires phase, speed limit, stopping envelope, and clock IDs";
    this.phase = phase;
    this.speedLimitMetersPerSecond = speedLimitMetersPerSecond;
    this.stoppingEnvelope = stoppingEnvelope;
    values = restrictions == null ? [] : restrictions.copy();
    for (value in values) if (value == null) throw "Safety restrictions cannot be null";
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs;
    this.sourceClockId = sourceClockId;
    this.receivedClockId = receivedClockId;
    this.effectiveMotionLimits = effectiveMotionLimits;
    this.footprint = footprint;
  }

  public function restrictions():Array<SafetyRestriction> return values.copy();
}
