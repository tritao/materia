package robotkit.world;
import trajectorykit.validation.ValidationGuarantee;
/** Deadline enforcement and mapping to the endpoint's explicit source clock. */
class TimingCapabilities {
  public final deadlines:Bool;
  public final clockMapping:Bool;
  public final prediction:ValidationGuarantee;
  public function new(deadlines:Bool, clockMapping:Bool, prediction:ValidationGuarantee) {
    this.deadlines = deadlines;
    this.clockMapping = clockMapping;
    this.prediction = prediction;
  }
}
