package robotkit.world;
import trajectorykit.validation.ValidationGuarantee;
/** Limits and contracts of buffered execution; zero limits mean no plan support. */
class ExecutionCapabilities {
  public final plans:Bool;
  public final maximumPolynomialDegree:Int;
  public final maximumJoints:Int;
  public final maximumSegments:Int;
  public final timedEvents:Bool;
  public final replacementBoundaries:Bool;
  public final holdResume:Bool;
  /** Validation of declared position, velocity and acceleration limits.
      Jerk remains a separate plan claim, represented by jerkUnchecked. */
  public final polynomialLimits:ValidationGuarantee;
  public function new(plans:Bool, maximumPolynomialDegree:Int, maximumJoints:Int,
      maximumSegments:Int, timedEvents:Bool, replacementBoundaries:Bool,
      holdResume:Bool, polynomialLimits:ValidationGuarantee) {
    if (maximumPolynomialDegree < 0 || maximumJoints < 0 || maximumSegments < 0 ||
        (plans && (maximumJoints == 0 || maximumSegments == 0)) ||
        (!plans && (maximumPolynomialDegree != 0 || maximumJoints != 0 ||
          maximumSegments != 0 || timedEvents || replacementBoundaries || holdResume)))
      throw "Invalid execution capabilities";
    this.plans = plans;
    this.maximumPolynomialDegree = maximumPolynomialDegree;
    this.maximumJoints = maximumJoints;
    this.maximumSegments = maximumSegments;
    this.timedEvents = timedEvents;
    this.replacementBoundaries = replacementBoundaries;
    this.holdResume = holdResume;
    this.polynomialLimits = polynomialLimits;
  }
  public static function unavailable():ExecutionCapabilities
    return new ExecutionCapabilities(false, 0, 0, 0, false, false, false, Unchecked);
}
