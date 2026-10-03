package motionkit.trajectory;

/** What a plan check found wrong with a plan, as a name that stays the same in logs and tests. */
enum abstract PlanDiagnosticKind(String) from String to String {
  /** A stepper is asked for more torque than its pull-out curve gives, so it would lose steps. */
  var StepperStall = "stepper-stall";
  /** A servo is asked for more than its peak torque (or more speed than it has torque for). */
  var ServoPeakTorque = "servo-peak-torque";
  /** A servo's torque over a plan, as a root mean square, is above its rated torque. */
  var ServoRatedTorque = "servo-rated-torque";
  /** The axis's drive stretches or lags by more than the tolerance. */
  var Accuracy = "accuracy";

}

/**
 * One finding of a plan check: which op of the program, which motor or axis, where in the plan,
 * and how far over its limit. A check reports its worst sample for each motor and kind, with how
 * many samples were over.
 */
class PlanDiagnostic {
  /** Quantities used by each finding's calculation. */
  public static function quantities(kind:PlanDiagnosticKind):Array<String> return switch kind {
    case PlanDiagnosticKind.Accuracy: ["stiffness", "backlash", "drag", "steady loads"];
    case _: ["motor curve", "inertia", "efficiency", "drag"];
  };

  public final kind:PlanDiagnosticKind;
  /** The program op the plan came from, -1 when unknown. */
  public final opIndex:Int;
  /** The actuator that is over its limit, or the axis for an accuracy finding. */
  public final subject:String;
  /** The axis joint the motor drives, or the axis itself. */
  public final axis:String;
  /** Time from the start of the plan at the worst sample, in seconds. */
  public final timeSeconds:Float;
  /** Distance along the op's path at the worst sample, when the plan knows its path; -1 otherwise. */
  public var pathDistance:Float = -1.0;
  /** The needed torque (N m) or deviation (m) at the worst sample. */
  public final value:Float;
  /** What the drive can give there (N m), or the tolerance (m). */
  public final limit:Float;
  /** How many samples of the plan were over. */
  public final samples:Int;
  /** Assumed model inputs behind this finding. */
  public var assumed:Array<String> = [];

  public function new(kind:PlanDiagnosticKind, opIndex:Int, subject:String, axis:String, timeSeconds:Float,
      value:Float, limit:Float, samples:Int) {
    this.kind = kind;
    this.opIndex = opIndex;
    this.subject = subject;
    this.axis = axis;
    this.timeSeconds = timeSeconds;
    this.value = value;
    this.limit = limit;
    this.samples = samples;
  }

  /** How much of its limit the sample asks for: 1.2 is 20% over. */
  public function ratio():Float return limit > 0.0 ? value / limit : Math.POSITIVE_INFINITY;

  /** Plain-sentence description; `line` is the source line of the op when the caller knows it, else 0. */
  public function describe(line:Int = 0):String {
    var where = (line > 0 ? 'line $line, ' : '') + 'op $opIndex' + (pathDistance >= 0.0 ? ' at ${round(pathDistance * 1000.0)} mm' : '') +
      ' (${round(timeSeconds)} s into the plan)';
    var description = switch kind {
      case Accuracy: '$where: axis $axis deviates ${round(value * 1000.0)} mm against a tolerance of ${round(limit * 1000.0)} mm';
      case ServoRatedTorque: '$where: motor $subject on $axis runs at ${round(value)} N m RMS against ${round(limit)} N m rated';
      case _: '$where: motor $subject on $axis needs ${round(value)} N m against ${round(limit)} N m available' +
        ' (${Std.int(Math.round(100.0 * (ratio() - 1.0)))}% over, $samples samples)';
    };
    return description + (assumed.length == 0 ? "" : ' (assumed: ${assumed.join(", ")})');
  }

  public function toString():String return '${kind}: ' + describe();

  static function round(value:Float):Float return Math.round(value * 1000.0) / 1000.0;
}

/**
 * How far an axis falls behind its command in a plan because its stepper motors lost sync: the
 * distance lost along the axis at the plan times where it was losing (signed along the motion,
 * constant between those times' ends), and the whole-plan total as full steps of the first motor.
 */
class PlanSlip {
  public final axis:String;
  /** The actuators that lost sync together, all of the axis's motors. */
  public final motors:Array<String>;
  /** Plan times (s) and the cumulative distance lost then, in the axis's units; both rise together. */
  public final times:Array<Float>;
  public final lost:Array<Float>;
  /** Full steps of the first motor the plan loses, a positive count. */
  public final steps:Float;

  public function new(axis:String, motors:Array<String>, times:Array<Float>, lost:Array<Float>, steps:Float) {
    this.axis = axis;
    this.motors = motors;
    this.times = times;
    this.lost = lost;
    this.steps = steps;
  }

  /** The distance lost by plan time `time`: nothing before the first loss, the total after the last. */
  public function lostAt(time:Float):Float {
    if (times.length == 0 || time <= times[0]) return 0.0;
    var last = times.length - 1;
    if (time >= times[last]) return lost[last];
    for (index in 1...times.length) if (time <= times[index]) {
      var span = times[index] - times[index - 1];
      return span > 0.0 ? lost[index - 1] + (time - times[index - 1]) / span * (lost[index] - lost[index - 1]) : lost[index];
    }
    return lost[last];
  }

  /** The distance the whole plan loses. */
  public function total():Float return lost.length == 0 ? 0.0 : lost[lost.length - 1];
}

/** What checking one plan found: its diagnostics and how near the limits it came. */
class PlanCheckResult {
  public final diagnostics:Array<PlanDiagnostic>;
  /** Active hardware speed ceilings; these explain planning limits rather than flagging violations. */
  public var speedLimits:Array<String> = [];
  /** The largest torque a motor needed over what its drive gives at that speed (1 is exactly at the limit); 0 when no motor is checked. */
  public final worstTorqueRatio:Float;
  public final worstMotor:String;
  /** The largest deviation an axis's drive allows, in metres (radians for a turning axis). */
  public final worstDeviation:Float;
  public final worstAxis:String;
  /** Where steppers over their curve would lose sync, for a simulation to carry out; empty when none would. */
  public final slips:Array<PlanSlip>;

  public function new(diagnostics:Array<PlanDiagnostic>, worstTorqueRatio:Float, worstMotor:String,
      worstDeviation:Float, worstAxis:String, ?slips:Array<PlanSlip>) {
    this.slips = slips == null ? [] : slips;
    this.diagnostics = diagnostics;
    this.worstTorqueRatio = worstTorqueRatio;
    this.worstMotor = worstMotor;
    this.worstDeviation = worstDeviation;
    this.worstAxis = worstAxis;
  }

  /** Fills each diagnostic's distance along the op's path from the plan's path timeline (times and distances alike). */
  public function locate(times:Array<Float>, distances:Array<Float>):Void {
    if (times.length < 2 || times.length != distances.length) return;
    for (diagnostic in diagnostics) {
      var time = diagnostic.timeSeconds;
      var found = distances[distances.length - 1];
      for (index in 1...times.length) if (time <= times[index]) {
        var span = times[index] - times[index - 1];
        var along = span > 0.0 ? (time - times[index - 1]) / span : 0.0;
        found = distances[index - 1] + along * (distances[index] - distances[index - 1]);
        break;
      }
      diagnostic.pathDistance = found;
    }
  }
}
