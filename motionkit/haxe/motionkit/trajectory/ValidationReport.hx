package motionkit.trajectory;

import trajectorykit.validation.ValidationGuarantee;

import TrajectoryCore;
import haxe.Int64;

/** Snapshot of native extrema and their claimed limits. */
class ValidationReport {
  final native:mk_validation_report;
  public final modelRevision:Int64;
  public final calibrationRevision:Int64;
  public final trajectoryRevision:Int64;
  public final executorTimeResolutionNs:Int64;
  public final checks:Array<ValidationCheck>;
  public final unresolvedAssumptions:Array<String>;
  public final unresolvedAssumptionCount:Int;
  /** Clearance against the cell (planner-owned; see `setCollision`). */
  public var collision(default, null):ValidationCheck;
  /** The pair and segment the clearance check names. */
  public var collisionPair(default, null):CollisionPairRecord;

  public function new(native:mk_validation_report) {
    this.native = native;
    modelRevision = native.get_model_revision();
    calibrationRevision = native.get_calibration_revision();
    trajectoryRevision = native.get_trajectory_revision();
    executorTimeResolutionNs = native.get_executor_time_resolution_ns();
    unresolvedAssumptionCount = native.get_assumption_count();
    unresolvedAssumptions = [];
    for (index in 0...unresolvedAssumptionCount) {
      var assumption = native.get_assumptions(index);
      var text = new StringBuf();
      for (letter in 0...TrajectoryCoreConstants.MK_ASSUMPTION_LENGTH) {
        var code = assumption.get_text(letter);
        if (code == 0) break;
        text.addChar(code);
      }
      unresolvedAssumptions.push(text.toString());
    }
    checks = [];
    for (index in 0...TrajectoryCoreConstants.MK_CHECK_COUNT) {
      var check = native.get_checks(index);
      checks.push(ValidationCheck.fromNative(check));
    }
    collision = ValidationCheck.fromNative(native.get_collision());
    collisionPair = CollisionPairRecord.fromNative(native.get_collision_pair());
  }

  /**
   * Records the planner's clearance check (COLLISION.md CL4b): a continuous
   * bound (`MK_CHECK_METHOD_BOUND`, zero resolution) or a sampled check, the
   * distance and required clearance of the named pair, when, and over which
   * segment. Object ids are -1 when the world names bodies only.
   */
  public function setCollision(status:Int, method:Int, distance:Float, required:Float, timeSeconds:Float,
      segmentStart:Float, segmentEnd:Float, resolutionNs:Int64, nameA:String, nameB:String, objectA:Int = -1,
      objectB:Int = -1):Void {
    var result = TrajectoryCore.mk_report_set_collision(native, status, method, distance, required, timeSeconds,
      segmentStart, segmentEnd, resolutionNs, objectA, objectB, utf8(nameA), utf8(nameB));
    if (result != TrajectoryCoreConstants.MK_OK)
      throw 'validationReport.setCollision failed with MotionKit error $result';
    collision = ValidationCheck.fromNative(native.get_collision());
    collisionPair = CollisionPairRecord.fromNative(native.get_collision_pair());
  }

  static function utf8(text:String):haxe.io.Bytes return haxe.io.Bytes.ofString(text == null ? "" : text);

  /** Records a task-space check sampled by the planner's kinematics layer. */
  public function setTaskSpace(status:Int, worst:Float, timeSeconds:Float,
      tolerance:Float, resolutionNs:Int64):Void {
    var result = TrajectoryCore.mk_report_set_task_space(native, status, worst,
      timeSeconds, tolerance, resolutionNs);
    if (result != TrajectoryCoreConstants.MK_OK)
      throw 'validationReport.setTaskSpace failed with MotionKit error $result';
    checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE] = ValidationCheck.fromNative(
      native.get_checks(TrajectoryCoreConstants.MK_CHECK_TASK_SPACE));
  }

  /** A continuous whole-path upper bound, with no sampled peak timestamp.
   * The compiler must own the geometric/lowering certificate. */
  public function setTaskSpaceBound(upperBound:Float,tolerance:Float):Void {
    var result=TrajectoryCore.mk_report_set_task_space_bound(native,upperBound,tolerance);
    if(result!=TrajectoryCoreConstants.MK_OK)throw 'validationReport.setTaskSpaceBound failed with MotionKit error $result';
    checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE]=ValidationCheck.fromNative(
      native.get_checks(TrajectoryCoreConstants.MK_CHECK_TASK_SPACE));
  }

  public function hasFailure():Bool {
    for (check in checks)
      if (check.status == TrajectoryCoreConstants.MK_CHECK_FAILED) return true;
    return collision.status == TrajectoryCoreConstants.MK_CHECK_FAILED;
  }

  /** Summarizes each check without treating sampled coverage as a proof. */
  public function guarantees():ValidationGuarantees {
    return new ValidationGuarantees(
      guarantee(TrajectoryCoreConstants.MK_CHECK_POSITION),
      guarantee(TrajectoryCoreConstants.MK_CHECK_VELOCITY),
      guarantee(TrajectoryCoreConstants.MK_CHECK_ACCELERATION),
      guarantee(TrajectoryCoreConstants.MK_CHECK_JERK),
      guarantee(TrajectoryCoreConstants.MK_CHECK_CONTINUITY),
      guarantee(TrajectoryCoreConstants.MK_CHECK_TASK_SPACE), guaranteeOf(collision));
  }

  function guarantee(index:Int):ValidationGuarantee return guaranteeOf(checks[index]);

  static function guaranteeOf(check:ValidationCheck):ValidationGuarantee {
    if (check.status == TrajectoryCoreConstants.MK_CHECK_FAILED) return Failed;
    if (check.status != TrajectoryCoreConstants.MK_CHECK_PASSED) return Unchecked;
    return check.method == TrajectoryCoreConstants.MK_CHECK_METHOD_SAMPLED ?
      Sampled(check.resolutionNs) : Proven;
  }
}

class ValidationCheck {
  public final status:Int;
  public final joint:Int;
  public final derivativeOrder:Int;
  public final method:Int;
  public final value:Float;
  public final timeSeconds:Float;
  public final limit:Float;
  public final margin:Float;
  public final tolerance:Float;
  public final resolutionNs:Int64;

  public function new(status:Int, joint:Int, derivativeOrder:Int, method:Int,
      value:Float, timeSeconds:Float, limit:Float, margin:Float, tolerance:Float,
      resolutionNs:Int64) {
    this.status = status;
    this.joint = joint;
    this.derivativeOrder = derivativeOrder;
    this.method = method;
    this.value = value;
    this.timeSeconds = timeSeconds;
    this.limit = limit;
    this.margin = margin;
    this.tolerance = tolerance;
    this.resolutionNs = resolutionNs;
  }

  public static function fromNative(native:mk_validation_check):ValidationCheck
    return new ValidationCheck(native.get_status(), native.get_joint(),
      native.get_derivative_order(), native.get_method(), native.get_value(),
      native.get_time_seconds(), native.get_limit(), native.get_margin(),
      native.get_tolerance(), native.get_resolution_ns());
}

/** The pair and segment a clearance check names; object ids are -1 when unknown. */
class CollisionPairRecord {
  public final objectA:Int;
  public final objectB:Int;
  public final segmentStart:Float;
  public final segmentEnd:Float;
  public final nameA:String;
  public final nameB:String;

  public function new(objectA:Int, objectB:Int, segmentStart:Float, segmentEnd:Float, nameA:String, nameB:String) {
    this.objectA = objectA;
    this.objectB = objectB;
    this.segmentStart = segmentStart;
    this.segmentEnd = segmentEnd;
    this.nameA = nameA;
    this.nameB = nameB;
  }

  public static function fromNative(native:mk_collision_pair):CollisionPairRecord {
    function text(read:Int->Int):String {
      var bytes = new haxe.io.BytesBuffer();
      for (index in 0...TrajectoryCoreConstants.MK_ASSUMPTION_LENGTH) {
        var code = read(index);
        if (code == 0) break;
        bytes.addByte(code & 0xff);
      }
      return bytes.getBytes().toString();
    }
    var a:Int = native.get_object_a(), b:Int = native.get_object_b();
    return new CollisionPairRecord(a == -1 ? -1 : a, b == -1 ? -1 : b, native.get_segment_start(),
      native.get_segment_end(), text(i -> native.get_name_a(i)), text(i -> native.get_name_b(i)));
  }
}
