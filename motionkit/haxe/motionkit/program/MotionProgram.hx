package motionkit.program;

import motionkit.event.EventValueTools;

/** Ordered motion request with structural validation and a copied operation list. */
class MotionProgram {
  public final ops:Array<MotionOp>;

  public function new(ops:Array<MotionOp>) {
    var error = validate(ops);
    if (error != null) throw error;
    this.ops = ops.copy();
  }

  /** Returns the first structural error, including the operation index. */
  public static function validate(ops:Array<MotionOp>):Null<String> {
    if (ops == null || ops.length == 0)
      return "Motion program needs at least one operation";
    for (index in 0...ops.length) {
      var op = ops[index];
      if (op == null) return 'Motion program op $index is null';
      var error = validateOp(op, index, ops);
      if (error != null) return error;
    }
    return null;
  }

  static function validateOp(op:MotionOp, index:Int,
      ops:Array<MotionOp>):Null<String> {
    return switch op {
      case MoveJ(target, limits, blend):
        var targetError = validateTarget(target, index);
        if (targetError != null) targetError;
        else if (limits == null) 'Motion program op $index needs motion limits';
        else validateBlend(blend, index, ops);
      case MoveL(pose, frameId, feed, blend):
        if (pose == null) 'Motion program op $index needs a line endpoint pose';
        else if (!hasId(frameId)) 'Motion program op $index needs a frame ID';
        else if (!validPositive(feed)) 'Motion program op $index feed must be finite and positive';
        else validateBlend(blend, index, ops);
      case MoveC(via, end, frameId, feed, blend):
        if (via == null || end == null) 'Motion program op $index needs circular via and end poses';
        else if (!hasId(frameId)) 'Motion program op $index needs a frame ID';
        else if (!validPositive(feed)) 'Motion program op $index feed must be finite and positive';
        else validateBlend(blend, index, ops);
      case FollowPath(path, frameId, timing, events):
        if (path == null) 'Motion program op $index needs an authored path';
        else if (!hasId(frameId)) 'Motion program op $index needs a frame ID';
        else if (!validPositive(timing))
          'Motion program op $index path feed timing must be finite and positive';
        else validateEvents(events, path.length(), index);
      case Dwell(seconds):
        if (!validPositive(seconds))
          'Motion program op $index dwell must be finite and positive';
        else null;
      case SetOutput(channel, value):
        if (!hasId(channel)) 'Motion program op $index needs an output channel ID';
        else valueError(value, 'Motion program op $index output value');
      case WaitInput(channel, predicate, timeoutSeconds):
        if (!hasId(channel)) 'Motion program op $index needs an input channel ID';
        else if (predicate == null) 'Motion program op $index needs an input predicate';
        else if (!validPositive(timeoutSeconds))
          'Motion program op $index timeout must be finite and positive';
        else validatePredicate(predicate, index);
    };
  }

  static function validateTarget(target:MoveTarget, index:Int):Null<String> {
    if (target == null) return 'Motion program op $index needs a move target';
    return switch target {
      case JointTarget(joints):
        if (joints == null || joints.length == 0)
          'Motion program op $index joint target must be non-empty';
        else if (!allFinite(joints))
          'Motion program op $index joint target must be finite';
        else null;
      case PoseTarget(pose, frameId, configurationHint):
        if (pose == null) 'Motion program op $index needs a target pose';
        else if (!hasId(frameId)) 'Motion program op $index pose target needs a frame ID';
        else if (configurationHint != null &&
            (configurationHint.length == 0 || !allFinite(configurationHint)))
          'Motion program op $index configuration hint must be non-empty and finite';
        else null;
    };
  }

  static function validateBlend(blend:Blend, index:Int,
      ops:Array<MotionOp>):Null<String> {
    if (blend == null) return 'Motion program op $index needs a blend policy';
    return switch blend {
      case ExactStop: null;
      case ToleranceBlend(metres):
        if (!validPositive(metres))
          'Motion program op $index blend tolerance must be finite and positive';
        else if (index + 1 >= ops.length || !isMove(ops[index + 1]))
          'Motion program op $index tolerance blend requires consecutive moves';
        else null;
    };
  }

  static function validateEvents(events:Array<motionkit.event.PathEvent>,
      pathLength:Float, index:Int):Null<String> {
    if (!Math.isFinite(pathLength) || pathLength < 0.0)
      return 'Motion program op $index path length must be finite and non-negative';
    if (events == null) return 'Motion program op $index needs a path-event list';
    var previous = -1.0;
    for (event in events) {
      if (event == null) return 'Motion program op $index contains a null path event';
      if (event.distance < previous)
        return 'Motion program op $index path events must be sorted by distance';
      if (event.distance > pathLength)
        return 'Motion program op $index path event lies beyond the path length';
      previous = event.distance;
    }
    return null;
  }

  static function validatePredicate(predicate:InputPredicate,
      index:Int):Null<String> return switch predicate {
    case Equals(value): valueError(value, 'Motion program op $index input predicate');
  };

  static function valueError(value:motionkit.event.EventValue,
      label:String):Null<String> {
    try {
      EventValueTools.validate(value, label);
      return null;
    } catch (error:Dynamic) {
      return Std.string(error);
    }
  }

  static function isMove(op:MotionOp):Bool {
    if (op == null) return false;
    return switch op {
      case MoveJ(_, _, _), MoveL(_, _, _, _), MoveC(_, _, _, _, _),
          FollowPath(_, _, _, _): true;
      case _: false;
    };
  }

  static function hasId(value:String):Bool
    return value != null && StringTools.trim(value).length > 0;

  static function validPositive(value:Float):Bool
    return Math.isFinite(value) && value > 0.0;

  static function allFinite(values:Array<Float>):Bool {
    for (value in values) if (!Math.isFinite(value)) return false;
    return true;
  }
}
