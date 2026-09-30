package kinematicskit;

/**
 * A soft cost on moving a root (see `KinematicProblem.setRootMotion`): rows
 * `weight · v` and `angularWeight · ω` with zero target. It makes a solve
 * prefer joint motion and move the base only for what the joints cannot
 * reach, e.g. a mobile manipulator driving only when its arm runs out.
 * Contributes nothing when the root does not move in the solve.
 */
class RootDampingTask implements KinematicTask {
  public final body:Int;
  public var linearWeight:Float;
  public var angularWeight:Float;

  public function new(model:KinematicModel, body:Int, linearWeight:Float, ?angularWeight:Float) {
    if (model == null || body < 0 || body >= model.bodyCount() || model.bodyParentJoint[body] >= 0)
      throw "Root damping task requires a root body of the model";
    if (!(linearWeight >= 0.0) || (angularWeight != null && !(angularWeight >= 0.0)))
      throw "Root damping weights must be non-negative";
    this.body = body;
    this.linearWeight = linearWeight;
    this.angularWeight = angularWeight == null ? linearWeight : angularWeight;
  }

  public function label():String return "root damping";
  /** Six rows whatever the mode; unused rows are zero. */
  public function rowCount():Int return 6;
  public function isSoft():Bool return true;
  public function positionError():Float return 0.0;
  public function orientationError():Float return 0.0;
  public function satisfied():Bool return true;

  public function evaluate(state:KinematicState, snapshot:KinematicSnapshot, layout:JacobianLayout,
      residual:Array<Float>, jacobian:Array<Float>, row:Int):Void {
    var w = layout.width;
    for (r in 0...6) {
      residual[row + r] = 0.0;
      for (c in 0...w) jacobian[(row + r) * w + c] = 0.0;
    }
    var block = layout.blockOfRoot[body];
    if (block < 0) return;
    var c = layout.rootColumns[block];
    if (layout.rootModes[block] == RootMotion.Planar) {
      jacobian[row * w + c] = linearWeight;
      jacobian[(row + 1) * w + c + 1] = linearWeight;
      jacobian[(row + 2) * w + c + 2] = angularWeight;
    } else {
      for (i in 0...3) jacobian[(row + i) * w + c + i] = linearWeight;
      for (i in 3...6) jacobian[(row + i) * w + c + i] = angularWeight;
    }
  }
}
