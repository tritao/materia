package kinematicskit;

/**
 * A soft preference for DOF values near `targets` (one row per DOF,
 * `weight · (target − q)`). It never blocks convergence; use it to resolve
 * redundancy (preferred posture) or to stay near a seed.
 */
class PostureTask implements KinematicTask {
  public final targets:Array<Float>;
  public var weight:Float;
  final dofCount:Int;
  var lastError = 0.0;

  public function new(model:KinematicModel, targets:Array<Float>, weight:Float) {
    if (model == null || targets == null || targets.length != model.dofCount())
      throw 'Posture task requires ${model == null ? 0 : model.dofCount()} target values';
    if (!Math.isFinite(weight) || weight < 0.0) throw "Posture task weight must be finite and non-negative";
    this.targets = targets.copy();
    this.weight = weight;
    dofCount = model.dofCount();
  }

  public function label():String return "posture";
  public function rowCount():Int return dofCount;
  public function isSoft():Bool return true;
  public function positionError():Float return 0.0;
  /** Euclidean distance from the targets (mixed units when DOFs mix kinds). */
  public function orientationError():Float return lastError;
  public function satisfied():Bool return true;

  public function evaluate(state:KinematicState, snapshot:KinematicSnapshot, residual:Array<Float>,
      jacobian:Array<Float>, row:Int):Void {
    var squared = 0.0;
    for (i in 0...dofCount) {
      var e = targets[i] - state.q[i];
      squared += e * e;
      residual[row + i] = weight * e;
      for (c in 0...dofCount) jacobian[(row + i) * dofCount + c] = c == i ? weight : 0.0;
    }
    lastError = Math.sqrt(squared);
  }
}
