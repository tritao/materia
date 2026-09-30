package kinematicskit;

/**
 * Buffers a solver needs, sized for one problem shape and reused across
 * solves (e.g. one per frame while an editor gizmo is dragged). Growing is
 * the only allocation; a solve with an already-fitting workspace allocates
 * nothing per iteration.
 */
class SolverWorkspace {
  public var snapshot(default, null):Null<KinematicSnapshot> = null;
  public final residual:Array<Float> = [];
  public final jacobian:Array<Float> = [];
  public final scaled:Array<Float> = [];
  public final normal:Array<Float> = [];
  public final rhs:Array<Float> = [];
  public final delta:Array<Float> = [];
  public final accepted:Array<Float> = [];
  public final scales:Array<Float> = [];

  public function new() {}

  public function prepare(problem:KinematicProblem):Void {
    if (snapshot == null || snapshot.model != problem.model) snapshot = new KinematicSnapshot(problem.model);
    var rows = problem.rowCount(), width = problem.layout().width;
    grow(residual, rows);
    grow(jacobian, rows * width);
    grow(scaled, rows * width);
    grow(normal, width * width);
    grow(rhs, width);
    grow(delta, width);
    grow(accepted, width);
    grow(scales, width);
  }

  static function grow(values:Array<Float>, size:Int):Void {
    if (values.length < size) values.resize(size);
  }
}
