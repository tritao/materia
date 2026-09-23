package cadkit.parametric;

/** Cooperative cancellation between document features and sketch-solver iterations. */
class EvaluationCancelled extends haxe.Exception {
  public function new() super("CAD evaluation was superseded");
}
