package robotkit.perception;

/**
 * Eigenvalues and eigenvectors of a symmetric matrix from `JacobiEigenSolver`.
 * `eigenvectors[i]` is the (unit-norm) eigenvector for `eigenvalues[i]`, both
 * arrays in the same, otherwise unspecified, order.
 */
class EigenDecomposition {
  public final eigenvalues:Array<Float>;
  public final eigenvectors:Array<Array<Float>>;

  public function new(eigenvalues:Array<Float>, eigenvectors:Array<Array<Float>>) {
    this.eigenvalues = eigenvalues;
    this.eigenvectors = eigenvectors;
  }
}
