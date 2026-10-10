package collisionkit;

/**
 * A mesh split into convex pieces (CL-D11): each piece's vertices (x, y, z
 * each), and the inflation after which their union encloses the mesh:
 * `measured` (the farthest a sample of the mesh lies outside every piece)
 * plus `spacing` (no point of the mesh is farther than that from a sample).
 */
class ConvexDecomposition {
  public final pieces:Array<Array<Float>>;
  public final measured:Float;
  public final spacing:Float;
  public final inflation:Float;

  public function new(pieces:Array<Array<Float>>, measured:Float, spacing:Float, inflation:Float) {
    this.pieces = pieces;
    this.measured = measured;
    this.spacing = spacing;
    this.inflation = inflation;
  }
}
