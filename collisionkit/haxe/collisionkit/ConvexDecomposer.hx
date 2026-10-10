package collisionkit;

/**
 * Splits a closed triangle mesh into convex pieces whose union, inflated by
 * the decomposition's `inflation`, encloses it (CL-D11).
 * `collisionkit.native.NativeConvexDecomposer` implements it with V-HACD 4.
 */
interface ConvexDecomposer {
  /**
   * `vertices` x, y, z each, `indices` three per triangle; at most
   * `maxPieces` pieces; `sampleSpacing` 0 picks 0.5 % of the bounding
   * diagonal.
   */
  function decompose(vertices:Array<Float>, indices:Array<Int>, maxPieces:Int, sampleSpacing:Float):ConvexDecomposition;
}
