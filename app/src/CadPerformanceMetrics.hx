package app;

typedef CadPerformanceMetrics = {
  var revision:Int;
  var recomputeAttempts:Int;
  var recomputeSeconds:Float;
  var evaluatedFeatures:Int;
  var sketchSolveSeconds:Float;
  var sketchSolveCount:Int;
  var sketchProfileSeconds:Float;
  var tessellationSeconds:Float;
  var geometryConversionSeconds:Float;
  var publicationSeconds:Float;
  /** Process-wide live native handles sampled when performanceMetrics() is called. */
  var nativeShapeHandles:Int;
  var nativeMeshHandles:Int;
  var nativeOperationHandles:Int;
}
