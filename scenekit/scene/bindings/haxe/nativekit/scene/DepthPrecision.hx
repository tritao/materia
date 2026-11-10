package nativekit.scene;

/**
 * What sizes a perspective near plane: the depth buffer's resolution, the
 * viewport's pixel height, and how deep the surface being looked at lies.
 */
class DepthPrecision {
  /** Largest depth step at the focus, in pixel footprints there. */
  public static inline var FOCUS_STEP_PIXELS:Float = 0.02;

  public final depthBits:Int;
  public final viewportHeight:Float;
  /** View depth of the looked-at surface; zero or less falls back to the orbit distance. */
  public final focusDepth:Float;

  public function new(depthBits:Int, viewportHeight:Float, focusDepth:Float = 0.0) {
    this.depthBits = depthBits;
    this.viewportHeight = viewportHeight;
    this.focusDepth = focusDepth;
  }

  /** One step of the depth buffer, as a fraction of its 0..1 window range. */
  public function depthStep():Float return 1.0 / Math.pow(2.0, depthBits);
}
