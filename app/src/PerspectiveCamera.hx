package app;

/** Compatibility subclass for the reusable EditorKit orbit camera. */
class PerspectiveCamera extends nativekit.editorkit.PerspectiveCamera {
  public function new() super();
}
typedef PerspectiveRay = nativekit.editorkit.PerspectiveCamera.PerspectiveRay;
typedef PerspectivePlanePoint = nativekit.editorkit.PerspectiveCamera.PerspectivePlanePoint;
typedef PerspectiveScreenPoint = nativekit.editorkit.PerspectiveCamera.PerspectiveScreenPoint;
