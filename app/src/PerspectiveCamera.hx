package app;

/** Compatibility subclass for the reusable SceneKit orbit camera. */
class PerspectiveCamera extends nativekit.scene.PerspectiveCamera {
  public function new() super();
}
typedef PerspectiveRay = nativekit.scene.PerspectiveCamera.PerspectiveRay;
typedef PerspectivePlanePoint = nativekit.scene.PerspectiveCamera.PerspectivePlanePoint;
typedef PerspectiveScreenPoint = nativekit.scene.PerspectiveCamera.PerspectiveScreenPoint;
