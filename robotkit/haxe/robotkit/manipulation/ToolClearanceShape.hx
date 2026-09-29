package robotkit.manipulation;

import robotkit.tool.ToolCollisionShape;

/** Collision axes and vertices prepared once in the tool frame. */
class ToolClearanceShape {
  public final checker:ToolClearanceChecker;
  public final shape:ToolCollisionShape;
  /** Diagnostic count of shape preparations, including tests. */
  public static var preparationCount(default, null):Int = 0;

  function new(shape:ToolCollisionShape, checker:ToolClearanceChecker) {
    this.shape = shape;
    this.checker = checker;
  }

  public static function prepare(shape:ToolCollisionShape):ToolClearanceShape {
    preparationCount++;
    return new ToolClearanceShape(shape, new ToolClearanceChecker(shape, []));
  }
}
