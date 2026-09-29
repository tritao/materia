package robotkit.manipulation;

import robotkit.tool.ToolCollisionShape;

/** Collision axes and vertices prepared once in the tool frame. */
class ToolClearanceShape {
  public final checker:ToolClearanceChecker;
  public static var preparationCount(default, null):Int = 0;

  function new(checker:ToolClearanceChecker) this.checker = checker;

  public static function prepare(shape:ToolCollisionShape):ToolClearanceShape {
    preparationCount++;
    return new ToolClearanceShape(new ToolClearanceChecker(shape, []));
  }
}
