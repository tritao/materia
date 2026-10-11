package robotkit.manipulation;

import robotkit.model.CollisionShape;
import robotkit.tool.ToolGeometry;

/** Collision axes and vertices prepared once in the tool frame. Reuse with the
 * same Array<CollisionShape> instance passed to prepare. */
class ToolClearanceShape {
  public final checker:ToolClearanceChecker;
  public final shape:Array<CollisionShape>;
  /** Diagnostic count of shape preparations, including tests. */
  public static var preparationCount(default, null):Int = 0;

  function new(shape:Array<CollisionShape>, checker:ToolClearanceChecker) {
    this.shape = shape;
    this.checker = checker;
  }

  public static function prepare(shape:Array<CollisionShape>):ToolClearanceShape {
    preparationCount++;
    return new ToolClearanceShape(shape, new ToolClearanceChecker(shape, []));
  }
}
