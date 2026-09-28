package robotkit.manipulation;

import robotkit.tool.Tool;

/** Mounted tool and map-frame obstacles used for tool-aware work planning. */
class ToolPlanningContext {
  public final tool:Tool;
  public final obstacles:Array<ToolBoxObstacle>;
  public final clearance:Float;
  public final maxJointStep:Float;

  public function new(tool:Tool, obstacles:Array<ToolBoxObstacle>,
      ?clearance:Float = 0.0, ?maxJointStep:Float = 0.02) {
    if (tool == null || obstacles == null || !Math.isFinite(clearance) || clearance < 0.0 ||
        !Math.isFinite(maxJointStep) || maxJointStep <= 0.0)
      throw "Tool planning requires a mounted tool, obstacles, and valid clearance settings";
    if (obstacles.length > 0) switch (tool.collision) {
      case NoCollision: throw "Tool planning requires collision geometry for a mounted tool";
      case _:
    }
    for (obstacle in obstacles) if (obstacle == null)
      throw "Tool planning obstacles cannot contain null";
    this.tool = tool;
    this.obstacles = obstacles.copy();
    this.clearance = clearance;
    this.maxJointStep = maxJointStep;
  }
}
