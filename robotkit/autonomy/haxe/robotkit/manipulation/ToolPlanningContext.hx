package robotkit.manipulation;

import robotkit.tool.Tool;
import collisionkit.CollisionDescription;
import collisionkit.CollisionWorld;

/** Mounted tool and an authored map-frame collision scene for work planning. */
class ToolPlanningContext {
  public final tool:Tool;
  public final obstacles:CollisionDescription;
  public final makeWorld:Void->CollisionWorld;
  public final clearance:Float;
  public final maxToolStep:Float;
  public final maxJointStep:Null<Float>;

  public function new(tool:Tool, obstacles:CollisionDescription, makeWorld:Void->CollisionWorld,
      clearance:Float = 0.0, ?maxJointStep:Null<Float>, maxToolStep:Float = 0.005) {
    if (tool == null || obstacles == null || makeWorld == null || !Math.isFinite(clearance) || clearance < 0 ||
        !Math.isFinite(maxToolStep) || maxToolStep <= 0 ||
        (maxJointStep != null && (!Math.isFinite(maxJointStep) || maxJointStep <= 0)))
      throw "Tool planning requires a tool, collision scene, backend and valid clearance settings";
    if (obstacles.objects.length > 0 && tool.collision.length == 0)
      throw "Tool planning requires collision geometry for a mounted tool";
    this.tool = tool;
    this.obstacles = obstacles.copy();
    this.makeWorld = makeWorld;
    this.clearance = clearance;
    this.maxJointStep = maxJointStep;
    this.maxToolStep = maxToolStep;
  }
}
