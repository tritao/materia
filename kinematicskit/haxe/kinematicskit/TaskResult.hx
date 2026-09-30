package kinematicskit;

/** One task's errors in a `KinematicSolution`. */
class TaskResult {
  public final label:String;
  public final satisfied:Bool;
  public final soft:Bool;
  public final positionError:Float;
  public final orientationError:Float;

  public function new(task:KinematicTask) {
    label = task.label();
    satisfied = task.satisfied();
    soft = task.isSoft();
    positionError = task.positionError();
    orientationError = task.orientationError();
  }
}
