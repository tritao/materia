package materia.automation.mission;

import materia.automation.task.Task;

/** Ordered facility-level task sequence with an explicit terminal lifecycle. */
class Mission {
  public final id:String;
  public final name:String;
  final work:Array<Task>;
  public var status(default, null):MissionStatus = Pending;
  var taskIndex:Int = 0;

  public function new(id:String, name:String, tasks:Array<Task>) {
    if (id == null || id.length == 0 || name == null || name.length == 0 ||
        tasks == null || tasks.length == 0)
      throw "Mission requires an ID, name, and at least one task";
    this.id = id;
    this.name = name;
    work = [];
    var taskIds = new Map<String,Bool>();
    for (task in tasks) {
      if (task == null || taskIds.exists(task.id)) throw "Mission task IDs must be unique";
      taskIds.set(task.id, true);
      work.push(task);
    }
  }

  public function tasks():Array<Task> return work.copy();
  public function currentTask():Null<Task> return isTerminal() ? null : work[taskIndex];
  public function completedTaskCount():Int return taskIndex;

  public function start():Void {
    if (status != Pending) throw "Only a pending mission can start";
    status = Running;
  }

  public function completeCurrentTask():Void {
    if (status != Running) throw "Only a running mission can complete tasks";
    taskIndex++;
    if (taskIndex == work.length) status = Succeeded;
  }

  public function fail(message:String):Void {
    if (isTerminal() || status == Pending) throw "Only a running mission can fail";
    if (message == null || message.length == 0) throw "Mission failure requires a message";
    status = Failed(message);
  }

  public function cancel():Void {
    if (isTerminal()) throw "A terminal mission cannot be cancelled";
    status = Cancelled;
  }

  public function isTerminal():Bool return switch status {
    case Succeeded | Failed(_) | Cancelled: true;
    case _: false;
  };
}
