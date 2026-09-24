package materia.automation.task;

/** Immutable identity and kind shared by declarative facility tasks. */
class Task {
  public final id:String;
  public final kind:TaskKind;

  public function new(id:String, kind:TaskKind) {
    if (id == null || id.length == 0) throw "Task requires a non-empty ID";
    this.id = id;
    this.kind = kind;
  }
}
