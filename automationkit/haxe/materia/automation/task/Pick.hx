package materia.automation.task;

import robotkit.material.Payload;

/** Retrieve a payload from one addressed rack slot. */
class Pick extends Task {
  public final rackId:String;
  public final slotId:String;
  public final payload:Payload;

  public function new(id:String, rackId:String, slotId:String, payload:Payload) {
    super(id, TaskKind.Pick);
    if (rackId == null || rackId.length == 0 || slotId == null || slotId.length == 0 || payload == null)
      throw "Pick requires a rack, slot, and payload";
    this.rackId = rackId;
    this.slotId = slotId;
    this.payload = payload;
  }
}
