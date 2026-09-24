package materia.automation.task;

import robotkit.material.Payload;

/** Place a payload into one addressed rack slot. */
class Place extends Task {
  public final rackId:String;
  public final slotId:String;
  public final payload:Payload;

  public function new(id:String, rackId:String, slotId:String, payload:Payload) {
    super(id, TaskKind.Place);
    if (rackId == null || rackId.length == 0 || slotId == null || slotId.length == 0 || payload == null)
      throw "Place requires a rack, slot, and payload";
    this.rackId = rackId;
    this.slotId = slotId;
    this.payload = payload;
  }
}
