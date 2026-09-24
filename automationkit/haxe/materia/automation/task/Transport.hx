package materia.automation.task;

import robotkit.material.Payload;

/** Move a known payload from one facility station to another. */
class Transport extends Task {
  public final pickupStationId:String;
  public final destinationStationId:String;
  public final payload:Payload;

  public function new(id:String, pickupStationId:String, destinationStationId:String,
      payload:Payload) {
    super(id, TaskKind.Transport);
    if (pickupStationId == null || pickupStationId.length == 0 ||
        destinationStationId == null || destinationStationId.length == 0 ||
        pickupStationId == destinationStationId || payload == null)
      throw "Transport requires distinct stations and a payload";
    this.pickupStationId = pickupStationId;
    this.destinationStationId = destinationStationId;
    this.payload = payload;
  }
}
