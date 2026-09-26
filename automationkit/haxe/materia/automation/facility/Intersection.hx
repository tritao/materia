package materia.automation.facility;

/** Shared conflict region whose configured lanes require exclusive passage. */
class Intersection {
  public final id:String;
  public final name:String;
  public final zoneId:String;
  public final frameId:String;
  final laneValues:Array<String>;

  public function new(id:String, name:String, zoneId:String, frameId:String,
      laneIds:Array<String>) {
    if (id == null || id.length == 0 || name == null || name.length == 0 ||
        zoneId == null || zoneId.length == 0 || frameId == null || frameId.length == 0 ||
        laneIds == null || laneIds.length < 2)
      throw "Intersection requires an ID, zone, frame, and at least two lanes";
    laneValues = [];
    for (laneId in laneIds) {
      if (laneId == null || laneId.length == 0 || laneValues.indexOf(laneId) >= 0)
        throw "Intersection lane IDs must be non-empty and unique";
      laneValues.push(laneId);
    }
    this.id = id;
    this.name = name;
    this.zoneId = zoneId;
    this.frameId = frameId;
  }

  public function lanes():Array<String> return laneValues.copy();
}
