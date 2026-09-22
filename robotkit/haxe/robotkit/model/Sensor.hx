package robotkit.model;

class Sensor {
  public final id:SensorId;
  public var name:String;
  public final kind:String;
  public var updateRate:Float;

  public function new(name:String, kind:String, ?updateRate:Float = 0.0, ?id:SensorId) {
    // Legacy callers use the initial name once; imports pass the stored ID.
    this.id = id == null ? name : id;
    this.name = name;
    this.kind = kind;
    this.updateRate = updateRate;
  }
}
