package robotkit.model;

class Sensor {
  public final id:SensorId;
  public var name:String;
  public final kind:String;
  public var updateRate:Float;
  /** Null mounts at the root link origin; otherwise references a model frame. */
  public var frame:Null<Frame>;
  public var rayCount:Int = 8;
  public var maxRange:Float = 10.0;
  public var noiseStddev:Float = 0.0;
  public var noiseSeed:Int = 1;

  public function new(name:String, kind:String, ?updateRate:Float = 0.0, ?id:SensorId) {
    // Legacy callers use the initial name once; imports pass the stored ID.
    this.id = id == null ? name : id;
    this.name = name;
    this.kind = kind;
    this.updateRate = updateRate;
  }
}
