package robot.model;

class Sensor {
  public final name:String;
  public final kind:String;
  public var updateRate:Float;

  public function new(name:String, kind:String, ?updateRate:Float = 0.0) {
    this.name = name;
    this.kind = kind;
    this.updateRate = updateRate;
  }
}
