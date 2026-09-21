package robotkit.model;

class Actuator {
  public final name:String;
  public var maxEffort:Float;
  public var maxRate:Float;

  public function new(name:String, ?maxEffort:Float = 0.0, ?maxRate:Float = 0.0) {
    this.name = name;
    this.maxEffort = maxEffort;
    this.maxRate = maxRate;
  }
}
