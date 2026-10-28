package machinekit.operator;

class JogMove {
  public final axis:String;
  public final distance:Float;
  public function new(axis:String, distance:Float) {
    this.axis = axis;
    this.distance = distance;
  }
}
