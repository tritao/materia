package motionkit.robot;

/** Per-joint runtime start-state tolerances in solver joint order. */
class StartTolerances {
  public final position:Array<Float>;
  public final velocity:Array<Float>;
  public final acceleration:Array<Float>;

  public function new(position:Array<Float>, velocity:Array<Float>, acceleration:Array<Float>) {
    this.position = checked(position, "position");
    this.velocity = checked(velocity, "velocity");
    this.acceleration = checked(acceleration, "acceleration");
  }

  public static function uniform(count:Int, position:Float, velocity:Float,
      acceleration:Float):StartTolerances {
    return new StartTolerances([for (_ in 0...count) position],
      [for (_ in 0...count) velocity], [for (_ in 0...count) acceleration]);
  }

  public function validate(count:Int):Void {
    if (position.length != count || velocity.length != count || acceleration.length != count)
      throw "Program compiler start tolerance counts must match solver joints";
    for (values in [position, velocity, acceleration])
      for (value in values)
        if (!Math.isFinite(value) || value < 0.0)
          throw "Program compiler start tolerances must be finite and non-negative";
  }

  static function checked(values:Array<Float>, label:String):Array<Float> {
    if (values == null) throw 'Start $label tolerances are required';
    for (value in values)
      if (!Math.isFinite(value) || value < 0.0)
        throw 'Start $label tolerances must be finite and non-negative';
    return values.copy();
  }
}
