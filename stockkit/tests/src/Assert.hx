class Assert {
  public static var count = 0;

  public static function check(ok:Bool, message:String):Void {
    count++;
    if (!ok) throw message;
  }

  public static function near(actual:Float, expected:Float, message:String,
      tolerance:Float = 1e-12):Void
    check(Math.abs(actual - expected) <= tolerance,
      '$message: expected $expected, got $actual');
}
