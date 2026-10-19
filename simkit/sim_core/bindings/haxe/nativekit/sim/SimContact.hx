package nativekit.sim;

/** One measured world contact, including object/object and robot/object pairs. SI units. */
typedef SimContact = {
  var bodyA:Int;
  var bodyB:Int;
  var distance:Float;
  var active:Bool;
  var wrenchValid:Bool;
  /** World-axis contact force acting on bodyA; negate it for bodyB (newtons). */
  var force:Array<Float>;
}
