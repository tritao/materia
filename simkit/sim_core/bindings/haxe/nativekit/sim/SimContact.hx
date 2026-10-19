package nativekit.sim;

/** One measured world contact, including object/object and robot/object pairs. SI units. */
typedef SimContact = {
  var bodyA:Int;
  var bodyB:Int;
  var distance:Float;
  var active:Bool;
  var wrenchValid:Bool;
  var force:Array<Float>;
}
