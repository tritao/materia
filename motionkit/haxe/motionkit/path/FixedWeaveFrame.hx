package motionkit.path;

class FixedWeaveFrame implements WeaveFrame {
  final direction:WeaveDirection;
  public function new(axis:Array<Float>) direction = new WeaveDirection(axis, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
  public function at(distance:Float):WeaveDirection return direction;
}
