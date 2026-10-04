package motionkit.path;

/** Material frame independent of the torch's orientation and roll. */
interface WeaveFrame {
  function at(distance:Float):WeaveDirection;
}
