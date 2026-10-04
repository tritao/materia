package robotkit.manipulation;

/** A circular keep-out region in the map frame a candidate base position must clear by the search's `clearance`. */
class BaseObstacle {
  public final centerX:Float;
  public final centerY:Float;
  public final radius:Float;

  public function new(centerX:Float, centerY:Float, radius:Float) {
    this.centerX = centerX;
    this.centerY = centerY;
    this.radius = radius;
  }
}
