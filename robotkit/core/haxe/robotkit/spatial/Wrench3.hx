package robotkit.spatial;

/** Immutable spatial force: force (N) and torque (N*m) parts, both expressed in the same frame. */
class Wrench3 {
  public final force:Vec3;
  public final torque:Vec3;

  public function new(force:Vec3, torque:Vec3) {
    if (force == null || torque == null)
      throw "Wrench3 requires force and torque components";
    this.force = force;
    this.torque = torque;
  }

  public static function zero():Wrench3 return new Wrench3(Vec3.zero(), Vec3.zero());
}
