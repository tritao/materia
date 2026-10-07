package motionkit.robot;

/** Planning contract in SI units. Command/observation latency is included in switch braking room. */
class HomingDynamics {
  public final acceleration:Float;
  public final jerk:Float;
  public final latencySeconds:Float;
  public final timestep:Float;

  public function new(acceleration:Float, jerk:Float, latencySeconds:Float, timestep:Float) {
    for (value in [acceleration, jerk, timestep])
      if (!Math.isFinite(value) || value <= 0) throw "Homing dynamics require positive finite limits";
    if (!Math.isFinite(latencySeconds) || latencySeconds < 0) throw "Invalid homing response latency";
    this.acceleration = acceleration; this.jerk = jerk;
    this.latencySeconds = latencySeconds; this.timestep = timestep;
  }

  /** Keep acceleration ramps observable for at least two owner periods. */
  public function planAcceleration(speed:Float):Float return Math.min(acceleration, speed / (2 * timestep));
  public function planJerk(speed:Float):Float return Math.min(jerk, planAcceleration(speed) / timestep);

  /** Conservative distance through response latency and a jerk-limited stop, including positive initial acceleration. */
  public function stoppingDistance(speed:Float):Float {
    if (!Math.isFinite(speed) || speed < 0) throw "Invalid homing speed";
    if (speed == 0) return 0;
    var a = planAcceleration(speed), j = planJerk(speed);
    var responseSpeed = speed + a * latencySeconds;
    var peakSpeed = responseSpeed + a * a / (2 * j);
    return speed * latencySeconds + 0.5 * a * latencySeconds * latencySeconds +
      peakSpeed * (responseSpeed / a + 3 * a / j);
  }

  public function speedForRoom(room:Float, maximum:Float):Float {
    if (!Math.isFinite(room) || room <= 0 || !Math.isFinite(maximum) || maximum <= 0)
      throw "Homing braking needs positive finite room and speed";
    var lower = 0.0, upper = maximum;
    for (_ in 0...48) {
      var candidate = (lower + upper) / 2;
      if (stoppingDistance(candidate) <= room) lower = candidate; else upper = candidate;
    }
    return lower;
  }
}
