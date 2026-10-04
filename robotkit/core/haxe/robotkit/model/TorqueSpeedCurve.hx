package robotkit.model;

/**
 * Torque a drive can deliver at each speed, as points joined by straight lines: speeds in
 * ascending order (in the drive's coordinate units per second, rad/s for a rotor) and the torque
 * at each (N m). Below the first speed the first torque holds; above the last speed the drive can
 * deliver nothing. Both stepper pull-out curves and servo torque-speed envelopes use it.
 */
class TorqueSpeedCurve {
  public final speeds:Array<Float>;
  public final torques:Array<Float>;

  public function new(speeds:Array<Float>, torques:Array<Float>) {
    if (speeds == null || torques == null || speeds.length != torques.length || speeds.length == 0)
      throw "A torque-speed curve needs the same, non-zero number of speeds and torques";
    for (index in 0...speeds.length) {
      if (!Math.isFinite(speeds[index]) || speeds[index] < 0.0 || !Math.isFinite(torques[index]) || torques[index] < 0.0)
        throw "Torque-speed curve points must be finite and non-negative";
      if (index > 0 && !(speeds[index] > speeds[index - 1]))
        throw "Torque-speed curve speeds must increase";
    }
    this.speeds = speeds.copy();
    this.torques = torques.copy();
  }

  /** The same torque up to `maxSpeed`, then nothing: how a drive with only two numbers behaves. */
  public static function flat(torque:Float, maxSpeed:Float):TorqueSpeedCurve
    return new TorqueSpeedCurve([0.0, maxSpeed], [torque, torque]);

  /** Torque available at `speed`, in either direction. */
  public function torqueAt(speed:Float):Float {
    var magnitude = Math.abs(speed);
    var last = speeds.length - 1;
    if (magnitude <= speeds[0]) return torques[0];
    if (magnitude > speeds[last] + 1e-9 * (1.0 + speeds[last])) return 0.0;
    for (index in 1...speeds.length) if (magnitude <= speeds[index]) {
      var span = speeds[index] - speeds[index - 1];
      var along = (magnitude - speeds[index - 1]) / span;
      return torques[index - 1] + along * (torques[index] - torques[index - 1]);
    }
    return torques[last];
  }

  /** The most torque at any speed. */
  public function peakTorque():Float {
    var peak = 0.0;
    for (torque in torques) peak = Math.max(peak, torque);
    return peak;
  }

  /** The highest speed with any torque. */
  public function maxSpeed():Float return speeds[speeds.length - 1];

  /** Alternating speed, torque, for saving. */
  public function flatten():Array<Float> {
    var out:Array<Float> = [];
    for (index in 0...speeds.length) {
      out.push(speeds[index]);
      out.push(torques[index]);
    }
    return out;
  }

  public static function unflatten(values:Array<Float>):TorqueSpeedCurve {
    if (values == null || values.length == 0 || values.length % 2 != 0)
      throw "A flattened torque-speed curve needs speed and torque pairs";
    return new TorqueSpeedCurve([for (index in 0...Std.int(values.length / 2)) values[2 * index]],
      [for (index in 0...Std.int(values.length / 2)) values[2 * index + 1]]);
  }

  /** A copy with every speed multiplied by `factor`, for a change of the coordinate's units. */
  public function scaledSpeeds(factor:Float):TorqueSpeedCurve
    return new TorqueSpeedCurve([for (speed in speeds) speed * factor], torques);
}
