package robotkit.policy;

import haxe.Int64;

/** A planar velocity command: forward, lateral (m/s) and yaw rate (rad/s), in the robot's base frame. */
typedef VelocityCommand = {vx:Float, vy:Float, wz:Float};

/** Why a submitted command was not accepted. */
enum CommandRejection {
  /** The command's sequence is not newer than the last accepted one. */
  Stale;
  /** A component is not a finite number. */
  NotFinite;
  /** The deadline has already passed on the reference's clock. */
  Expired;
}

/**
 * The velocity command a walking controller follows, as a cyclic reference
 * (motionkit/plans/LANE_D_REDUNDANCY_SERVO.md, LD-D3) rather than a queue of
 * trajectories to replace:
 *
 * - every command carries a sequence, and a deadline on the clock the
 *   controller runs on; older or equal sequences are rejected;
 * - the target is clamped to the configured limits;
 * - the value the controller reads changes no faster than the acceleration
 *   limits, so a step in the command is a ramp; and
 * - when the deadline passes without a newer command the target becomes zero,
 *   and the value brakes to a stop within the same limits, so a lost
 *   controller leaves the robot standing, not walking on.
 */
class VelocityReference {
  final limit:Array<Float>;
  final acceleration:Array<Float>;
  final target:Array<Float> = [0.0, 0.0, 0.0];
  final current:Array<Float> = [0.0, 0.0, 0.0];
  var deadlineNs:Int64 = Int64.ofInt(0);
  var lastSequence:Int = 0;
  var lastSampleNs:Null<Int64> = null;
  /** Whether the last accepted command's deadline has passed. */
  public var expired(default, null):Bool = true;

  /**
   * `limit` and `acceleration` are per axis (forward, lateral, yaw): the
   * largest magnitude of the command and the fastest it may change per second.
   */
  public function new(limit:Array<Float>, acceleration:Array<Float>) {
    if (limit.length != 3 || acceleration.length != 3) throw "velocity limits are per axis: forward, lateral, yaw";
    for (i in 0...3)
      if (!(limit[i] >= 0.0) || !(acceleration[i] > 0.0) || !Math.isFinite(limit[i]) || !Math.isFinite(acceleration[i]))
        throw "velocity limits must be finite and non-negative, accelerations positive";
    this.limit = limit.copy();
    this.acceleration = acceleration.copy();
  }

  /**
   * Accepts a command, or names why not. `deadlineNs` is on the same clock as
   * `sample`'s `nowNs`, which the caller must pass (a zero deadline is already
   * past).
   */
  public function submit(command:VelocityCommand, sequence:Int, deadlineNs:Int64, nowNs:Int64):Null<CommandRejection> {
    if (!Math.isFinite(command.vx) || !Math.isFinite(command.vy) || !Math.isFinite(command.wz)) return NotFinite;
    if (sequence <= lastSequence) return Stale;
    if (deadlineNs <= nowNs) return Expired;
    lastSequence = sequence;
    this.deadlineNs = deadlineNs;
    var values = [command.vx, command.vy, command.wz];
    for (i in 0...3) target[i] = Math.max(-limit[i], Math.min(limit[i], values[i]));
    expired = false;
    return null;
  }

  /** Drops the command at once: the target is zero from the next sample. */
  public function clear():Void {
    for (i in 0...3) target[i] = 0.0;
    expired = true;
  }

  /** Advances the reference to `nowNs` and returns the velocity to follow. */
  public function sample(nowNs:Int64):VelocityCommand {
    if (!expired && nowNs >= deadlineNs) clear();
    var seconds = lastSampleNs == null ? 0.0 : Int64.toInt(nowNs - lastSampleNs) * 1e-9;
    lastSampleNs = nowNs;
    if (seconds < 0.0) seconds = 0.0;
    for (i in 0...3) {
      var step = acceleration[i] * seconds, error = target[i] - current[i];
      current[i] += Math.abs(error) <= step ? error : (error > 0.0 ? step : -step);
    }
    return {vx: current[0], vy: current[1], wz: current[2]};
  }

  /** The clamped target the reference is heading for, zero once expired. */
  public function targetCommand():VelocityCommand return {vx: target[0], vy: target[1], wz: target[2]};
}
