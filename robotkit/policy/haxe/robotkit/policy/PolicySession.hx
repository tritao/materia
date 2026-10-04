package robotkit.policy;

import haxe.Int64;
import robotkit.policy.VelocityReference.CommandRejection;
import robotkit.policy.VelocityReference.VelocityCommand;
import robotkit.core.JointTarget;
import robotkit.core.Robot;
import robotkit.core.RobotCommand;

/**
 * The execution session of a policy-driven robot. It closes the loop every
 * control period: read the robot, sample the velocity reference, evaluate the
 * policy and submit its joint servo targets, all through the `Robot` boundary,
 * so the same session drives a simulated, remote or replayed robot and a
 * RecordingRobot records what it sends.
 *
 * The velocity command is a cyclic reference (VelocityReference, LD-D3): it
 * carries a sequence and a deadline, and when it lapses the reference brakes to
 * zero. Zero is "hold": the policy keeps balancing in place, joints are never
 * frozen.
 *
 * Call `start()` once, then `update()` after each simulation (or device) step.
 */
class PolicySession {
  public final controller:PolicyController;
  public final reference:VelocityReference;
  final robot:Robot;
  /** Session updates per policy evaluation. */
  final ticksPerControl:Int;
  /** Ticks of warm-up left: the default pose is held and only the IMU filter runs. */
  var warmupTicks:Int = 0;
  final tickSeconds:Float;
  var ticks:Int = 0;
  var commandSequence:Int = 0;
  var nowNs:Int64 = Int64.ofInt(0);
  /** The velocity the policy was last given. */
  public var lastCommand(default, null):VelocityCommand = {vx: 0.0, vy: 0.0, wz: 0.0};

  /**
   * `tickSeconds` is the period at which `update()` is called: the simulation
   * tick, or the device's sensor period. The IMU is read on every tick and the
   * policy runs every `controlPeriod / tickSeconds` ticks, which must be a
   * whole number; the default is one evaluation per tick.
   */
  public function new(robot:Robot, controller:PolicyController, ?reference:VelocityReference, ?tickSeconds:Float) {
    this.robot = robot;
    this.controller = controller;
    var period = controller.spec.controlPeriod;
    this.tickSeconds = tickSeconds == null ? period : tickSeconds;
    var ratio = period / this.tickSeconds;
    ticksPerControl = Std.int(Math.round(ratio));
    if (ticksPerControl < 1 || Math.abs(ratio - ticksPerControl) > 1e-6)
      throw 'the control period ($period s) must be a whole number of ticks of $tickSeconds s';
    var limits = controller.spec.command;
    this.reference = reference == null ? new VelocityReference(limits.limit, limits.acceleration) : reference;
  }

  /**
   * Holds the default pose from the first physics step. For `warmupSeconds`
   * the policy stays off while the robot settles and the IMU filter finds down,
   * as a real robot's would while it stands before the walking controller is
   * enabled.
   */
  public function start(?warmupSeconds:Float = 0.0):Void {
    controller.reset();
    ticks = 0;
    warmupTicks = Std.int(Math.round(warmupSeconds / tickSeconds));
    send(controller.standTargets());
  }

  /**
   * Submits a velocity command valid until `deadlineNs` on the robot's clock;
   * sequences must increase, and this session numbers them itself.
   */
  public function command(velocity:VelocityCommand, deadlineNs:Int64):Null<CommandRejection>
    return reference.submit(velocity, ++commandSequence, deadlineNs, nowNs);

  /** Commands `velocity` for `seconds` from the robot's present time. */
  public function commandFor(velocity:VelocityCommand, seconds:Float):Null<CommandRejection>
    return command(velocity, nowNs + Int64.fromFloat(seconds * 1e9 + 0.5));

  /**
   * One tick: read the robot and feed its IMU to the gravity filter, and once
   * per control period sample the reference, evaluate the policy and command.
   * Returns the targets it sent, empty between evaluations.
   */
  public function update():Array<JointTarget> {
    var snapshot = robot.snapshot();
    nowNs = snapshot.sourceTimestampNs;
    var sensors = snapshot.sensors.toArray();
    controller.observeImu(sensors);
    if (++ticks % ticksPerControl != 0 || ticks <= warmupTicks) return [];
    lastCommand = reference.sample(nowNs);
    var targets = controller.update(snapshot.positions.toArray(), snapshot.velocities.toArray(), sensors, lastCommand);
    send(targets);
    return targets;
  }

  function send(targets:Array<JointTarget>):Void
    robot.submit(RobotCommand.JointTargets(targets, null));
}
