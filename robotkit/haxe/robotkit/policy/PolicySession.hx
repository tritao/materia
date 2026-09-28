package robotkit.policy;

import haxe.Int64;
import robotkit.policy.VelocityReference.CommandRejection;
import robotkit.policy.VelocityReference.VelocityCommand;
import robotkit.world.JointTarget;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;

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
  var commandSequence:Int = 0;
  var nowNs:Int64 = Int64.ofInt(0);
  /** The velocity the policy was last given. */
  public var lastCommand(default, null):VelocityCommand = {vx: 0.0, vy: 0.0, wz: 0.0};

  public function new(robot:Robot, controller:PolicyController, ?reference:VelocityReference) {
    this.robot = robot;
    this.controller = controller;
    var limits = controller.spec.command;
    this.reference = reference == null ? new VelocityReference(limits.limit, limits.acceleration) : reference;
  }

  /** Holds the default pose from the first physics step. */
  public function start():Void {
    controller.reset();
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

  /** One control period: observe, evaluate, command. Returns the targets it sent. */
  public function update():Array<JointTarget> {
    var snapshot = robot.snapshot();
    nowNs = snapshot.sourceTimestampNs;
    lastCommand = reference.sample(nowNs);
    var targets = controller.update(snapshot.positions.toArray(), snapshot.velocities.toArray(),
      snapshot.sensors.toArray(), lastCommand);
    send(targets);
    return targets;
  }

  function send(targets:Array<JointTarget>):Void
    robot.submit(RobotCommand.JointTargets(targets, null));
}
