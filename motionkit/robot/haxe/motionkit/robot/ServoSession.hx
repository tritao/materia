package motionkit.robot;

import haxe.Int64;
import motionkit.kinematics.Twist6;
import robotkit.manipulation.Manipulator;
import robotkit.policy.VelocityReference.CommandRejection;
import robotkit.runtime.RobotRuntimeError;
import robotkit.world.JointTarget;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;

/** What one servo tick did. */
class ServoTick {
  /** The joint velocities commanded for the next period, in arm order. */
  public final velocity:Array<Float>;
  /** True while no live command is being followed and the arm is braking (or at rest). */
  public final braking:Bool;
  /** True once braking has brought every joint to rest. */
  public final atRest:Bool;
  public final step:ServoStep;

  public function new(velocity:Array<Float>, braking:Bool, atRest:Bool, step:ServoStep) {
    this.velocity = velocity;
    this.braking = braking;
    this.atRest = atRest;
    this.step = step;
  }
}

/**
 * Live Cartesian servoing of an arm (jogging, teleoperation) as a cyclic
 * reference (motionkit/plans/LANE_D_REDUNDANCY_SERVO.md, LD-D3), not a queue
 * of trajectories to replace:
 *
 * - an operator submits tool twists (base frame, at the TCP) with a sequence
 *   and a deadline on the robot's clock; older sequences, non-finite twists
 *   and already-passed deadlines are rejected;
 * - every robot tick, `update` reads the snapshot, takes one bounded,
 *   acceleration-limited `ManipulatorServo` step towards the live twist and
 *   submits the resulting joint velocities;
 * - when the deadline passes without a newer command (or on `stop`), the
 *   twist becomes zero and the arm brakes to rest within its acceleration
 *   limits, so a lost operator leaves the arm standing, not moving on.
 *
 * The clock is the snapshot's `sourceTimestampNs`. The joint targets carry
 * the deadline too (braking ones a two-period keepalive), so if the host
 * stalls, the runtime brakes each joint to zero within its own acceleration
 * limit. That brake is plain per joint, so the tool leaves its line and a
 * joint may reach its limit; while `update` runs, the session's brake keeps
 * the servo's limits. Joint targets are sticky in the runtime, so braking
 * ends with exact zeros, sent without a deadline.
 *
 * With `ServoPlanOptions` the session drives robots that execute plans
 * instead (the virtual device): `update` keeps a short stream of one-period
 * `ServoPlan` chunks queued ahead of execution, each stepping the servo from
 * the state at the end of the queue (so the servo acts `leadSeconds` ahead).
 * Braking ends the stream with a chunk at rest. A stalled host lets the
 * queue run dry within the lead, and the runtime or device then brakes
 * every joint at its acceleration limit (reporting an underflow).
 */
class ServoSession {
  public final robot:Robot;
  public final manipulator:Manipulator;
  final servo:ManipulatorServo;
  final indices:Array<Int>;
  final restVelocity:Float;
  final controlPeriod:Float;
  var twist:Array<Float> = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
  var live = false;
  var deadlineNs:Int64 = Int64.ofInt(0);
  var lastSequence = 0;
  var lastNs:Null<Int64> = null;
  var commanded:Array<Float>;
  final plan:Null<ServoPlan>;

  /**
   * `controlPeriod` is the interval `update` is meant to run at (the robot's
   * control period). The runtime applies a velocity target at once, so each
   * update may change a joint's velocity by at most a·controlPeriod even if
   * updates were missed: a stalled host ramps back up instead of jumping.
   * `restVelocity`: below this joint speed (rad/s or m/s) a braking arm counts
   * as at rest.
   */
  public function new(robot:Robot, manipulator:Manipulator, ?controlPeriod:Float = 0.01, ?damping:Float = 1e-3,
      ?restVelocity:Float = 1e-4, ?plans:ServoPlanOptions) {
    if (robot == null || manipulator == null) throw "Servo session requires a robot and a manipulator";
    if (!(controlPeriod > 0.0) || !Math.isFinite(controlPeriod)) throw "Servo control period must be positive";
    this.controlPeriod = controlPeriod;
    this.robot = robot;
    this.manipulator = manipulator;
    this.restVelocity = restVelocity;
    servo = new ManipulatorServo(manipulator, damping);
    indices = manipulator.jointIndices();
    commanded = [for (_ in indices) 0.0];
    if (plans != null && !robot.capabilities().supportsExecutionPlans)
      throw "Servo plans need a robot that executes plans";
    plan = plans == null ? null : new ServoPlan(plans, manipulator, robot.snapshot().positions.length);
  }

  /** The robot's clock now: the latest snapshot's source timestamp. */
  public function nowNs():Int64 return robot.snapshot().sourceTimestampNs;

  /**
   * Accepts a tool twist to follow until `deadlineNs` (robot clock), or names
   * why not. A newer command replaces the current one at once.
   */
  public function command(value:Twist6, sequence:Int, deadlineNs:Int64):Null<CommandRejection> {
    if (value == null) return NotFinite;
    var values = value.toArray();
    for (component in values) if (!Math.isFinite(component)) return NotFinite;
    if (sequence <= lastSequence) return Stale;
    if (deadlineNs <= nowNs()) return Expired;
    lastSequence = sequence;
    this.deadlineNs = deadlineNs;
    twist = values;
    live = true;
    return null;
  }

  /** Drops the live command: the arm brakes to rest from the next tick. */
  public function stop():Void {
    live = false;
    twist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
  }

  /** True while a command is being followed (not expired, not stopped). */
  public function following():Bool return live;

  /** One tick: read the robot, step towards the live twist (or brake) and submit joint velocities. */
  public function update():ServoTick {
    var snapshot = robot.snapshot();
    var now = snapshot.sourceTimestampNs;
    if (live && now >= deadlineNs) stop();
    if (plan != null) return updatePlan(snapshot);
    var elapsed = lastNs == null ? 0.0 : Int64.toInt(now - lastNs) * 1e-9;
    // Missed updates do not license a bigger velocity change: the runtime applies targets at once.
    var dt = Math.min(elapsed, controlPeriod);
    lastNs = now;
    var q = [for (index in indices) snapshot.positions.get(index)];
    if (!(dt > 0.0)) {
      // First tick (or no time passed): hold what was commanded.
      return new ServoTick(commanded.copy(), !live, isAtRest(commanded), null);
    }
    var requested = new Twist6(twist[0], twist[1], twist[2], twist[3], twist[4], twist[5]);
    var step = servo.step(q, requested, dt, null, 1000, 1.0, commanded);
    commanded = step.velocity.copy();
    // Joint targets are sticky in the runtime: when braking, a speed below the rest threshold becomes an
    // exact zero, or the last tiny command would keep the arm creeping.
    if (!live) for (i in 0...commanded.length) if (Math.abs(commanded[i]) <= restVelocity) commanded[i] = 0.0;
    var targets = [for (i in 0...indices.length) JointTarget.velocity(indices[i], commanded[i])];
    // The runtime enforces the deadline if this host stalls; a brake in progress gets a short keepalive.
    var expiry:Null<Int64> = live ? deadlineNs
      : isAtRest(commanded) ? null : now + Int64.fromFloat(2.0 * controlPeriod * 1e9);
    robot.submit(RobotCommand.JointTargets(targets, expiry));
    return new ServoTick(commanded.copy(), !live, !live && isAtRest(commanded), step);
  }

  function updatePlan(snapshot:robotkit.world.RobotSnapshot):ServoTick {
    var plan:ServoPlan = this.plan;
    var active = snapshot.trajectoryActive;
    // The stream drained: it ended at rest, or ran dry and the robot braked it. A device starts a new
    // queue after a delay and reports no motion until then, so only a stream seen running can drain.
    if (plan.streaming && active) plan.markRunning();
    if (plan.streaming && plan.running && !active) plan.clear();
    if (!plan.streaming) {
      if (!live || active) {
        commanded = [for (_ in indices) 0.0];
        return new ServoTick(commanded.copy(), !live, !live && !active, null);
      }
      plan.begin([for (j in 0...snapshot.positions.length) snapshot.positions.get(j)]);
    }
    if (plan.endedAtRest) return new ServoTick(commanded.copy(), !live, false, null);
    var periodNs = Int64.fromFloat(Math.round(controlPeriod * 1e9));
    var leadNs = Int64.fromFloat(plan.options.leadSeconds * 1e9);
    var executed = active ? snapshot.trajectoryTimeNs : Int64.ofInt(0);
    var step:Null<ServoStep> = null;
    // One chunk per period; a few more catch up after a late update.
    var chunks = 0;
    while (chunks < 4 && !plan.endedAtRest && (!plan.streaming || plan.endNs - executed < leadNs)) {
      var requested = new Twist6(twist[0], twist[1], twist[2], twist[3], twist[4], twist[5]);
      var armV = plan.armVelocity();
      step = servo.step(plan.armPosition(), requested, controlPeriod, null, 1000, 1.0, armV,
        plan.accelerationLimits, true);
      var velocity = step.velocity.copy();
      if (!live) for (i in 0...velocity.length) if (Math.abs(velocity[i]) <= restVelocity) velocity[i] = 0.0;
      var submission = plan.chunk(velocity, periodNs, !live && isAtRest(velocity));
      try robot.submit(RobotCommand.ExecutionPlan(submission)) catch (error:RobotRuntimeError) {
        // The queue moved on underneath (it ran dry): start a new stream next update.
        if (error.status != RobotKitRuntimeConstants.RK_ERROR_INVALID_STATE) throw error;
        plan.clear();
        break;
      }
      plan.accept(submission);
      commanded = velocity;
      chunks++;
    }
    return new ServoTick(commanded.copy(), !live, false, step);
  }

  public function dispose():Void servo.dispose();

  function isAtRest(velocity:Array<Float>):Bool {
    for (value in velocity) if (Math.abs(value) > restVelocity) return false;
    return true;
  }
}
