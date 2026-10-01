package motionkit.robot;

import kinematicskit.FrameVelocityTask;
import kinematicskit.KinematicProblem;
import kinematicskit.SolverWorkspace;
import kinematicskit.StepLimits;
import kinematicskit.native.DifferentialIk;
import kinematicskit.native.NativeQpStep;
import motionkit.kinematics.Twist6;
import robotkit.manipulation.KinematicGroup;

/** One servo tick's answer. */
class ServoStep {
  /** Joint velocities, one per arm DOF, in `q` order. */
  public final velocity:Array<Float>;
  /** True when the QP did not solve and the clamped Haxe damped step was used instead. */
  public final fallback:Bool;
  /** Why the QP stopped (`KinematicsKitNativeConstants.KK_QP_*`), or -1 when it threw. */
  public final qpStatus:Int;
  public final iterations:Int;
  /** Arm DOF indices whose step sits on a position or velocity bound. */
  public final limited:Array<Int>;

  public function new(velocity:Array<Float>, fallback:Bool, qpStatus:Int, iterations:Int, limited:Array<Int>) {
    this.velocity = velocity;
    this.fallback = fallback;
    this.qpStatus = qpStatus;
    this.iterations = iterations;
    this.limited = limited;
  }
}

/**
 * Bounded differential IK for live servoing (jogging, teleoperation): each
 * tick turns a requested tool twist (at the TCP, in the group's reference
 * frame) into joint velocities for the next period `dt`. It is
 * kinematicskit's `DifferentialIk` on the group's tool `FrameVelocityTask`,
 * natively on ProxQP:
 *
 *   minimize ½‖J·Δ − twist·dt‖² + ½λ²‖Δ‖²   subject to `StepLimits`
 *
 * with the group's position limits, its velocity limits (`velocityLimits`
 * overrides them) and, given `previousVelocity`, its acceleration limits
 * (`accelerationLimits` overrides them), so the step also never approaches a
 * stop faster than it can brake. `limitGain` and `ramped` are StepLimits'
 * (a ramped step is a streamed plan chunk). If the QP does not solve, the
 * damped step clamped into the same bounds is used, and the answer says so.
 *
 * Keep one per arm and reuse it: the QP warm-starts from the previous tick.
 */
class ManipulatorServo {
  public final manipulator:KinematicGroup;
  public final damping:Float;
  final qp:NativeQpStep;
  final problem:KinematicProblem;
  final task:FrameVelocityTask;
  final workspace = new SolverWorkspace();

  public function new(manipulator:KinematicGroup, ?damping:Float = 1e-3) {
    if (manipulator == null) throw "Servo requires a manipulator";
    if (!Math.isFinite(damping) || damping < 0.0) throw "Servo damping must be finite and non-negative";
    this.manipulator = manipulator;
    this.damping = damping;
    qp = new NativeQpStep(manipulator.dofCount());
    task = manipulator.toolVelocityTask();
    problem = manipulator.problem().add(task);
  }

  public function step(q:Array<Float>, twist:Twist6, dt:Float, ?velocityLimits:Array<Float>,
      ?maxIterations:Int = 1000, ?limitGain:Float = 1.0, ?previousVelocity:Array<Float>,
      ?accelerationLimits:Array<Float>, ?ramped:Bool = false):ServoStep {
    var n = manipulator.dofCount();
    if (q == null || q.length != n) throw 'Servo requires $n joint values';
    if (twist == null) throw "Servo requires a tool twist";
    if (!(dt > 0.0) || !Math.isFinite(dt)) throw "Servo period must be positive and finite";
    var limits = new StepLimits();
    limits.limitGain = limitGain;
    limits.ramped = ramped;
    limits.velocity = velocityLimits != null ? velocityLimits.copy() : [for (joint in 0...n) {
      var speed = manipulator.group.limitsOf(joint).velocity;
      speed > 0.0 ? speed : Math.POSITIVE_INFINITY;
    }];
    if (previousVelocity != null) {
      limits.previousVelocity = previousVelocity.copy();
      limits.acceleration = accelerationLimits != null ? accelerationLimits.copy()
        : [for (joint in 0...n) manipulator.group.limitsOf(joint).maxAcceleration];
    }
    task.setTwist(twist.toArray(), dt);
    var solved = DifferentialIk.step(problem, manipulator.stateOf(q), dt, qp, limits, 1.0, damping, workspace,
      maxIterations);
    return new ServoStep(solved.velocity, solved.fallback, solved.status, solved.iterations, solved.limited);
  }

  public function dispose():Void qp.dispose();
}
