import kinematicskit.FrameVelocityTask;
import kinematicskit.JointKind;
import kinematicskit.KinematicModelBuilder;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicState;
import kinematicskit.StepLimits;
import kinematicskit.Transform;
import kinematicskit.Vector3;
import kinematicskit.native.DifferentialIk;
import kinematicskit.native.NativeQpStep;

class NativeStepLimitTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void { checks++; if (!ok) throw message; }
  public static function main():Void {
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base"), tip = builder.addBody("tip");
    builder.addJoint("slide", JointKind.Prismatic, base, tip, Transform.identity(), Transform.identity(), new Vector3(0, 0, 1));
    var model = builder.build();
    var task = new FrameVelocityTask(model, tip);
    var problem = new KinematicProblem(model).setLimits(0, -0.1, 0.1).add(task);
    var qp = new NativeQpStep(1);
    for (ramped in [false, true]) for (mode in 0...4) {
      var limits = new StepLimits(); limits.ramped = ramped;
      limits.velocity = [0.1]; limits.previousVelocity = [0.0];
      if (mode > 0) limits.acceleration = [mode == 1 ? 0.0 : mode == 2 ? Math.POSITIVE_INFINITY : 1.0];
      task.setTwist([0.0, 0.0, -0.005, 0.0, 0.0, 0.0], 0.01);
      var step = DifferentialIk.step(problem, new KinematicState(model, [0.0]), 0.01, qp, limits);
      check(!step.fallback && step.status == 0, "Native QP solves with absent, zero, infinite or finite acceleration");
      check(Math.isFinite(step.velocity[0]) && Math.abs(step.velocity[0] + 0.005) < 5e-8,
        "Missing acceleration retains a finite accurate differential-IK velocity");
      task.setTwist([0.0, 0.0, 0.1, 0.0, 0.0, 0.0], 0.01);
      step = DifferentialIk.step(problem, new KinematicState(model, [0.0999]), 0.01, qp, limits);
      var travel = step.velocity[0] * (ramped ? 0.005 : 0.01);
      check(Math.isFinite(travel) && travel >= 0 && 0.0999 + travel <= 0.10000000001,
        "Native steps retain the position stop with unlimited acceleration");
      step = DifferentialIk.step(problem, new KinematicState(model, [0.0999]), 0.01, qp, limits, 1.0, 1e-3, null, 0);
      check(step.fallback && Math.isFinite(step.velocity[0]) && 0.0999 + step.velocity[0] * (ramped ? 0.005 : 0.01) <= 0.10000000001,
        "A failed QP returns a finite damped step within the same travel bounds");
    }
    qp.dispose();
    Sys.println('Native step limits: $checks assertions passed');
  }
}
