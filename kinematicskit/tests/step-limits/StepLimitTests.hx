import kinematicskit.JointKind;
import kinematicskit.KinematicModelBuilder;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicState;
import kinematicskit.StepLimits;
import kinematicskit.Transform;
import kinematicskit.Vector3;

class StepLimitTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void { checks++; if (!ok) throw message; }
  static function close(a:Float, b:Float, message:String):Void
    check(Math.isFinite(a) && Math.abs(a - b) <= 1e-10 * Math.max(1e-12, Math.abs(b)), message);
  public static function main():Void {
    for (distance in [1e-9, 0.001, 1.0]) for (acceleration in [1e-6, 1.0, 1e12, Math.POSITIVE_INFINITY]) {
      var speed = StepLimits.brakingSpeed(acceleration, 0.01, distance);
      close(speed * 0.01 + speed * speed / (2 * acceleration), distance, "Cyclic speed satisfies travel plus braking across scales");
      var ramped = StepLimits.rampedBrakingSpeed(acceleration, 0.01, 0.0, distance);
      close(ramped * 0.005 + ramped * ramped / (2 * acceleration), distance, "Ramped speed satisfies travel plus braking across scales");
    }
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base"), tip = builder.addBody("tip");
    builder.addJoint("slide", JointKind.Prismatic, base, tip, Transform.identity(), Transform.identity(), new Vector3(0, 0, 1));
    var model = builder.build();
    var problem = new KinematicProblem(model).setLimits(0, -0.1, 0.1);
    for (ramped in [false, true]) for (acceleration in [0.0, Math.POSITIVE_INFINITY]) {
      var limits = new StepLimits();
      limits.velocity = [0.1]; limits.previousVelocity = [0.0]; limits.acceleration = [acceleration]; limits.ramped = ramped;
      var low = [0.0], high = [0.0];
      limits.bounds(problem, new KinematicState(model, [0.0]), 0.01, low, high);
      close(low[0], -0.001, "Unlimited acceleration retains the velocity lower bound");
      close(high[0], 0.001, "Unlimited acceleration retains the velocity upper bound");
      limits.bounds(problem, new KinematicState(model, [0.0999]), 0.01, low, high);
      close(high[0], ramped ? 0.0002 : 0.0001, "Unlimited acceleration retains the position stop");
    }
    // An endpoint inside the range is insufficient: a linear ramp may overshoot before turning.
    for (acceleration in [0.0, Math.POSITIVE_INFINITY]) {
      var limits = new StepLimits(); limits.ramped = true;
      limits.previousVelocity = [1.0]; limits.acceleration = [acceleration];
      var low = [0.0], high = [0.0];
      limits.bounds(problem, new KinematicState(model, [0.099]), 0.01, low, high);
      var endVelocity = high[0] / 0.01;
      var turnTravel = 1.0 * 1.0 * 0.01 / (2 * (1.0 - endVelocity));
      check(turnTravel <= 0.001000000001, "Unlimited ramp constrains the interior turning point");
    }
    var invalid = new StepLimits(); invalid.previousVelocity = [Math.NaN];
    var rejected = false;
    try invalid.bounds(problem, new KinematicState(model, [0.0]), 0.01, [0.0], [0.0]) catch (_:Dynamic) rejected = true;
    check(rejected, "Invalid previous velocity fails before producing QP bounds");
    Sys.println('Step limits: $checks assertions passed');
  }
}
