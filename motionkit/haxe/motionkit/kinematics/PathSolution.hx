package motionkit.kinematics;

/**
 * A solved path: one configuration per sample (null where unreachable), and for a redundant solver
 * how its redundancy changes along it, d(values)/ds per sample in the order of the solver's
 * redundancy values (a swivel, external axes), exact from the curve the resolver chose rather than
 * differenced from the solved samples; null when the solver has no redundancy or the route has no
 * such curve.
 */
class PathSolution {
  public final configurations:Array<Null<Array<Float>>>;
  public final redundancyRates:Null<Array<Array<Float>>>;

  public function new(configurations:Array<Null<Array<Float>>>, ?redundancyRates:Array<Array<Float>>) {
    this.configurations = configurations;
    this.redundancyRates = redundancyRates;
  }
}
