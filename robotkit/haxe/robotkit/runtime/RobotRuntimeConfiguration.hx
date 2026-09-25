package robotkit.runtime;

/** User-layer semantic configuration compiled alongside native runtime data. */
class RobotRuntimeConfiguration {
  public final mobileBase:Null<RobotRuntimeMobileConfiguration>;
  public final forks:Null<RobotRuntimeForkConfiguration>;

  public function new(mobileBase:Null<RobotRuntimeMobileConfiguration>, forks:Null<RobotRuntimeForkConfiguration>) {
    this.mobileBase = mobileBase;
    this.forks = forks;
  }
}
