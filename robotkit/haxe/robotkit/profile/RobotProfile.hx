package robotkit.profile;

/** Interpretation and task roles beside a mechanical model, authored with stable joint IDs.
    A mobile manipulator or forklift may compose multiple roles in the same profile. */
class RobotProfile {
  public static inline final CURRENT_VERSION:Int = 1;
  public var mobileBase:Null<RobotMobileConfiguration>;
  public var forkMechanism:Null<RobotForkConfiguration>;
  public function new(?mobileBase:RobotMobileConfiguration, ?forkMechanism:RobotForkConfiguration) {
    this.mobileBase = mobileBase;
    this.forkMechanism = forkMechanism;
  }
}
