package robotkit.auth;
/** Authenticated identity and independent authorization grants. */
class AuthPrincipal {
  public final identity:String;
  public final mayObserve:Bool;
  public final mayCommand:Bool;
  public final mayDeploy:Bool;
  @:allow(robotkit.auth.RobotAuthorization)
  function new(identity:String, permissions:Array<String>) {
    this.identity = identity;
    mayObserve = permissions.contains("observe");
    mayCommand = permissions.contains("command");
    mayDeploy = permissions.contains("deployment");
  }
}
