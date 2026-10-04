package tests;
import robotkit.auth.RobotAuthorization;
class RobotAuthorizationTests {
  static var assertions:Int;
  static function check(value:Bool):Void { assertions++; if (!value) throw "Robot authorization contract failed"; }
  static function refused(call:Void->Void):Bool { try call() catch (_:Dynamic) return true; return false; }
  public static function run():Int {
    assertions = 0;
    var path = Sys.getCwd()+"/robotkit/tests/fixtures/authorization.json";
    if (!sys.FileSystem.exists(path)) path = Sys.getCwd()+"/fixtures/authorization.json";
    var auth = new RobotAuthorization(path);
    var observer = auth.authenticate("test-observer","robotkit-test-observer-token-0001");
    var controller = auth.authenticate("test-controller","robotkit-test-controller-token-0001");
    var deployer = auth.authenticate("test-deployer","robotkit-test-deployer-token-0001");
    check(observer.mayObserve && !observer.mayCommand && !observer.mayDeploy);
    check(controller.mayObserve && controller.mayCommand && !controller.mayDeploy);
    check(deployer.mayObserve && !deployer.mayCommand && deployer.mayDeploy);
    check(refused(function() auth.authenticate("test-observer","robotkit-test-controller-token-0001")));
    check(refused(function() auth.authenticate("unknown","robotkit-test-observer-token-0001")));
    check(refused(function() auth.authenticate("","")));
    check(refused(function() auth.deploymentPath(controller,"bench")));
    check(refused(function() auth.deploymentPath(observer,"bench")));
    check(refused(function() auth.deploymentPath(deployer,"../escape")));
    check(StringTools.endsWith(auth.deploymentPath(deployer,"bench"),"device-deployment/deployment.json"));
    check(refused(function() new RobotAuthorization(null)));
    return assertions;
  }
}
