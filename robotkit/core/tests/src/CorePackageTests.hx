import haxe.Int64;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.model.RobotModelCodec;
import robotkit.profile.RobotProfile;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeCompiler;

/** This consumer deliberately depends only on robotkit-core. */
class CorePackageTests {
  static function main():Void {
    var model = new RobotModel("core-only");
    var base = model.addLink(new Link("base"));
    var tip = model.addLink(new Link("tip"));
    var joint = model.addJoint(new Joint("axis", JointType.Revolute, base, tip));
    joint.limits = new JointLimits(-1.0, 1.0, 1.0, 2.0, 3.0);
    var decoded = RobotModelCodec.decode(RobotModelCodec.encode(model));
    var blueprint = RobotRuntimeCompiler.compile(decoded, new RobotProfile());
    var runtime = RobotRuntime.create(blueprint);
    var capabilities = runtime.capabilities("core-only");
    if (capabilities.jointCount != 1 || !capabilities.execution.plans)
      throw "Core-only runtime did not retain joint and execution contracts";
    runtime.dispose();
    Sys.println("RobotKit core-only package test passed");
  }
}
