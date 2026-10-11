package tests;

import haxe.Int64;
import robotkit.core.JointTarget;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.profile.RobotProfile;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.SimulationHarness;

/** Passive ownership is checked on both backends; MuJoCo additionally resolves hinge gravity. */
class PassiveJointRuntimeTests {
  public static function main():Void {
    var total = run(0);
    Sys.println('Passive joint runtime tests passed ($total assertions)');
  }
  public static function run(backend:Int):Int {
    var assertions = 0;
    for (passive in [false, true]) {
      var model = new RobotModel("gravity-finger");
      var base = model.addLink(new Link("base"));
      var finger = model.addLink(new Link("finger"));
      finger.mass = 0.1884;
      finger.centerOfMass = [0, -0.02, 0.01];
      finger.inertiaTensor = [0.00008, 0, 0, 0, 0.00008, 0, 0, 0, 0.00008];
      var hinge = model.addJoint(new Joint("hinge", JointType.Revolute, base, finger));
      hinge.axis = [-1, 0, 0];
      hinge.parentFramePosition = [0, 0, 1];
      hinge.limits.lower = -0.05; hinge.limits.upper = 0;
      hinge.damping = 0.01;
      hinge.passive = passive;
      var blueprint = RobotRuntimeCompiler.compile(model, new RobotProfile());
      var harness = new SimulationHarness(0.001, 1, backend);
      try {
        var runtime = harness.simulation.addRobotAtPose(blueprint, [0, 0, 0], [0, 0, 0, 1],
          null, null, null, null, null, null, null, true);
        if (passive) {
          var rejected = false;
          try runtime.submitTargets([JointTarget.position(0, 0)], 1)
          catch (_:Dynamic) rejected = true;
          if (!rejected) throw 'Passive hinge accepted a host target backend $backend';
          assertions++;
        }
        for (tick in 1...501) harness.step(Int64.ofInt(tick * 1000000));
        var measured = runtime.snapshot().q.get(0);
        if (!Math.isFinite(measured) || (passive && backend == 1 ? measured > -0.04 : Math.abs(measured) > 0.001))
          throw 'Gravity hinge passive=$passive backend=$backend measured=$measured';
        assertions++;
        harness.dispose();
      } catch (error:Dynamic) {
        harness.dispose();
        throw error;
      }
    }
    return assertions;
  }
}
