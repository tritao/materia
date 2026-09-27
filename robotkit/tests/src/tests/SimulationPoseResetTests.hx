package tests;

import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.Simulation;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import haxe.Int64;

class SimulationPoseResetTests {
  public static function run(backend:Int):Void {
    var position = [1.25, -2.5, 0.75];
    var rotation = [Math.sin(0.3), 0.0, 0.0, Math.cos(0.3)];
    var simulation = new Simulation(0.01, 1, backend);
    simulation.addRobotAtPose(new RobotRuntimeBlueprint(1, 0, 1), position, rotation);
    simulation.teleportRobot(0, [4.0, 5.0, 6.0]);
    simulation.resetRobot(0);
    checkPose(simulation, position, rotation, 'resetRobot on backend $backend');
    simulation.teleportRobot(0, [7.0, 8.0, 9.0]);
    simulation.reset();
    checkPose(simulation, position, rotation, 'reset on backend $backend');
    simulation.dispose();
    coupling(backend);
  }

  static function coupling(backend:Int):Void {
    var model = new RobotModel("coupled joints");
    var base = model.addLink(new Link("base"));
    var first = model.addLink(new Link("first"));
    var second = model.addLink(new Link("second"));
    var source = model.addJoint(new Joint("source", JointType.Revolute, base, first));
    var follower = model.addJoint(new Joint("follower", JointType.Revolute, base, second));
    source.limits = new JointLimits(-2, 2);
    follower.limits = new JointLimits(-2, 2);
    var simulation = new Simulation(0.01, 1, backend);
    var runtime = simulation.addRobot(RobotRuntimeCompiler.compile(model));
    simulation.setJointCoupling(0, 0, 1, -2.0, 0.0);
    runtime.submitPosition(0, 0.3, 1);
    for (index in 0...200) simulation.step(Int64.ofInt(index));
    var q = runtime.snapshot().q;
    if (!(Math.abs(q.get(0)) > 0.1 && Math.abs(q.get(1) + 2.0 * q.get(0)) < 0.08))
      throw 'coupling did not follow source on backend $backend: ${q.get(0)}, ${q.get(1)}';
    simulation.dispose();
  }

  static function checkPose(simulation:Simulation, position:Array<Float>, rotation:Array<Float>,
      operation:String):Void {
    var actual = simulation.robotPose(0);
    for (axis in 0...3)
      if (!(Math.abs(actual.position[axis] - position[axis]) < 1e-6))
        throw '$operation did not restore position axis $axis';
    for (axis in 0...4)
      if (!(Math.abs(actual.rotation[axis] - rotation[axis]) < 1e-6))
        throw '$operation did not restore rotation axis $axis';
  }
}
