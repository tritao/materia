package tests;

import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.Simulation;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.model.JointCoupling;
import haxe.Int64;
import robotkit.tool.ToolCollisionShape;
import robotkit.spatial.Vec3;

class SimulationPoseResetTests {
  public static function run(backend:Int):Void {
    var position = [1.25, -2.5, 0.75];
    var rotation = [Math.sin(0.3), 0.0, 0.0, Math.cos(0.3)];
    var simulation = new Simulation(0.01, 1, backend);
    var piece:Array<Float> = [];
    for (index in 0...8) {
      piece.push((index & 1) == 0 ? -0.01 : 0.01);
      piece.push((index & 2) == 0 ? -0.01 : 0.01);
      piece.push((index & 4) == 0 ? 0.0 : 0.02);
    }
    simulation.addRobotAtPose(new RobotRuntimeBlueprint(1, 0, 1), position, rotation,
      null, null, null, null, ToolCollisionShape.Hulls([piece], 0.005), 0);
    simulation.teleportRobot(0, [4.0, 5.0, 6.0]);
    simulation.resetRobot(0);
    checkPose(simulation, position, rotation, 'resetRobot on backend $backend');
    simulation.teleportRobot(0, [7.0, 8.0, 9.0]);
    simulation.reset();
    checkPose(simulation, position, rotation, 'reset on backend $backend');
    simulation.dispose();
    var boxSimulation = new Simulation(0.01, 1, backend);
    boxSimulation.addRobotAtPose(new RobotRuntimeBlueprint(2, 0, 1), [0, 0, 0],
      [0, 0, 0, 1], null, null, null, null,
      ToolCollisionShape.Box(new Vec3(0.01, 0.01, 0.02), new Vec3(0, 0, 0.02)), 0);
    boxSimulation.step(Int64.ofInt(0));
    boxSimulation.dispose();
    if (backend == 1) toolProximity();
    coupling(backend);
  }

  static function toolProximity():Void {
    var simulation = new Simulation(0.01, 2, 1);
    var cup:Array<Float> = [];
    for (index in 0...8) {
      cup.push((index & 1) == 0 ? -0.01 : 0.01);
      cup.push((index & 2) == 0 ? -0.01 : 0.01);
      cup.push((index & 4) == 0 ? 0.0 : 0.02);
    }
    var runtime = simulation.addRobotAtPose(new RobotRuntimeBlueprint(1, 0, 1),
      [0, 0, 0], [0, 0, 0, 1], null, null, null, null,
      ToolCollisionShape.Hulls([cup], 0.03), 0);
    var obstacle = simulation.spawnBox([0, 0, 0.05], [0.01, 0.01, 0.01]);
    simulation.step(Int64.ofInt(0));
    var contacts = runtime.toolProximity();
    if (contacts.length == 0 || contacts[0].toolPieceIndex != 0 || contacts[0].active ||
        contacts[0].otherObject != obstacle)
      throw "Tool cup proximity was not reported";
    simulation.dispose();
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
    model.addCoupling(new JointCoupling("gears", source.id, follower.id, -2.0, 0.0));
    var simulation = new Simulation(0.01, 1, backend);
    var runtime = simulation.addRobot(RobotRuntimeCompiler.compile(model));
    runtime.submitPosition(0, 0.3, 1);
    for (index in 0...200) simulation.step(Int64.ofInt(index));
    var q = runtime.snapshot().q;
    var tolerance = backend == 0 ? 1e-9 : 0.02;
    if (!(Math.abs(q.get(0)) > 0.1 && Math.abs(q.get(1) + 2.0 * q.get(0)) < tolerance))
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
