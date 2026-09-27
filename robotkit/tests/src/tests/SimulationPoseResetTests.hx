package tests;

import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.Simulation;

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
