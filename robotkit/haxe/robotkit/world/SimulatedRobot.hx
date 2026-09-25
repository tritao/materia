package robotkit.world;

import robotkit.runtime.RobotRuntime;

/** RobotWorld adapter over a runtime owned by an externally managed Simulation. */
class SimulatedRobot extends RuntimeRobotAdapter {
  public function new(id:RobotId, runtime:RobotRuntime, name:String,
      links:Array<String>, joints:Array<String>) {
    super(id, runtime, name, links, joints, false, false, "simulated runtime fault");
  }
}
