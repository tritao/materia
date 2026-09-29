package app.editor;

import humankit.Pick;
import humankit.Place;
import humankit.facility.FacilityJobs;
import humankit.sim.HumanWorker;
import humankit.sim.HumanZone;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import robotkit.runtime.Simulation;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.JointTarget;

/** Rack-to-table acceptance scene wiring and robot motion. */
class WorkerDemoJob {
  public static inline var ROBOT_ID = "materia/robot";

  public static function configure(worker:HumanWorker, objects:Array<{id:String,object:SimObject}>,
      simulation:Simulation, robotIndex:Int):Void {
    var part:Null<SimObject> = null;
    for (entry in objects) if (entry.id == "worker-demo-part") part = entry.object;
    if (part == null) throw "Worker demo part is missing from the scene";
    if (robotIndex < 0) throw "Worker demo robot is missing from the scene";
    var job = FacilityJobs.fetch(FacilityRouteDemo.demoFacility(), "shelf", "B3")
      .deliver("bench", [3.75, 2.7, 1.0]);
    for (action in job.orderedActions()) {
      if (Std.isOfType(action, Pick)) {
        var pick:Pick = cast action;
        worker.bindPick(pick, part, pick.target);
      } else if (Std.isOfType(action, Place)) {
        var place:Place = cast action;
        worker.bindPlace(place, part, place.target);
      }
    }
    worker.addZone(new HumanZone("rack", [[2.6, -0.7], [4.0, -0.7], [4.0, 0.5], [2.6, 0.5]]));
    worker.addZone(new HumanZone("table", [[2.6, 1.9], [4.0, 1.9], [4.0, 3.1], [2.6, 3.1]]));
    // The jointed robot's collision sphere is 45 cm from its link origin.
    worker.addRobotLinkPose("demo-arm", function() {
      var link = simulation.linkPose(robotIndex, 1);
      var q = link.rotation;
      var x = 0.45 * (1 - 2 * (q[1] * q[1] + q[2] * q[2]));
      var y = 0.9 * (q[0] * q[1] + q[2] * q[3]);
      var z = 0.9 * (q[0] * q[2] - q[1] * q[3]);
      return new SimPose(link.position[0] + x, link.position[1] + y,
        link.position[2] + z);
    }, 0.12);
    worker.run(job);
  }

  public static function cycle(robot:Robot, time:Float):Void
    robot.submit(RobotCommand.JointTargets(
      [JointTarget.position(0, 0.6 * Math.sin(time * 2.0))], null));
}
