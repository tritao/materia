package app.editor;

import humankit.Pick;
import humankit.Place;
import humankit.facility.FacilityJobs;
import humankit.sim.HumanWorker;
import humankit.sim.HumanZone;
import nativekit.sim.SimObject;
import robotkit.runtime.Simulation;
import robotkit.model.RobotModel;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.JointTarget;

/** Rack-to-table acceptance scene wiring and robot motion. */
class WorkerDemoJob {
  public static inline var ROBOT_ID = "materia/robot";

  public static function configure(worker:HumanWorker, objects:Array<{id:String,object:SimObject}>,
      simulation:Simulation, robotIndex:Int, model:RobotModel):Void {
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
    for (bound in RobotLinkBounds.shapes(model)) {
      var linkIndex = bound.link, offset = bound.offset;
      var id = linkIndex == 1 && bound.shape == 0 ? "demo-arm" : 'demo-link-$linkIndex-${bound.shape}';
      worker.addRobotLinkPose(id, function() {
        var link = simulation.linkPose(robotIndex, linkIndex);
        return RobotLinkBounds.worldCenter(link.position, link.rotation, offset);
      }, bound.radius);
    }
    worker.run(job);
  }

  public static function cycle(robot:Robot, time:Float):Void
    robot.submit(RobotCommand.JointTargets(
      [JointTarget.position(0, 0.6 * Math.sin(time * 2.0))], null));
}
