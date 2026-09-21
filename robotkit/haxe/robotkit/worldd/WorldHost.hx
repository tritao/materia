package robotkit.worldd;

import haxe.Int64;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.world.RemoteRobot;
import robotkit.world.RobotWorld;
import robotkit.world.SimulatedRobot;

/** Headless composition of the existing RobotWorld and shared Simulation. */
class WorldHost {
  public final world:RobotWorld;
  public final simulation:Simulation;
  var closed:Bool = false;

  public function new(?fixedTimestep:Float = 0.01, ?physicsSubsteps:Int = 1) {
    world = new RobotWorld();
    simulation = new Simulation(fixedTimestep, physicsSubsteps);
  }

  public function addSimulatedRobot(id:String, model:RobotModel):SimulatedRobot {
    ensureOpen();
    var blueprint = RobotRuntimeCompiler.compile(model);
    var runtime = simulation.addRobot(blueprint);
    var robot = new SimulatedRobot(id, runtime, model.name,
      [for (link in model.links) link.name], [for (joint in model.joints) joint.name]);
    world.attach(robot);
    return robot;
  }

  public function addRemoteRobot(id:String):RemoteRobot {
    ensureOpen();
    var robot = new RemoteRobot(id);
    world.attach(robot);
    return robot;
  }

  /** Runs one shared deterministic tick and returns the immutable world view. */
  public function step(timestampNs:Int64):robotkit.world.WorldSnapshot {
    ensureOpen();
    simulation.step(timestampNs);
    return world.snapshot();
  }

  public function close():Void {
    if (closed) return;
    closed = true;
    world.close();
    simulation.dispose();
  }

  function ensureOpen():Void if (closed) throw "WorldHost has been closed";
}
