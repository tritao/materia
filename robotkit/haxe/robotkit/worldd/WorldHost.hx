package robotkit.worldd;

import haxe.Int64;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.SimulationHarness;
import robotkit.world.RemoteRobot;
import robotkit.world.RobotWorld;
import robotkit.world.SimulatedRobot;
import robotkit.deployment.PerceptionPipelineConfig;
import robotkit.perception.PerceptionHost;
import robotkit.perception.PerceptionPipelineRegistry;

/** Headless composition of the existing RobotWorld and a shared Simulation
 * this host owns end to end (through a SimulationHarness). */
class WorldHost {
  public final world:RobotWorld;
  public final simulation:SimulationHarness;
  var closed:Bool = false;
  final perceptionHosts:Map<String, PerceptionHost> = new Map<String, PerceptionHost>();
  final perceptionSequences:Map<String, haxe.Int64> = new Map<String, haxe.Int64>();

  public function new(?fixedTimestep:Float = 0.01, ?physicsSubsteps:Int = 1) {
    world = new RobotWorld();
    simulation = new SimulationHarness(fixedTimestep, physicsSubsteps);
  }

  public function addSimulatedRobot(id:String, model:RobotModel):SimulatedRobot {
    ensureOpen();
    var blueprint = RobotRuntimeCompiler.compile(model);
    var runtime = simulation.simulation.addRobot(blueprint);
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

  /** Enables pipelines placed at worldd and requests their remote camera inputs. */
  public function configurePerception(robot:RemoteRobot, configs:Array<PerceptionPipelineConfig>):Void {
    ensureOpen();
    var pipelines:Array<robotkit.perception.PerceptionPipeline> = [];
    for (config in configs) if (config.host == "worldd") {
      pipelines.push(PerceptionPipelineRegistry.create(config, "worldd/" + config.id));
      robot.enableCamera();
    }
    if (pipelines.length > 0) perceptionHosts.set(robot.id(), new PerceptionHost(pipelines));
  }

  /** Runs one shared deterministic tick and returns the immutable world view. */
  public function step(timestampNs:Int64):robotkit.world.WorldSnapshot {
    ensureOpen();
    simulation.step(timestampNs);
    for (id in world.robotIds()) {
      var host = perceptionHosts.get(id);
      if (host == null) continue;
      var robot = world.robot(id);
      if (robot == null) continue;
      for (sensor in robot.sensors()) if (sensor.image != null) {
        var key = id + ":" + sensor.sensorId;
        var last = perceptionSequences.get(key);
        if (last == null || Int64.compare(sensor.sequence, last) > 0) {
          perceptionSequences.set(key, sensor.sequence);
          host.submit(sensor);
        }
      }
      for (observation in host.poll()) {
        var remote:RemoteRobot = cast robot;
        remote.publishObservation(observation);
      }
    }
    return world.snapshot();
  }

  public function close():Void {
    if (closed) return;
    closed = true;
    for (host in perceptionHosts) host.dispose();
    world.close();
    simulation.dispose();
  }

  function ensureOpen():Void if (closed) throw "WorldHost has been closed";
}
