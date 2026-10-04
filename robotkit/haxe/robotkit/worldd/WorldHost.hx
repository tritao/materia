package robotkit.worldd;

import haxe.Int64;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.SimulationHarness;
import robotkit.remote.RemoteRobot;
import robotkit.world.RobotWorld;
import robotkit.simulation.SimulatedRobot;
import robotkit.deployment.PerceptionPipelineConfig;
import robotkit.deployment.SerialDeployment;
import robotkit.perception.PerceptionHost;
import robotkit.perception.PerceptionPipelineRegistry;

/** Headless composition of the existing RobotWorld and a shared Simulation
 * this host owns end to end (through a SimulationHarness). */
class WorldHost {
  public final world:RobotWorld;
  public final simulation:SimulationHarness;
  var closed:Bool = false;
  final perceptionHosts:Map<String, PerceptionHost> = new Map<String, PerceptionHost>();
  final remoteRobots:Map<String, RemoteRobot> = new Map<String, RemoteRobot>();
  final perceptionSequences:Map<String, haxe.Int64> = new Map<String, haxe.Int64>();

  public function new(?fixedTimestep:Float = 0.01, ?physicsSubsteps:Int = 1) {
    world = new RobotWorld();
    simulation = new SimulationHarness(fixedTimestep, physicsSubsteps);
  }

  public function addSimulatedRobot(id:String, model:RobotModel):SimulatedRobot {
    ensureOpen();
    var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    var runtime = simulation.simulation.addRobot(blueprint);
    var robot = new SimulatedRobot(id, runtime, model.name,
      [for (link in model.links) link.name], [for (joint in model.joints) joint.name]);
    world.attach(robot);
    return robot;
  }

  public function addRemoteRobot(id:String, ?camera:Bool = false,
      ?deployment:SerialDeployment):RemoteRobot {
    ensureOpen();
    var robot = new RemoteRobot(id);
    if (camera) robot.enableCamera();
    world.attach(robot);
    remoteRobots.set(id, robot);
    if (deployment != null) configurePerception(robot, deployment.perception);
    return robot;
  }

  /** Enables pipelines placed at worldd and requests their remote camera inputs. */
  public function configurePerception(robot:RemoteRobot, configs:Array<PerceptionPipelineConfig>):Void {
    ensureOpen();
    if (remoteRobots.get(robot.id()) != robot) throw "WorldHost does not own this remote robot";
    var pipelines:Array<robotkit.perception.PerceptionPipeline> = [];
    for (config in configs) if (config.host == "worldd") {
      robot.enableCamera();
      pipelines.push(PerceptionPipelineRegistry.create(config, "worldd/" + config.id));
    }
    var previous = perceptionHosts.get(robot.id());
    if (previous != null) previous.dispose();
    if (pipelines.length > 0) perceptionHosts.set(robot.id(), new PerceptionHost(pipelines));
    else perceptionHosts.remove(robot.id());
  }

  /** Runs one shared deterministic tick and returns the immutable world view. */
  public function step(timestampNs:Int64):robotkit.world.WorldSnapshot {
    ensureOpen();
    simulation.step(timestampNs);
    for (id in perceptionHosts.keys()) if (world.robot(id) != remoteRobots.get(id)) {
      perceptionHosts.get(id).dispose();
      perceptionHosts.remove(id);
      remoteRobots.remove(id);
    }
    for (id in world.robotIds()) {
      var host = perceptionHosts.get(id);
      if (host == null) continue;
      var robot = remoteRobots.get(id);
      if (robot == null) continue;
      for (sensor in robot.sensors()) if (sensor.image != null) {
        var key = id + ":" + sensor.sensorId;
        var last = perceptionSequences.get(key);
        // A remote camera may restart its sequence after the device reconnects.
        if (last == null || Int64.compare(sensor.sequence, last) != 0) {
          perceptionSequences.set(key, sensor.sequence);
          host.submit(sensor);
        }
      }
      for (observation in host.poll()) {
        robot.publishObservation(observation);
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
