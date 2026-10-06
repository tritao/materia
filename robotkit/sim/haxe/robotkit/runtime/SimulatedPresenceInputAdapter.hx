package robotkit.runtime;

import haxe.Int64;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import robotkit.mobile.Pose3;

/** Samples authored cabinet presence inputs from free-object geometry after each physics tick. */
class SimulatedPresenceInputAdapter implements SimulationStepObserver {
  final simulation:Simulation;
  final runtime:RobotRuntime;
  final robotIndex:Int;
  final sensors:Array<RobotRuntimeSensorBlueprint>;
  final objects:Array<SimObject>;
  var sequence:Int64 = Int64.ofInt(0);

  public function new(simulation:Simulation, runtime:RobotRuntime, blueprint:RobotRuntimeBlueprint,
      robotIndex:Int, objects:Array<SimObject>) {
    if (simulation == null || runtime == null || blueprint == null || robotIndex < 0 || objects == null)
      throw "Simulated presence inputs need a simulation, runtime, blueprint, robot and scene objects";
    this.simulation = simulation;
    this.runtime = runtime;
    this.robotIndex = robotIndex;
    this.objects = objects.copy();
    sensors = [for (sensor in blueprint.externalSensorLayout()) if (sensor.kind == "presence_input") sensor];
    var ids = new Map<String, Bool>();
    for (sensor in sensors) {
      if (sensor.id == null || sensor.id.length == 0 || ids.exists(sensor.id) ||
          sensor.link < 0 || sensor.link >= blueprint.linkCount ||
          !Math.isFinite(sensor.maxRange) || sensor.maxRange <= 0.0)
        throw 'Presence input "${sensor.id}" has an invalid mount or sensing volume';
      ids.set(sensor.id, true);
    }
  }

  public function afterSimulationStep(sourceTimestampNs:Int64):Void {
    sequence = Int64.add(sequence, Int64.ofInt(1));
    for (sensor in sensors) {
      var link = simulation.linkPose(robotIndex, sensor.link);
      var parent = new Pose3(link.position[0], link.position[1], link.position[2],
        link.rotation[0], link.rotation[1], link.rotation[2], link.rotation[3]);
      var local = new Pose3(sensor.position.get(0), sensor.position.get(1), sensor.position.get(2),
        sensor.rotation.get(0), sensor.rotation.get(1), sensor.rotation.get(2), sensor.rotation.get(3));
      var mount = parent.compose(local);
      var direction = mount.rotate(0.0, 0.0, 1.0);
      var range = sensor.maxRange;
      var volume = new SimPose(mount.x + direction.x * range * 0.5,
        mount.y + direction.y * range * 0.5, mount.z + direction.z * range * 0.5,
        mount.qx, mount.qy, mount.qz, mount.qw);
      var halfExtents = [range, range, range * 0.5];
      var present = false;
      for (object in objects) if (object.overlapsVolume(volume, halfExtents)) {
        present = true;
        break;
      }
      runtime.publishSensorFrame(sensor.id, [present ? 1.0 : 0.0], sequence, sourceTimestampNs,
        "robotkit.simulation");
    }
  }
}
