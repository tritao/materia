package robotkit.runtime;

import collisionkit.CollisionGeometry;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionPose;
import collisionkit.CollisionWorld;
import haxe.Int64;
import NativeKitSim;
import nativekit.sim.SimObject;
import robotkit.mobile.Pose3;

/** Another controller's mechanism whose links a robot zone watches. */
typedef RobotZoneMechanism = {
  var robotIndex:Int;
  var shapes:Array<RobotRuntimeLinkShape>;
}

/**
 * Samples a controller's `zone_input` sensors after each physics tick, like a safety area scanner:
 * a zone is occupied while a link of another robot, or a part one of them carries, comes within
 * `margin` of any of its boxes. The links are their collision hulls and a carried part its simulated shape.
 * The zone's own machine and parts lying free (the stock in the vise) never occupy it.
 */
class SimulatedRobotZoneAdapter implements SimulationStepObserver {
  final simulation:Simulation;
  final runtime:RobotRuntime;
  final robotIndex:Int;
  final margin:Float;
  final world:CollisionWorld;
  final sensors:Array<RobotRuntimeSensorBlueprint>;
  final zones:Array<Int> = [];
  final links:Array<{robotIndex:Int, link:Int, body:Int, carrier:nksim_body}> = [];
  final objects:Array<{object:SimObject, body:Int}> = [];
  final carried:Array<Bool> = [];
  var sequence:Int64 = Int64.ofInt(0);

  public function new(simulation:Simulation, runtime:RobotRuntime, blueprint:RobotRuntimeBlueprint, robotIndex:Int,
      mechanisms:Array<RobotZoneMechanism>, objects:Array<SimObject>, margin:Float, makeWorld:Void->CollisionWorld) {
    if (simulation == null || runtime == null || blueprint == null || robotIndex < 0 || mechanisms == null ||
        objects == null || !(margin >= 0.0) || !Math.isFinite(margin) || makeWorld == null)
      throw "Simulated robot zones need a simulation, runtime, blueprint, robots, scene objects and a margin";
    this.simulation = simulation;
    this.runtime = runtime;
    this.robotIndex = robotIndex;
    this.margin = margin;
    world = makeWorld();
    sensors = [for (sensor in blueprint.externalSensorLayout()) if (sensor.kind == "zone_input") sensor];
    for (sensor in sensors) {
      var boxes = sensor.zoneBoxes;
      if (sensor.link < 0 || sensor.link >= blueprint.linkCount || boxes == null || boxes.length == 0 ||
          boxes.length % 6 != 0)
        throw 'Robot zone "${sensor.id}" has an invalid mount or boxes';
      // One body per zone: its field is the union of its boxes.
      var zone = world.addBody(0);
      for (box in 0...Std.int(boxes.length / 6))
        world.add(zone, new CollisionPose(boxes.get(6 * box), boxes.get(6 * box + 1), boxes.get(6 * box + 2), 0, 0, 0, 1),
          CollisionGeometry.Box(boxes.get(6 * box + 3), boxes.get(6 * box + 4), boxes.get(6 * box + 5)));
      zones.push(zone);
    }
    for (mechanism in mechanisms) {
      if (mechanism.robotIndex == robotIndex) throw "A robot zone does not watch its own machine";
      var bodies = new Map<Int, Int>();
      for (hull in mechanism.shapes) {
        var body = bodies.get(hull.link);
        if (body == null) {
          body = world.addBody(1);
          bodies.set(hull.link, body);
          links.push({robotIndex: mechanism.robotIndex, link: hull.link, body: body,
            carrier: simulation.linkBody(mechanism.robotIndex, hull.link)});
        }
        world.add(body, CollisionPose.identity(), CollisionGeometry.Convex(hull.shape.convexVertices()));
      }
    }
    for (object in objects) {
      var points = object.shape.convexVertices();
      var body = world.addBody(2);
      if (points.length > 0) world.add(body, CollisionPose.identity(), CollisionGeometry.Convex(points));
      else {
        var half = object.shape.boundingHalfExtents();
        if (half == null) throw "A robot zone needs finite geometry for every part a robot may carry";
        world.add(body, CollisionPose.identity(), CollisionGeometry.Box(half.x, half.y, half.z));
      }
      this.objects.push({object: object, body: body});
      carried.push(false);
    }
    // Only a zone and what may enter it are compared; a free part never occupies a zone.
    var watched = [for (link in links) link.body].concat([for (entry in this.objects) entry.body]);
    for (i in 0...watched.length) for (j in i + 1...watched.length)
      world.setBodyRule(watched[i], watched[j], CollisionPairRule.Allow, CollisionPairStatus.Rigid);
    for (i in 0...zones.length) for (j in i + 1...zones.length)
      world.setBodyRule(zones[i], zones[j], CollisionPairRule.Allow, CollisionPairStatus.Rigid);
    for (zone in zones) for (entry in this.objects)
      world.setBodyRule(zone, entry.body, CollisionPairRule.Allow, CollisionPairStatus.Rigid);
  }

  public function afterSimulationStep(sourceTimestampNs:Int64):Void {
    if (sensors.length == 0) return;
    sequence = Int64.add(sequence, Int64.ofInt(1));
    for (index in 0...sensors.length) {
      var sensor = sensors[index];
      var link = simulation.linkPose(robotIndex, sensor.link);
      var parent = new Pose3(link.position[0], link.position[1], link.position[2],
        link.rotation[0], link.rotation[1], link.rotation[2], link.rotation[3]);
      var mount = parent.compose(new Pose3(sensor.position.get(0), sensor.position.get(1), sensor.position.get(2),
        sensor.rotation.get(0), sensor.rotation.get(1), sensor.rotation.get(2), sensor.rotation.get(3)));
      world.setBodyPose(zones[index], new CollisionPose(mount.x, mount.y, mount.z, mount.qx, mount.qy, mount.qz, mount.qw));
    }
    for (link in links) {
      var pose = simulation.linkPose(link.robotIndex, link.link);
      world.setBodyPose(link.body, new CollisionPose(pose.position[0], pose.position[1], pose.position[2],
        pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]));
    }
    for (index in 0...objects.length) {
      var entry = objects[index];
      var carrier = simulation.objectCarrier(entry.object), held = false;
      for (link in links) if (link.carrier == carrier) held = true;
      if (held != carried[index]) {
        carried[index] = held;
        for (zone in zones) world.setBodyRule(zone, entry.body,
          held ? CollisionPairRule.Default : CollisionPairRule.Allow, CollisionPairStatus.Rigid);
      }
      if (held) {
        var pose = simulation.objectPose(entry.object);
        world.setBodyPose(entry.body, new CollisionPose(pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw));
      }
    }
    var occupied = [for (_ in zones) false];
    for (pair in world.colliding(margin)) for (index in 0...zones.length)
      if (pair.bodyA == zones[index] || pair.bodyB == zones[index]) occupied[index] = true;
    for (index in 0...sensors.length)
      runtime.publishSensorFrame(sensors[index].id, [occupied[index] ? 1.0 : 0.0], sequence, sourceTimestampNs,
        runtime.sourceClockId);
  }

  public function dispose():Void world.dispose();
}
