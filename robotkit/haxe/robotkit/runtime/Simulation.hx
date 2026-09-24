package robotkit.runtime;

import RobotKitSimKit;
import haxe.Int64;

/**
 * Owns one shared simulated universe and its fixed-step clock.
 *
 * The object creates RobotRuntime handles but remains their simulation owner:
 * callers should submit through those handles and advance this object once per
 * tick. It is intentionally separate from SimulatedRobot, which is only a
 * live Robot adapter for RobotWorld.
 */
class Simulation {
  final owner:Ownedrk_simulation;
  final robots:Array<RobotRuntime> = [];
  var disposed:Bool = false;

  public function new(?fixedTimestep:Float = 0.01, ?physicsSubsteps:Int = 1, ?backend:Int = 0) {
    var desc = new rk_simulation_desc();
    desc.set_struct_size(rk_simulation_desc.size());
    desc.set_fixed_timestep(fixedTimestep);
    desc.set_physics_substeps(physicsSubsteps);
    desc.set_backend(backend);
    var result = RobotKitSimKit.rk_simulation_create(desc);
    check(result.status, "simulation.create");
    owner = result.out_simulation;
  }

  /** Adds topology before the first start or step. */
  public function addRobot(blueprint:RobotRuntimeBlueprint):RobotRuntime {
    ensureLive();
    var result = RobotKitSimKit.rk_simulation_add_robot(owner.borrow(), blueprint.nativeValue());
    check(result.status, "simulation.addRobot");
    var runtime = new RobotRuntime(result.out_runtime, blueprint);
    robots.push(runtime);
    return runtime;
  }

  /** Advances once. timestampNs is a legacy hint, not source or receive time. */
  public function step(timestampNs:Int64):Void {
    ensureLive();
    check(RobotKitSimKit.rk_simulation_step(owner.borrow(), timestampNs), "simulation.step");
  }

  /** Starts the shared realtime clock after topology construction is complete. */
  public function start():Void {
    ensureLive();
    check(RobotKitSimKit.rk_simulation_start(owner.borrow()), "simulation.start");
  }

  /** Stops realtime stepping but leaves the simulation available for disposal. */
  public function stop():Void {
    if (!disposed) check(RobotKitSimKit.rk_simulation_stop(owner.borrow()), "simulation.stop");
  }

  /** Restores every body and the fixed-step clock to the editable-scene state. */
  public function reset():Void {
    ensureLive();
    stop();
    check(RobotKitSimKit.rk_simulation_reset(owner.borrow()), "simulation.reset");
  }

  /** Restores one robot's initial body pose and runtime state. */
  public function resetRobot(robotIndex:Int):Void {
    ensureLive();
    stop();
    check(RobotKitSimKit.rk_simulation_reset_robot(owner.borrow(), robotIndex),
      "simulation.resetRobot");
  }

  /** Teleports one robot base while leaving the shared clock untouched. */
  public function teleportRobot(robotIndex:Int, position:Array<Float>,
      ?rotation:Array<Float>):Void {
    ensureLive();
    if (position == null || position.length != 3)
      throw "Simulation.teleportRobot requires a three-component position";
    var pose = makePose(position, rotation);
    stop();
    check(RobotKitSimKit.rk_simulation_teleport_robot(owner.borrow(), robotIndex, pose),
      "simulation.teleportRobot");
  }

  /** Reads one robot base pose without mutating physics or the editable model. */
  public function robotPose(robotIndex:Int):{position:Array<Float>,rotation:Array<Float>} {
    ensureLive();var pose=new rk_simulation_pose();pose.set_struct_size(rk_simulation_pose.size());
    var result=RobotKitSimKit.rk_simulation_get_robot_pose(owner.borrow(),robotIndex,pose);
    check(result.status,"simulation.getRobotPose");
    return {position:[for(index in 0...3)pose.get_position(index)],
      rotation:[for(index in 0...4)pose.get_rotation(index)]};
  }

  /** Reads an articulated link pose without mutating the simulation. */
  public function linkPose(robotIndex:Int, linkIndex:Int):{position:Array<Float>,rotation:Array<Float>} {
    ensureLive();
    var pose = new rk_simulation_pose(); pose.set_struct_size(rk_simulation_pose.size());
    var result = RobotKitSimKit.rk_simulation_get_link_pose(owner.borrow(), robotIndex, linkIndex, pose);
    check(result.status, "simulation.getLinkPose");
    return poseValue(pose);
  }

  /** Reads an environment body's latest physics pose. */
  public function objectPose(objectId:Int):{position:Array<Float>,rotation:Array<Float>} {
    ensureLive();
    var pose = new rk_simulation_pose(); pose.set_struct_size(rk_simulation_pose.size());
    var result = RobotKitSimKit.rk_simulation_get_object_pose(owner.borrow(), objectId, pose);
    check(result.status, "simulation.getObjectPose");
    return poseValue(pose);
  }

  /** Copies every presentation body pose under one native simulation lock. */
  public function capturePresentation():SimulationPresentationSnapshot {
    ensureLive();
    var result = RobotKitSimKit.rk_simulation_capture_presentation(owner.borrow());
    check(result.status, "simulation.capturePresentation");
    try {
      return new SimulationPresentationSnapshot(result.out_presentation);
    } catch (error:Dynamic) {
      result.out_presentation.close();
      throw error;
    }
  }

  static function poseValue(pose:rk_simulation_pose):{position:Array<Float>,rotation:Array<Float>}
    return {position:[for(index in 0...3) pose.get_position(index)],
      rotation:[for(index in 0...4) pose.get_rotation(index)]};

  /** Adds a box to the shared physics world and returns its owned object ID. */
  public function spawnBox(position:Array<Float>, halfExtents:Array<Float>,
      ?dynamicBody:Bool = false, ?mass:Float = 1.0):Int {
    ensureLive();
    if (position == null || position.length != 3 || halfExtents == null || halfExtents.length != 3)
      throw "Simulation.spawnBox requires three-component position and extents";
    var desc = new rk_simulation_object_desc();
    desc.set_struct_size(rk_simulation_object_desc.size());
    desc.set_motion_type(dynamicBody ? 2 : 0);
    for (index in 0...3) {
      desc.set_position(index, position[index]);
      desc.set_half_extents(index, halfExtents[index]);
    }
    var chosenMass = dynamicBody ? mass : 0.0;
    desc.set_mass(chosenMass);
    var rotation = [0.0, 0.0, 0.0, 1.0];
    for (index in 0...4) desc.set_rotation(index, rotation[index]);
    stop();
    var result = RobotKitSimKit.rk_simulation_spawn_object(owner.borrow(), desc);
    check(result.status, "simulation.spawnBox");
    return result.out_object;
  }

  /** Removes an environment object from the shared scene and physics world. */
  public function removeObject(objectId:Int):Void {
    ensureLive();
    stop();
    check(RobotKitSimKit.rk_simulation_remove_object(owner.borrow(), objectId),
      "simulation.removeObject");
  }

  /** Teleports an environment object and clears its velocity. */
  public function teleportObject(objectId:Int, position:Array<Float>,
      ?rotation:Array<Float>):Void {
    ensureLive();
    if (position == null || position.length != 3)
      throw "Simulation.teleportObject requires a three-component position";
    var pose = makePose(position, rotation);
    stop();
    check(RobotKitSimKit.rk_simulation_teleport_object(owner.borrow(), objectId, pose),
      "simulation.teleportObject");
  }

  public function stepIndex():Int64 {
    var value = readClock();
    return value.get_step_index();
  }

  public function simulationTime():Float {
    var value = readClock();
    return value.get_simulation_time();
  }

  public function dispose():Void {
    if (disposed) return;
    stop();
    for (runtime in robots) runtime.dispose();
    robots.resize(0);
    owner.close();
    disposed = true;
  }

  function readClock():rk_simulation_clock {
    ensureLive();
    var value = new rk_simulation_clock();
    value.set_struct_size(rk_simulation_clock.size());
    var result = RobotKitSimKit.rk_simulation_get_clock(owner.borrow(), value);
    check(result.status, "simulation.getClock");
    return value;
  }

  function ensureLive():Void {
    if (disposed) throw "RobotKit simulation has been disposed";
  }

  function makePose(position:Array<Float>, ?rotation:Array<Float>):rk_simulation_pose {
    var pose = new rk_simulation_pose();
    pose.set_struct_size(rk_simulation_pose.size());
    for (index in 0...3) pose.set_position(index, position[index]);
    var chosen = rotation == null ? [0.0, 0.0, 0.0, 1.0] : rotation;
    if (chosen.length != 4)
      throw "Simulation pose rotation requires four components";
    for (index in 0...4) pose.set_rotation(index, chosen[index]);
    return pose;
  }

  static function check(status:Int, operation:String):Void {
    if (status != RobotKitRuntimeConstants.RK_OK) throw '$operation failed with RobotKit status $status';
  }
}
