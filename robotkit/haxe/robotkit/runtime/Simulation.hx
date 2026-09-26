package robotkit.runtime;

import RobotKitSimKit;
import haxe.Int64;
import robotkit.mobile.Pose2;

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
  public final fixedTimestepSeconds:Float;
  var disposed:Bool = false;

  public function new(?fixedTimestep:Float = 0.01, ?physicsSubsteps:Int = 1, ?backend:Int = 0) {
    if (!Math.isFinite(fixedTimestep) || fixedTimestep <= 0.0 || physicsSubsteps <= 0)
      throw "Simulation requires a positive finite timestep and positive substep count";
    fixedTimestepSeconds = fixedTimestep;
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
  public function addRobot(blueprint:RobotRuntimeBlueprint, ?initialPose:Pose2):RobotRuntime {
    ensureLive();
    var robotDesc:Null<rk_simulation_robot_desc> = null;
    if (initialPose != null) {
      robotDesc = new rk_simulation_robot_desc();
      robotDesc.set_struct_size(rk_simulation_robot_desc.size());
      robotDesc.set_initial_struct_size(rk_simulation_pose.size());
      var pose = makePose([initialPose.x, initialPose.y, 0.0],
        [0.0, 0.0, Math.sin(initialPose.yaw * 0.5), Math.cos(initialPose.yaw * 0.5)]);
      for (index in 0...3) robotDesc.set_initial_position(index, pose.get_position(index));
      for (index in 0...4) robotDesc.set_initial_rotation(index, pose.get_rotation(index));
    }
    var result = RobotKitSimKit.rk_simulation_add_robot(owner.borrow(), blueprint.nativeValue(), robotDesc);
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

  /**
   * Moves one robot's kinematic base to a pose that takes effect on the next
   * tick. Unlike teleportRobot, this keeps a realtime clock running, keeps
   * sensor history, and does not change the pose restored by reset or
   * resetRobot. The base's velocity over the tick is the motion from its
   * previous pose, so the IMU measures consecutive drives as continuous
   * motion. Use it to drive a base every tick; use placeRobotBase for a jump.
   */
  public function driveRobotBase(robotIndex:Int, position:Array<Float>,
      ?rotation:Array<Float>):Void {
    ensureLive();
    if (position == null || position.length != 3)
      throw "Simulation.driveRobotBase requires a three-component position";
    var pose = makePose(position, rotation);
    check(RobotKitSimKit.rk_simulation_drive_robot_base(owner.borrow(), robotIndex, pose),
      "simulation.driveRobotBase");
  }

  /**
   * Jumps one robot's base to a pose for the next tick without stopping a
   * realtime clock, resetting sensors, or changing the reset pose. The base
   * keeps its body-frame velocity through the jump, so the IMU sees no spike,
   * and a differential-drive plant continues from the new pose.
   */
  public function placeRobotBase(robotIndex:Int, position:Array<Float>,
      ?rotation:Array<Float>):Void {
    ensureLive();
    if (position == null || position.length != 3)
      throw "Simulation.placeRobotBase requires a three-component position";
    var pose = makePose(position, rotation);
    check(RobotKitSimKit.rk_simulation_place_robot_base(owner.borrow(), robotIndex, pose),
      "simulation.placeRobotBase");
  }

  /**
   * Couples one robot's wheel velocity targets to its kinematic base. Each
   * tick the base rolls by the wheel targets the robot applied for that tick
   * (after runtime clamping, zero after any stop), before physics advances,
   * whatever submitted them. Joint indices are robot joint indices; lengths
   * are metres. The plant starts from the base's current pose.
   */
  public function setDifferentialDrive(robotIndex:Int, leftWheelJoint:Int,
      rightWheelJoint:Int, wheelRadius:Float, trackWidth:Float):Void {
    ensureLive();
    if (leftWheelJoint < 0 || rightWheelJoint < 0)
      throw "Simulation.setDifferentialDrive requires wheel joint indices";
    var desc = new rk_simulation_differential_drive_desc();
    desc.set_struct_size(rk_simulation_differential_drive_desc.size());
    desc.set_left_wheel_joint(leftWheelJoint);
    desc.set_right_wheel_joint(rightWheelJoint);
    desc.set_wheel_radius(wheelRadius);
    desc.set_track_width(trackWidth);
    check(RobotKitSimKit.rk_simulation_set_differential_drive(owner.borrow(), robotIndex, desc),
      "simulation.setDifferentialDrive");
  }

  /** Removes a robot's differential-drive coupling; the base stays in place. */
  public function clearDifferentialDrive(robotIndex:Int):Void {
    ensureLive();
    check(RobotKitSimKit.rk_simulation_clear_differential_drive(owner.borrow(), robotIndex),
      "simulation.clearDifferentialDrive");
  }

  /**
   * Reads a robot's differential-drive plant after the latest tick: its
   * planar pose (yaw unwrapped) and the wheel rates the robot applied.
   */
  public function differentialDriveState(robotIndex:Int):SimulationDifferentialDriveState {
    ensureLive();
    var state = new rk_simulation_differential_drive_state();
    state.set_struct_size(rk_simulation_differential_drive_state.size());
    var result = RobotKitSimKit.rk_simulation_get_differential_drive_state(owner.borrow(),
      robotIndex, state);
    check(result.status, "simulation.differentialDriveState");
    return {
      enabled: state.get_enabled() != 0,
      x: state.get_x(),
      y: state.get_y(),
      yaw: state.get_yaw(),
      height: state.get_height(),
      leftWheelRate: state.get_left_wheel_rate(),
      rightWheelRate: state.get_right_wheel_rate()
    };
  }

  /**
   * Couples one robot's three omni-wheel velocity targets to its kinematic
   * base, like setDifferentialDrive but decoding a full planar body twist, so
   * the base can strafe. `wheelAngles` are the wheels' mount angles about +Z
   * from the base's x axis; see rk_simulation_set_omni_drive.
   */
  public function setOmniDrive(robotIndex:Int, wheelJoints:Array<Int>,
      wheelAngles:Array<Float>, wheelRadius:Float, baseRadius:Float):Void {
    ensureLive();
    if (wheelJoints == null || wheelJoints.length != 3 || wheelAngles == null ||
        wheelAngles.length != 3)
      throw "Simulation.setOmniDrive requires three wheel joints and three wheel angles";
    for (joint in wheelJoints)
      if (joint < 0) throw "Simulation.setOmniDrive requires wheel joint indices";
    var desc = new rk_simulation_omni_drive_desc();
    desc.set_struct_size(rk_simulation_omni_drive_desc.size());
    for (index in 0...3) {
      desc.set_wheel_joints(index, wheelJoints[index]);
      desc.set_wheel_angles(index, wheelAngles[index]);
    }
    desc.set_wheel_radius(wheelRadius);
    desc.set_base_radius(baseRadius);
    check(RobotKitSimKit.rk_simulation_set_omni_drive(owner.borrow(), robotIndex, desc),
      "simulation.setOmniDrive");
  }

  /** Removes a robot's omni-wheel coupling; the base stays in place. */
  public function clearOmniDrive(robotIndex:Int):Void {
    ensureLive();
    check(RobotKitSimKit.rk_simulation_clear_omni_drive(owner.borrow(), robotIndex),
      "simulation.clearOmniDrive");
  }

  /**
   * Reads a robot's omni-wheel plant after the latest tick: its planar pose
   * (yaw unwrapped) and the three wheel rates the robot applied.
   */
  public function omniDriveState(robotIndex:Int):SimulationOmniDriveState {
    ensureLive();
    var state = new rk_simulation_omni_drive_state();
    state.set_struct_size(rk_simulation_omni_drive_state.size());
    var result = RobotKitSimKit.rk_simulation_get_omni_drive_state(owner.borrow(), robotIndex,
      state);
    check(result.status, "simulation.omniDriveState");
    return {
      enabled: state.get_enabled() != 0,
      x: state.get_x(),
      y: state.get_y(),
      yaw: state.get_yaw(),
      height: state.get_height(),
      wheelRates: [for (index in 0...3) state.get_wheel_rates(index)]
    };
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

/** Omni-wheel plant state; see Simulation.omniDriveState. */
typedef SimulationOmniDriveState = {
  enabled:Bool,
  x:Float,
  y:Float,
  yaw:Float,
  height:Float,
  wheelRates:Array<Float>
};

/** Differential-drive plant state; see Simulation.differentialDriveState. */
typedef SimulationDifferentialDriveState = {
  enabled:Bool,
  x:Float,
  y:Float,
  yaw:Float,
  height:Float,
  leftWheelRate:Float,
  rightWheelRate:Float
};
