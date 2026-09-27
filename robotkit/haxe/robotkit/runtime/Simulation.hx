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
  public function addRobot(blueprint:RobotRuntimeBlueprint, ?initialPose:Pose2,
      ?virtualDevice:VirtualDeviceOptions):RobotRuntime {
    return addRobotWithPose(blueprint, initialPose == null ? null :
      makePose([initialPose.x, initialPose.y, 0.0],
        [0.0, 0.0, Math.sin(initialPose.yaw * 0.5), Math.cos(initialPose.yaw * 0.5)]),
      virtualDevice);
  }

  /** Adds a robot with a full 3D pose that reset restores. */
  public function addRobotAtPose(blueprint:RobotRuntimeBlueprint, position:Array<Float>,
      rotation:Array<Float>, ?virtualDevice:VirtualDeviceOptions,
      ?linkCollisionBoxes:Array<Null<Array<Float>>>,
      ?linkCollisionHulls:Array<Null<Array<Float>>>,
      ?closures:Array<SimulationClosure>):RobotRuntime {
    if (position == null || position.length != 3 || rotation == null || rotation.length != 4)
      throw "Simulation.addRobotAtPose requires a three-component position and four-component rotation";
    return addRobotWithPose(blueprint, makePose(position, rotation), virtualDevice, linkCollisionBoxes,
      linkCollisionHulls, closures);
  }

  function addRobotWithPose(blueprint:RobotRuntimeBlueprint,
      initialPose:Null<rk_simulation_pose>,
      ?virtualDevice:VirtualDeviceOptions,
      ?linkCollisionBoxes:Array<Null<Array<Float>>>,
      ?linkCollisionHulls:Array<Null<Array<Float>>>,
      ?closures:Array<SimulationClosure>):RobotRuntime {
    ensureLive();
    var robotDesc:Null<rk_simulation_robot_desc> = null;
    if (initialPose != null || virtualDevice != null) {
      robotDesc = new rk_simulation_robot_desc();
      robotDesc.set_struct_size(rk_simulation_robot_desc.size());
      robotDesc.set_initial_pose(initialPose == null ?
        makePose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) : initialPose);
      if (virtualDevice != null) {
        if (blueprint.jointCount > 64 || virtualDevice.actuators.length > 64 ||
            virtualDevice.fingerprint.length != 32 ||
            !~/^[0-9a-fA-F]{32}$/.match(virtualDevice.fingerprint) ||
            (virtualDevice.stepsPerUnit.length != 0 &&
             virtualDevice.stepsPerUnit.length != blueprint.jointCount))
          throw "Simulation virtual device configuration is invalid";
        robotDesc.set_virtual_device_enabled(1);
        robotDesc.set_virtual_device_tick_hz(virtualDevice.tickHz);
        robotDesc.set_virtual_device_step_tick_hz(virtualDevice.stepTickHz);
        robotDesc.set_virtual_device_offset_ticks(virtualDevice.offsetTicks);
        robotDesc.set_virtual_device_drift_ppm(virtualDevice.driftPpm);
        robotDesc.set_virtual_device_baud(virtualDevice.baud);
        robotDesc.set_virtual_device_latency_ns(virtualDevice.latencyNs);
        robotDesc.set_virtual_device_jitter_ns(virtualDevice.jitterNs);
        robotDesc.set_virtual_device_drop_rate(virtualDevice.frameDropRate);
        robotDesc.set_virtual_device_corruption_rate(virtualDevice.corruptionRate);
        robotDesc.set_virtual_device_seed(virtualDevice.seed);
        for (i in 0...blueprint.jointCount)
          robotDesc.set_virtual_device_steps_per_unit(i,
            virtualDevice.stepsPerUnit.length == 0 ? 1000.0 : virtualDevice.stepsPerUnit[i]);
        for (i in 0...16)
          robotDesc.set_virtual_device_fingerprint(i,
            Std.parseInt("0x" + virtualDevice.fingerprint.substr(i * 2, 2)));
        robotDesc.set_virtual_device_target_error(virtualDevice.targetError);
        robotDesc.set_virtual_device_clock_bound_ns(virtualDevice.clockBoundNs);
        robotDesc.set_virtual_device_link_loss_timeout_ns(virtualDevice.linkLossTimeoutNs);
        robotDesc.set_virtual_device_actuator_count(virtualDevice.actuators.length);
        for (i in 0...virtualDevice.actuators.length) {
          var actuator = virtualDevice.actuators[i];
          if (actuator.jointIndex >= blueprint.jointCount)
            throw "Virtual actuator references an unknown joint";
          robotDesc.set_virtual_device_actuator_joint(i, actuator.jointIndex);
          robotDesc.set_virtual_device_actuator_ratio(i, actuator.ratio);
          robotDesc.set_virtual_device_actuator_offset(i, actuator.offset);
          robotDesc.set_virtual_device_actuator_steps_per_unit(i, actuator.stepsPerUnit);
          robotDesc.set_virtual_device_actuator_max_rate(i, actuator.maxRate);
          robotDesc.set_virtual_device_actuator_direction_setup_ticks(i,
            actuator.directionSetupTicks);
          robotDesc.set_virtual_device_actuator_skew_bound(i, actuator.skewBound);
          for (byte in 0...actuator.id.length)
            robotDesc.set_virtual_device_actuator_ids(i * 64 + byte,
              actuator.id.charCodeAt(byte));
        }
      }
    }
    if (linkCollisionBoxes != null) {
      if (linkCollisionBoxes.length != blueprint.linkCount)
        throw "Simulation link collision boxes must match the link count";
      if (robotDesc == null) {
        robotDesc = new rk_simulation_robot_desc();
        robotDesc.set_struct_size(rk_simulation_robot_desc.size());
      }
      for (link in 0...linkCollisionBoxes.length) {
        var bounds = linkCollisionBoxes[link];
        if (bounds == null) continue;
        if (bounds.length != 3) throw "Simulation link collision box needs three extents";
        for (axis in 0...3) {
          if (!Math.isFinite(bounds[axis]) || bounds[axis] <= 0.0)
            throw "Simulation link collision extents must be finite and positive";
          robotDesc.set_collision_half_extents(link * 3 + axis, bounds[axis]);
        }
      }
    }
    if (linkCollisionHulls != null) {
      if (linkCollisionHulls.length != blueprint.linkCount)
        throw "Simulation link collision hulls must match the link count";
      if (robotDesc == null) {
        robotDesc = new rk_simulation_robot_desc();
        robotDesc.set_struct_size(rk_simulation_robot_desc.size());
      }
      for (link in 0...linkCollisionHulls.length) {
        var hull = linkCollisionHulls[link];
        if (hull == null) continue;
        if (hull.length % 3 != 0 || hull.length < 12 || hull.length > 64 * 3)
          throw "Simulation link collision hull needs 4..64 vertices";
        robotDesc.set_collision_hull_count(link, Std.int(hull.length / 3));
        for (axis in 0...hull.length) {
          if (!Math.isFinite(hull[axis])) throw "Simulation link collision hull has a non-finite vertex";
          robotDesc.set_collision_hull_vertices(link * 64 * 3 + axis, hull[axis]);
        }
      }
    }
    if (closures != null && closures.length > 0) {
      if (closures.length > 64) throw "Simulation supports at most 64 assembly closures";
      if (robotDesc == null) {
        robotDesc = new rk_simulation_robot_desc();
        robotDesc.set_struct_size(rk_simulation_robot_desc.size());
      }
      robotDesc.set_closure_count(closures.length);
      for (index in 0...closures.length) {
        var source = closures[index];
        if (source.parentLink < 0 || source.childLink < 0 ||
            source.parentLink >= blueprint.linkCount || source.childLink >= blueprint.linkCount ||
            source.anchorParent.length != 3 || source.axisParent.length != 3)
          throw "Simulation closure has invalid links or geometry";
        var native = new rk_simulation_closure_desc();
        native.set_parent_link(source.parentLink);
        native.set_child_link(source.childLink);
        native.set_type(source.type);
        for (axis in 0...3) {
          native.set_anchor_parent(axis, source.anchorParent[axis]);
          native.set_axis_parent(axis, source.axisParent[axis]);
        }
        robotDesc.set_closures(index, native);
      }
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
      ?dynamicBody:Bool = false, ?mass:Float = 1.0,
      ?orientation:Array<Float>):Int {
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
    var rotation = orientation == null ? [0.0, 0.0, 0.0, 1.0] : orientation;
    if (rotation.length != 4) throw "Simulation.spawnBox requires a quaternion";
    var length = 0.0;
    for (component in rotation) {
      if (!Math.isFinite(component)) throw "Simulation.spawnBox requires a finite quaternion";
      length += component * component;
    }
    if (Math.abs(length - 1.0) > 1e-4) throw "Simulation.spawnBox requires a normalized quaternion";
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
