package robotkit.runtime;

import NativeKitSim;
import RobotKitSimKit;
import haxe.Int64;
import nativekit.sim.SimFrame;
import nativekit.sim.SimSession;
import robotkit.mobile.Pose2;
import robotkit.model.CollisionShape.CollisionPrimitive;
import robotkit.model.CollisionShape.ShapeContact;
import robotkit.tool.ToolCollisionShape;
import robotkit.tool.ToolCollisionShapes;
import robotkit.spatial.Vec3;

private typedef StepObserverEntry = {id:Int, observer:SimulationStepObserver};

/**
 * The robots taking part in one SimKit session.
 *
 * The session's owner (Simulation.inSession()'s caller) advances it, alongside
 * its props and people, and controls its clock (step, start, stop, reset) and
 * its environment (session objects and actors) directly.
 *
 * The object creates RobotRuntime handles but remains their simulation owner:
 * callers should submit through those handles and advance the session once per
 * tick. It is intentionally separate from SimulatedRobot, which is only a
 * live Robot adapter for RobotWorld.
 */
class Simulation {
  final owner:Ownedrk_simulation;
  final robots:Array<RobotRuntime> = [];
  final stepObservers:Array<StepObserverEntry> = [];
  var nextStepObserverId = 1;
  public final fixedTimestepSeconds:Float;
  /** The session this simulation joined. */
  final session:SimSession;
  /** Runs this simulation's step observers after each session tick. */
  final sessionObserverId:Int;
  final fixedTimestepNs:Int64;
  var sourceTimeNs:Int64 = Int64.ofInt(0);
  var disposed:Bool = false;

  /** Attaches robots to a stopped session the caller owns, steps, and outlives. */
  public function new(session:SimSession) {
    this.session = session;
    fixedTimestepSeconds = session.fixedTimestep();
    fixedTimestepNs = Int64.fromFloat(Math.max(1, Math.round(fixedTimestepSeconds * 1e9)));
    var attached = RobotKitSimKit.rk_simulation_create_in_session(session.nativeHandle());
    check(attached.status, "simulation.createInSession");
    owner = attached.out_simulation;
    sessionObserverId = session.addStepObserver(afterStep);
  }

  /** Attaches robots to a stopped session the caller owns, steps, and outlives. */
  public static function inSession(session:SimSession):Simulation
    return new Simulation(session);

  /** Robot poses from a frame captured from the session. */
  public function presentFrame(frame:SimFrame):SimulationPresentationSnapshot {
    ensureLive();
    var result = RobotKitSimKit.rk_simulation_present_frame(owner.borrow(), frame.nativeHandle());
    check(result.status, "simulation.presentFrame");
    try {
      return new SimulationPresentationSnapshot(result.out_presentation);
    } catch (error:Dynamic) {
      result.out_presentation.close();
      throw error;
    }
  }

  /** Adds topology before the first start or step. */
  public function addRobot(blueprint:RobotRuntimeBlueprint, ?initialPose:Pose2,
      ?virtualDevice:VirtualDeviceOptions, ?tool:ToolCollisionShape,
      ?toolLink:Int, ?toolMargin:Float, ?toolGap:Float):RobotRuntime {
    return addRobotWithPose(blueprint, initialPose == null ? null :
      makePose([initialPose.x, initialPose.y, 0.0],
        [0.0, 0.0, Math.sin(initialPose.yaw * 0.5), Math.cos(initialPose.yaw * 0.5)]),
      virtualDevice, null, null, null, tool, toolLink, toolMargin, toolGap);
  }

  /** Adds a robot with a resettable 3D pose. Tool padding defaults to a proximity
   * gap; pass toolMargin to make it physical.
   */
  public function addRobotAtPose(blueprint:RobotRuntimeBlueprint, position:Array<Float>,
      rotation:Array<Float>, ?virtualDevice:VirtualDeviceOptions,
      ?linkCollisionBoxes:Array<Null<Array<Float>>>,
      ?linkCollisionHulls:Array<Null<Array<Float>>>,
      ?closures:Array<SimulationClosure>, ?tool:ToolCollisionShape,
      ?toolLink:Int, ?toolMargin:Float, ?toolGap:Float, ?linkHulls:Array<SimulationLinkHull>,
      holdAtRest:Bool = false):RobotRuntime {
    if (position == null || position.length != 3 || rotation == null || rotation.length != 4)
      throw "Simulation.addRobotAtPose requires a three-component position and four-component rotation";
    return addRobotWithPose(blueprint, makePose(position, rotation), virtualDevice, linkCollisionBoxes,
      linkCollisionHulls, closures, tool, toolLink, toolMargin, toolGap, linkHulls, holdAtRest);
  }

  function addRobotWithPose(blueprint:RobotRuntimeBlueprint,
      initialPose:Null<rk_simulation_pose>,
      ?virtualDevice:VirtualDeviceOptions,
      ?linkCollisionBoxes:Array<Null<Array<Float>>>,
      ?linkCollisionHulls:Array<Null<Array<Float>>>,
      ?closures:Array<SimulationClosure>, ?tool:ToolCollisionShape,
      ?toolLink:Int, ?toolMargin:Float, ?toolGap:Float, ?linkHulls:Array<SimulationLinkHull>,
      holdAtRest:Bool = false):RobotRuntime {
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
        robotDesc.set_virtual_device_profile(virtualDevice.profile);
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
    if (linkHulls != null && linkHulls.length > 0) {
      if (linkHulls.length > RobotKitSimKitConstants.RK_MAX_LINK_HULLS)
        throw 'Simulation supports at most ${RobotKitSimKitConstants.RK_MAX_LINK_HULLS} link hulls';
      if (robotDesc == null) {
        robotDesc = new rk_simulation_robot_desc();
        robotDesc.set_struct_size(rk_simulation_robot_desc.size());
      }
      robotDesc.set_link_hull_count(linkHulls.length);
      for (index in 0...linkHulls.length) {
        var source = linkHulls[index];
        var vertices = source.vertices;
        if (source.link < 0 || source.link >= blueprint.linkCount)
          throw "Simulation link hull names an unknown link";
        if (vertices.length % 3 != 0 || vertices.length < 12 || vertices.length > 64 * 3)
          throw "Simulation link hull needs 4..64 vertices";
        var native = new rk_simulation_link_hull();
        native.set_link(source.link);
        native.set_vertex_count(Std.int(vertices.length / 3));
        for (coordinate in 0...vertices.length) {
          if (!Math.isFinite(vertices[coordinate])) throw "Simulation link hull has a non-finite vertex";
          native.set_vertices(coordinate, vertices[coordinate]);
        }
        robotDesc.set_link_hulls(index, native);
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
    if (tool != null) {
      var hulls:Array<Array<Float>> = switch tool {
        case NoCollision: [];
        case Hulls(pieces, _): pieces;
        case Box(half, centre):
          var c = centre == null ? Vec3.zero() : centre;
          [[for (index in 0...8) for (axis in 0...3)
            (axis == 0 ? c.x : axis == 1 ? c.y : c.z) +
            ((index & (1 << axis)) == 0 ? -1 : 1) *
            (axis == 0 ? half.x : axis == 1 ? half.y : half.z)]];
        case Cylinder(_, _): throw "Simulation tool collision supports Hulls or Box";
      };
      if (hulls.length > 0) {
        ToolCollisionShapes.bounds(tool);
        if (hulls.length > 16) throw "Simulation tool supports at most 16 convex pieces";
        var link = toolLink == null ? blueprint.linkCount - 1 : toolLink;
        if (link < 0 || link >= blueprint.linkCount)
          throw "Simulation tool link is outside the robot";
        var padding = switch tool { case Hulls(_, value): value; case _: 0.0; };
        var margin = toolMargin == null ? 0.0 : toolMargin;
        var gap = toolGap == null ? padding : toolGap;
        if (!Math.isFinite(margin) || !Math.isFinite(gap) || margin < 0 || gap < 0)
          throw "Simulation tool margin and gap are invalid";
        if (robotDesc == null) {
          robotDesc = new rk_simulation_robot_desc();
          robotDesc.set_struct_size(rk_simulation_robot_desc.size());
        }
        robotDesc.set_tool_link_index(link);
        robotDesc.set_tool_piece_count(hulls.length);
        robotDesc.set_tool_margin(margin);
        robotDesc.set_tool_gap(gap);
        for (piece in 0...hulls.length) {
          var vertices = hulls[piece];
          if (vertices == null || vertices.length < 12 || vertices.length > 192 ||
              vertices.length % 3 != 0)
            throw "Simulation tool hull needs 4..64 vertices";
          robotDesc.set_tool_piece_vertex_count(piece, Std.int(vertices.length / 3));
          for (index in 0...vertices.length) {
            if (!Math.isFinite(vertices[index])) throw "Simulation tool hull vertex is not finite";
            robotDesc.set_tool_piece_vertices(piece * 64 * 3 + index, vertices[index]);
          }
        }
      }
    }
    var shapes = blueprint.linkCollisionShapes;
    if (shapes.length > 0) {
      if (shapes.length > RobotKitSimKitConstants.RK_MAX_LINK_SHAPES)
        throw 'Simulation supports at most ${RobotKitSimKitConstants.RK_MAX_LINK_SHAPES} link collision shapes';
      if (robotDesc == null) {
        robotDesc = new rk_simulation_robot_desc();
        robotDesc.set_struct_size(rk_simulation_robot_desc.size());
      }
      robotDesc.set_link_shape_count(shapes.length);
      for (index in 0...shapes.length) {
        var source = shapes[index];
        var native = new rk_simulation_link_shape();
        native.set_link(source.link);
        var size:Array<Float> = switch source.shape.primitive {
          case CollisionPrimitive.Box(x, y, z):
            native.set_type(RobotKitSimKitConstants.RK_LINK_SHAPE_BOX);
            [x, y, z];
          case CollisionPrimitive.Sphere(radius):
            native.set_type(RobotKitSimKitConstants.RK_LINK_SHAPE_SPHERE);
            [radius];
          case CollisionPrimitive.Capsule(radius, half):
            native.set_type(RobotKitSimKitConstants.RK_LINK_SHAPE_CAPSULE);
            [radius, half];
          case CollisionPrimitive.Cylinder(radius, half):
            native.set_type(RobotKitSimKitConstants.RK_LINK_SHAPE_CYLINDER);
            [radius, half];
        };
        for (axis in 0...size.length) native.set_size(axis, size[axis]);
        for (axis in 0...3) native.set_position(axis, source.shape.position[axis]);
        for (axis in 0...4) native.set_rotation(axis, source.shape.rotation[axis]);
        var surface = source.shape.surface;
        if (surface != null) {
          for (axis in 0...3) native.set_friction(axis, surface.friction[axis]);
          native.set_friction_dimensions(surface.frictionDimensions);
          native.set_contact_time_constant(surface.contactTimeConstant);
          native.set_contact_damping_ratio(surface.contactDampingRatio);
        }
        native.set_contact_filter(switch source.shape.contact {
          case Layers: 0;
          case PairsOnly: 1;
          case PairsAndEnvironment: 2;
        });
        robotDesc.set_link_shapes(index, native);
      }
      var pairs = blueprint.contactPairs;
      if (pairs.length > RobotKitSimKitConstants.RK_MAX_CONTACT_PAIRS)
        throw 'Simulation supports at most ${RobotKitSimKitConstants.RK_MAX_CONTACT_PAIRS} contact pairs';
      robotDesc.set_contact_pair_count(pairs.length);
      for (index in 0...pairs.length) {
        var pair = pairs[index];
        var native = new rk_simulation_contact_pair();
        native.set_shape_a(pair.shapeA);
        native.set_shape_b(pair.shapeB);
        for (axis in 0...3) native.set_friction(axis, pair.surface.friction[axis]);
        native.set_friction_dimensions(pair.surface.frictionDimensions);
        native.set_contact_time_constant(pair.surface.contactTimeConstant);
        native.set_contact_damping_ratio(pair.surface.contactDampingRatio);
        robotDesc.set_contact_pairs(index, native);
      }
    }
    // Joints hold the designed pose until commanded, like servos enabled at power-on.
    if (holdAtRest) {
      if (robotDesc == null) {
        robotDesc = new rk_simulation_robot_desc();
        robotDesc.set_struct_size(rk_simulation_robot_desc.size());
      }
      var desc:rk_simulation_robot_desc = cast robotDesc;
      desc.set_flags(desc.get_flags() | RobotKitSimKitConstants.RK_SIMULATION_ROBOT_HOLD_AT_REST);
    }
    var result = RobotKitSimKit.rk_simulation_add_robot(owner.borrow(), blueprint.nativeValue(), robotDesc);
    check(result.status, "simulation.addRobot");
    var runtime = new RobotRuntime(result.out_runtime, blueprint);
    runtime.simulation = this;
    robots.push(runtime);
    return runtime;
  }

  /** Contacts for a runtime attached to this simulation. */
  public function ownsRobot(runtime:RobotRuntime):Bool {
    ensureLive();
    return runtime != null && robots.indexOf(runtime) >= 0;
  }

  public function robotContacts(runtime:RobotRuntime):Array<RobotContact> {
    ensureLive();
    if (runtime == null || runtime.simulation != this)
      throw "Runtime does not belong to this simulation";
    var capture = RobotKitSimKit.rk_simulation_capture_robot_contacts(
      owner.borrow(), runtime.owner.borrow());
    check(capture.status, "simulation.robotContacts");
    var contacts:Array<RobotContact> = [];
    var list = capture.out_list;
    try {
      var countResult = RobotKitSimKit.rk_robot_contact_list_count(list.borrow());
      check(countResult.status, "simulation.robotContacts.count");
      var stepResult = RobotKitSimKit.rk_robot_contact_list_step_index(list.borrow());
      check(stepResult.status, "simulation.robotContacts.stepIndex");
      for (index in 0...countResult.out_count) {
        var value = new rk_robot_contact();
        value.set_struct_size(rk_robot_contact.size());
        var result = RobotKitSimKit.rk_robot_contact_list_get(list.borrow(), index, value);
        check(result.status, "simulation.robotContact");
        value = result.out_contact;
        contacts.push({linkIndex: value.get_link_index(),
          toolPieceIndex: value.get_tool_piece_index(),
          otherObject: value.get_other_object().rawValue(),
          distance: value.get_distance(),
          position: new Vec3(value.get_position(0), value.get_position(1), value.get_position(2)),
          normal: new Vec3(value.get_normal(0), value.get_normal(1), value.get_normal(2)),
          active: value.get_active() != 0,
          stepIndex: stepResult.out_step_index,
          otherKind: cast value.get_other_kind(),
          otherRobot: value.get_other_robot(),
          otherLink: value.get_other_link()});
      }
    } catch (error:Dynamic) {
      list.close();
      throw error;
    }
    list.close();
    return contacts;
  }

  /** Logical source time for explicitly stepped sensors. It stays monotonic
   * across physics resets so published frames remain ordered. */
  public function sourceTimestampNs():Int64 return sourceTimeNs;

  public function addStepObserver(observer:SimulationStepObserver):Int {
    ensureLive();
    if (observer == null) throw "Simulation step observer is required";
    var id = nextStepObserverId++;
    stepObservers.push({id: id, observer: observer});
    return id;
  }

  public function removeStepObserver(id:Int):Void {
    for (index in 0...stepObservers.length) if (stepObservers[index].id == id) {
      stepObservers.splice(index, 1);
      return;
    }
  }

  /**
   * Runs step observers after each session tick, keeping the monotonically
   * increasing simulation source time explicitly stepped sensors read.
   */
  function afterStep():Void {
    sourceTimeNs = Int64.add(sourceTimeNs, fixedTimestepNs);
    for (entry in stepObservers.copy()) {
      var registered = false;
      for (current in stepObservers) if (current.id == entry.id) registered = true;
      if (registered) entry.observer.afterSimulationStep(sourceTimeNs);
    }
  }

  /** Injects a virtual RKD6 link loss or reconnects the link. */
  public function cutVirtualDeviceLink(robotIndex:Int, cut:Bool):Void {
    ensureLive();
    if (robotIndex < 0 || robotIndex >= robots.length)
      throw "Simulation virtual-device robot index is out of range";
    check(RobotKitSimKit.rk_simulation_cut_virtual_device_link(owner.borrow(),
      robotIndex, cut ? 1 : 0), "simulation.cutVirtualDeviceLink");
  }

  /** Restores one robot's initial body pose and runtime state. The session
   * must be stopped. */
  public function resetRobot(robotIndex:Int):Void {
    ensureLive();
    check(RobotKitSimKit.rk_simulation_reset_robot(owner.borrow(), robotIndex),
      "simulation.resetRobot");
  }

  /** Teleports one robot base while leaving the shared clock untouched. The
   * session must be stopped. */
  public function teleportRobot(robotIndex:Int, position:Array<Float>,
      ?rotation:Array<Float>):Void {
    ensureLive();
    if (position == null || position.length != 3)
      throw "Simulation.teleportRobot requires a three-component position";
    var pose = makePose(position, rotation);
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
   * are metres. A wheel's direction is -1 when a positive joint rate rolls it
   * backward. The plant starts from the base's current pose.
   */
  public function setDifferentialDrive(robotIndex:Int, leftWheelJoint:Int,
      rightWheelJoint:Int, wheelRadius:Float, trackWidth:Float,
      leftDirection:Int = 1, rightDirection:Int = 1):Void {
    ensureLive();
    if (leftWheelJoint < 0 || rightWheelJoint < 0)
      throw "Simulation.setDifferentialDrive requires wheel joint indices";
    var desc = new rk_simulation_differential_drive_desc();
    desc.set_struct_size(rk_simulation_differential_drive_desc.size());
    desc.set_left_wheel_joint(leftWheelJoint);
    desc.set_right_wheel_joint(rightWheelJoint);
    desc.set_wheel_radius(wheelRadius);
    desc.set_track_width(trackWidth);
    if (Math.abs(leftDirection) != 1 || Math.abs(rightDirection) != 1)
      throw "Simulation.setDifferentialDrive wheel directions must be 1 or -1";
    desc.set_reversed_wheels((leftDirection < 0 ? RobotKitSimKitConstants.RK_DRIVE_REVERSED_LEFT : 0)
      | (rightDirection < 0 ? RobotKitSimKitConstants.RK_DRIVE_REVERSED_RIGHT : 0));
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

  /**
   * Places a robot's joints, in joint order, as a pose to start from (such as
   * a standing keyframe) while the simulation is stopped. Velocities become
   * zero; reset returns joints to zero.
   */
  public function setJointPositions(robotIndex:Int, positions:Array<Float>):Void {
    ensureLive();
    check(RobotKitSimKit.rk_simulation_set_joint_positions(owner.borrow(), robotIndex, positions),
      "simulation.setJointPositions");
  }

  /**
   * Pushes a robot's base for the next tick: a world-frame force (N) and
   * torque (N m) at its centre of mass. Repeat every tick to push for longer.
   */
  public function applyRobotForce(robotIndex:Int, force:Array<Float>, ?torque:Array<Float>):Void {
    ensureLive();
    var moment = torque == null ? [0.0, 0.0, 0.0] : torque;
    if (force == null || force.length != 3 || moment.length != 3)
      throw "Simulation.applyRobotForce requires three-component force and torque";
    var wrench = new rk_simulation_wrench();
    wrench.set_struct_size(rk_simulation_wrench.size());
    for (axis in 0...3) {
      wrench.set_force(axis, force[axis]);
      wrench.set_torque(axis, moment[axis]);
    }
    check(RobotKitSimKit.rk_simulation_apply_robot_force(owner.borrow(), robotIndex, wrench),
      "simulation.applyRobotForce");
  }

  /** Reads one robot base pose without mutating physics or the editable model. */
  public function robotPose(robotIndex:Int):{position:Array<Float>,rotation:Array<Float>} {
    ensureLive();var pose=new rk_simulation_pose();pose.set_struct_size(rk_simulation_pose.size());
    var result=RobotKitSimKit.rk_simulation_get_robot_pose(owner.borrow(),robotIndex,pose);
    check(result.status,"simulation.getRobotPose");
    return {position:[for(index in 0...3)pose.get_position(index)],
      rotation:[for(index in 0...4)pose.get_rotation(index)]};
  }

  /**
   * Reads one robot base's world-frame twist: linear in metres per second,
   * angular in radians per second. A floating base reports its free motion.
   */
  public function robotBaseVelocity(robotIndex:Int):{linear:Array<Float>, angular:Array<Float>} {
    ensureLive();
    var twist = new rk_simulation_twist();
    twist.set_struct_size(rk_simulation_twist.size());
    var result = RobotKitSimKit.rk_simulation_get_robot_base_velocity(owner.borrow(), robotIndex, twist);
    check(result.status, "simulation.getRobotBaseVelocity");
    return {linear: [for (index in 0...3) twist.get_linear(index)],
      angular: [for (index in 0...3) twist.get_angular(index)]};
  }

  /**
   * The physics body that carries a robot link, to use as the carrier when a session object is held by
   * that link (`SimSession.holdObject`), for instance a workpiece gripped by a suction cup.
   */
  public function linkBody(robotIndex:Int, linkIndex:Int):nksim_body {
    ensureLive();
    var result = RobotKitSimKit.rk_simulation_get_link_body(owner.borrow(), robotIndex, linkIndex);
    check(result.status, "simulation.linkBody");
    return result.out_body;
  }

  /** Where a session object is now. */
  public function objectPose(object:nativekit.sim.SimObject):nativekit.sim.SimPose {
    ensureLive();
    var frame = session.capture();
    try {
      var pose = frame.objectPose(object);
      frame.dispose();
      return pose;
    } catch (failure:Dynamic) {
      frame.dispose();
      throw failure;
    }
  }

  /** Carries a session object on a robot link at `relative`, its pose in the link's frame, until released. */
  public function holdObjectOnLink(object:nativekit.sim.SimObject, robotIndex:Int, linkIndex:Int,
      relative:nativekit.sim.SimPose):Void {
    ensureLive();
    session.holdObject(object, linkBody(robotIndex, linkIndex), relative);
  }

  /** Lets go of a session object some link carries. */
  public function releaseObject(object:nativekit.sim.SimObject):Void {
    ensureLive();
    session.releaseObject(object);
  }

  /** Reads an articulated link pose without mutating the simulation. */
  public function linkPose(robotIndex:Int, linkIndex:Int):{position:Array<Float>,rotation:Array<Float>} {
    ensureLive();
    var pose = new rk_simulation_pose(); pose.set_struct_size(rk_simulation_pose.size());
    var result = RobotKitSimKit.rk_simulation_get_link_pose(owner.borrow(), robotIndex, linkIndex, pose);
    check(result.status, "simulation.getLinkPose");
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
    session.removeStepObserver(sessionObserverId);
    stepObservers.resize(0);
    for (runtime in robots) runtime.dispose();
    robots.resize(0);
    owner.close();
    disposed = true;
  }

  /**
   * RobotKit's status for the robot fault that failed the latest tick, such
   * as RK_ERROR_LIMIT, or RK_OK. The session that stepped reports only that a
   * participant failed.
   */
  public function rejection():Int {
    ensureLive();
    var value = RobotKitSimKit.rk_simulation_get_rejection(owner.borrow());
    check(value.status, "simulation.rejection");
    return value.out_result;
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
/** One convex hull of a link built from several rigid parts, as XYZ vertices in the link frame. */
typedef SimulationLinkHull = {
  var link:Int;
  var vertices:Array<Float>;
}

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

/**
 * Solver choices for a session's world (SimWorldOptions.integrator and
 * frictionCone, as SimulationSpace and SimulationHarness take them), matching
 * SimKit's integrator and friction-cone values.
 */
class SimulationSolver {
  public static inline final DEFAULT = 0;
  public static inline final EULER = 1;
  public static inline final IMPLICIT_FAST = 2;
  public static inline final RK4 = 3;
  public static inline final PYRAMIDAL_CONE = 1;
  public static inline final ELLIPTIC_CONE = 2;
}
