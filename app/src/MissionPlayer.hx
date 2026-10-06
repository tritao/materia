package app;

import app.SimulatedTools.GripObject;
import cadkit.modeling.AssemblyState;
import haxe.Int64;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactMission;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
import materia.project.SceneArtifact.SceneArtifactPlace;
import materia.project.SceneArtifact.SceneArtifactRobotTool;
import materia.project.SceneArtifact.SceneArtifactTorchPose;
import materia.project.SceneArtifact.SceneArtifactMissionWaypoint;
import materia.project.SceneArtifact.SceneArtifactWeld;
import materia.project.SceneContactRegistration.SceneContactWork;
import motionkit.robot.HandlingPlanRunner;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.ManipulatorKinematics;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.StartTolerances;
import motionkit.robot.MotionProgramSkill;
import motionkit.event.EventValue;
import motionkit.trajectory.ValidationLimits;
import motionkit.program.MotionProgram;
import motionkit.program.MotionOp;
import motionkit.program.MoveTarget;
import motionkit.program.Blend;
import motionkit.kinematics.Pose3;
import motionkit.MotionOptions;
import motionkit.robot.MotionSystem;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.robot.StepperSlip;
import robotkit.runtime.EncoderMonitor;
import motionkit.robot.PlanningLimits;
import processkit.WeldingPlanRunner;
import processkit.WeldStowPlanner;
import processkit.skill.WeldPlan;
import processkit.skill.WeldSeam;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.manipulation.ArmClearance;
import motionkit.robot.ArmPlanClearance;
import robotkit.manipulation.ArmClearance.ClearanceViolation;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.Grasp;
import robotkit.manipulation.Grasp.GraspFrame;
import robotkit.manipulation.Grasp.GraspPlane;
import robotkit.model.Frame;
import robotkit.model.SteadyLoads;
import robotkit.skill.HandlePart;
import robotkit.spatial.Vec3;
import robotkit.localization.SimulationTruthLocalization;
import robotkit.localization.WheelOdometryLocalization;
import robotkit.mobile.Pose2;
import robotkit.navigation.AStarPlanner;
import robotkit.navigation.Costmap2;
import robotkit.navigation.MotionGuard;
import robotkit.navigation.MotionGuardState;
import robotkit.navigation.Navigation;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Navigator;
import robotkit.navigation.OccupancyCell;
import robotkit.navigation.OccupancyGrid2;
import robotkit.perception.FrameAwarePerception;
import robotkit.perception.LidarFreeSpace;
import robotkit.perception.LidarMapFilter;
import robotkit.perception.LidarObstaclePerception;
import robotkit.perception.Obstacle;
import robotkit.perception.PerceptionSnapshot;
import robotkit.runtime.Simulation;
import robotkit.skill.GoTo;
import robotkit.skill.Skill;
import robotkit.skill.SkillLifecycle;
import robotkit.skill.SkillRunner;
import robotkit.skill.SkillStatus;
import robotkit.skill.SkillResult;
import robotkit.core.RobotSnapshot;

/** A box held where it stands: centre, half extents and heading, in metres and radians. */
typedef FloorObstacle = robotkit.navigation.FloorMap.FloorBox;

/**
 * What a view of the running mission draws, all in the map frame and metres: the route being followed,
 * the costmap it plans on (with the obstacles the lidar adds to it), the pose the wheel odometry
 * believes, and what the safety guard is doing.
 */
typedef MissionOverlay = {
  var route:Array<Pose2>;
  var costmap:Null<Costmap2>;
  var obstacles:Array<Obstacle>;
  /** Where wheel odometry puts the robot, and the outline of its footprint, counter-clockwise. */
  var odometry:Null<Pose2>;
  var outline:Array<Pose2>;
  var guard:MotionGuardState;
}

/** Sets one already-validated robot-owned wire without writing controller internals. */
private class MissionSignalSkill implements Skill {
  final name:String;
  final value:Bool;
  final write:String->Bool->Void;
  final lifecycle = new SkillLifecycle();

  public function new(name:String, value:Bool, write:String->Bool->Void) {
    this.name = name; this.value = value; this.write = write;
  }

  public function start():Void {
    lifecycle.begin();
    write(name, value);
    lifecycle.succeed('set $name to $value');
  }

  public function update(_snapshot:RobotSnapshot, _durationSeconds:Float):SkillStatus return lifecycle.status();
  public function cancel():Void lifecycle.cancel();
  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
}

/** Waits on a typed cell input using simulation time, not polling count or wall time. */
private class MissionWaitSkill implements Skill {
  final name:String;
  final expected:Bool;
  final timeout:Float;
  final read:String->Null<EventValue>;
  final lifecycle = new SkillLifecycle();
  var elapsed = 0.0;

  public function new(name:String, expected:Bool, timeout:Float, read:String->Null<EventValue>) {
    this.name = name; this.expected = expected; this.timeout = timeout; this.read = read;
  }

  public function start():Void {
    elapsed = 0.0;
    lifecycle.begin();
  }

  public function update(_snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    switch read(name) {
      case Digital(value) if (value == expected):
        lifecycle.succeed('input $name became $expected');
      case Digital(_):
      case null:
      case _:
        lifecycle.fail('input $name is not digital');
    }
    if (!lifecycle.isRunning()) return lifecycle.status();
    elapsed += durationSeconds;
    if (elapsed >= timeout) lifecycle.fail('timed out waiting for $name to become $expected after $timeout s');
    return lifecycle.status();
  }

  public function cancel():Void lifecycle.cancel();
  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
}

/** Applies a simulation-time deadline to one asynchronous handling or motion skill. */
private class MissionTimedSkill implements Skill {
  final inner:Skill;
  final label:String;
  final timeout:Float;
  final lifecycle = new SkillLifecycle();
  var elapsed = 0.0;

  public function new(inner:Skill, label:String, timeout:Float) {
    this.inner = inner; this.label = label; this.timeout = timeout;
  }

  public function start():Void {
    elapsed = 0.0;
    lifecycle.begin();
    inner.start();
    accept(inner.status());
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    accept(inner.update(snapshot, durationSeconds));
    if (!lifecycle.isRunning()) return lifecycle.status();
    elapsed += durationSeconds;
    if (elapsed >= timeout) {
      inner.cancel();
      lifecycle.fail('$label timed out after $timeout s');
    }
    return lifecycle.status();
  }

  function accept(status:SkillStatus):Void switch status {
    case Succeeded: lifecycle.succeed('$label completed');
    case Failed(message): lifecycle.fail('$label: $message');
    case Cancelled: lifecycle.cancel();
    case Idle: lifecycle.fail('$label returned to idle');
    case Running:
  }

  public function cancel():Void {
    if (lifecycle.isRunning()) inner.cancel();
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
}

private typedef JointMotionContext = {
  var motion:ManipulatorMotion;
  var arm:Manipulator;
  var indices:Array<Int>;
}

private typedef HandlingToolKinematics = {var link:Int; var tip:AssemblyFrame;}

/**
 * The project's mission, run by its assembly robot when the simulation runs: each step becomes a
 * RobotKit skill, advanced one tick at a time by a `SkillRunner`, and the next starts when it
 * succeeds; the round starts over after its last step when it loops, and from its first step when
 * the session resets. `goTo` is RobotKit's `GoTo`: `Navigator` plans on a map drawn from the
 * robot's surroundings (the boxes it collides with as held objects, so nothing is drawn by hand)
 * and follows the route with pure pursuit, the robot's place coming from the simulation. `pick` and
 * `place` are RobotKit's `HandlePart`: the arm runs a MotionKit program down onto the selected grasp,
 * or onto its seat with the held part, read where they are when the step starts. Physical jaws hold
 * gripper-picked parts by contact; suction tools use the simulated cup. Each handling tool reports its grasp
 * sensor. `weld` is RobotKit's `WeldSeam` with the robot's
 * torch: the arm runs a process run's MotionKit program along the seam the step carries, the program switches the torch's
 * channels on the robot, the simulated welder strikes and holds the arc, and the skill reads the outcome on the torch's
 * weld sensor (`WeldBeads` lays the metal beside it). A mission that does not loop stops after its last step.
 *
 * A robot with a lidar sees what the map lacks: RobotKit's `LidarObstaclePerception` reads each scan with
 * the returns the map explains removed (`LidarMapFilter`), the obstacles it finds join the costmap's
 * dynamic layer, so `Navigator` replans round them, and `MotionGuard` slows the base for them and stops it
 * short, whatever the route says.
 */
class MissionPlayer implements SessionMember {
  /** Map cells, in metres: fine enough that a wall a tenth of a metre thick covers whole cells. */
  public static inline var RESOLUTION:Float = robotkit.navigation.FloorMap.DEFAULT_RESOLUTION;
  /** Room left round the obstacles' extent so the robot's own place is always on the map. */
  public static inline var MARGIN:Float = robotkit.navigation.FloorMap.DEFAULT_MARGIN;
  /** Boxes this far above the floor or higher pass over the robot. */
  public static inline var CLEARANCE:Float = robotkit.navigation.FloorMap.DEFAULT_CLEARANCE;
  public static inline var FRAME:String = "map";
  /** How close a `goTo` stands to its pose, in metres and radians. */
  public static inline var POSITION_TOLERANCE:Float = 0.05;
  public static inline var HEADING_TOLERANCE:Float = 0.05;
  /** Assumed acceleration only for arm joints whose drive model supplies no cap, rad/s². */
  public static inline var ARM_ACCELERATION:Float = 2.0;
  /** A lidar return this close (m) to a mapped cell belongs to the map: it is the room, not an obstacle. */
  public static inline var MAP_TOLERANCE:Float = 0.1;
  /** Radius (m) of the disk a lidar return stands for: a scan samples the face of what it sees. */
  public static inline var SCAN_RADIUS:Float = 0.08;
  /** The guard's reaction time (s), the clearance (m) it stops at, and the further distance (m) over which it slows. */
  public static inline var GUARD_REACTION:Float = 0.2;
  public static inline var GUARD_MARGIN:Float = 0.1;
  public static inline var GUARD_SLOWDOWN:Float = 0.5;
  /** An obstacle the lidar has lost sight of stays on the map this long (s), unless a scan sees through its place. */
  public static inline var MEMORY_SECONDS:Float = 20.0;

  public final mission:SceneArtifactMission;
  /** The navigation map, when the mission drives. */
  public final costmap:Null<Costmap2>;
  public final assemblyParts:SimulationAssemblyParts;
  /** The boxes the map was drawn from: those standing in the robot's way. */
  public final obstacles:Array<FloorObstacle>;
  /** The software guard that slows and stops the base for what the lidar sees, when the robot has a lidar. */
  public final guard:Null<MotionGuard>;
  /** What the lidar found beyond the map in its latest scan, in the map frame. */
  public var sensed(default, null):PerceptionSnapshot = new PerceptionSnapshot();
  /** The step running now. */
  public var stepIndex(default, null):Int = 0;
  /** Steps finished since the session started. */
  public var completed(default, null):Int = 0;
  /** Why the mission stopped, or null while it runs. */
  public var failure(default, null):Null<String> = null;
  /** Stable alarm code for the operator panel and recovery decision. */
  public var alarmCode(default, null):Null<MissionAlarmCode> = null;
  /** Every step is done and the mission does not loop: nothing more to run until a reset. */
  public var finished(default, null):Bool = false;

  final robot:AssemblyRobot;
  var homing:Null<MotionSystem> = null;
  var homingStarted:Bool = false;
  var homingComplete:Bool = false;
  public var homingSeconds(default, null):Float = 0.0;
  var homingEncoders:Null<EncoderMonitor> = null;
  var homingSlip:Null<StepperSlip> = null;
  final cellRobots:Array<AssemblyRobot>;
  var runner = new SkillRunner();
  var jointMotions = new Map<String, ManipulatorMotion>();
  /** Handling tool definitions, planning contexts and physical contacts, keyed by their logical channel. */
  final handlingTools:Map<String, SceneArtifactRobotTool> = new Map();
  final gripperTools:Map<String, SceneArtifactRobotTool> = new Map();
  final handlingRunners:Map<String, HandlingPlanRunner> = new Map();
  final newHandlingByChannel:Map<String, Void->HandlingPlanRunner> = new Map();
  final toolKinematics:Map<String, HandlingToolKinematics> = new Map();
  /** The arm and its torch, when the mission welds; and the torch's weld sensor. */
  var welding:Null<WeldingPlanRunner>;
  final newWelding:Null<Void->WeldingPlanRunner>;
  final weldSensor:Null<String>;
  final newRegistration:Null<SceneContactWork->processkit.ContactRegistrationRunner>;
  var registeredWork = new Map<String, Transform3>();
  /** What welds are planned clear of, and where the arm's joints are among the robot's. */
  var clearance:Null<ArmClearance> = null;
  var clearanceJoints:Array<Int> = [];
  /** Runtime weld metal in world coordinates, read before each pass is planned. */
  public var depositedWeldHulls:Void -> Array<{name:String, vertices:Array<Float>}> = () -> [];
  final robotIndex:Int;
  final simulation:Simulation;
  final weldAcceleration:Float;
  final objects:Array<GripObject>;
  final project:ProjectDocumentSession;
  final assembly:Null<AssemblyDefinition>;
  final placement:Null<AssemblyState>;
  final metres:Float;
  /** Each jaw keeps its own workpiece and measured pickup transform during overlap. */
  final heldParts:Map<String, SceneArtifactPlace> = new Map();
  final currentGrasps:Map<String, GraspFrame> = new Map();
  final heldToolOffsets:Map<String, Vec3> = new Map();
  final heldToolRotations:Map<String, Quat> = new Map();
  final heldToolYaws:Map<String, Float> = new Map();
  final heldApproaches:Map<String, Vec3> = new Map();
  final carryingTools:Map<String, Bool> = new Map();
  final navigator:Null<Navigator>;
  final localization:SimulationTruthLocalization;
  /** The lidar's perception, in the map frame, and the sensor it reads. */
  final scanning:Null<FrameAwarePerception>;
  final scanner:Null<String>;
  /** Sequence of the scan `sensed` was made from, to read each scan once. */
  var scanned:Null<Int64> = null;
  /** The robot's wheel odometry, run beside the truth for comparison; seeded from the truth when the mission starts. */
  final wheels:Null<WheelOdometryLocalization>;
  var wheelsSeeded:Bool = false;
  final timestep:Float;
  final cellInput:Null<String->Null<EventValue>>;
  final cellOutput:Null<String->Bool->Void>;
  var missionOutputs = new Map<String, Bool>();
  var failedStepIndex:Int = -1;
  var recoveryPending:Bool = false;
  var recoveryPulsePending:Bool = false;
  var recoveryOutputPending:Bool = false;
  var recoveryElapsed:Float = 0.0;
  var recoveryStep:Int = -1;
  final jointMotionPolicy:JointMotionPolicy;
  final idle = new PerceptionSnapshot();

  public function new(mission:SceneArtifactMission, robot:AssemblyRobot, simulation:Simulation, robotIndex:Int,
      timestep:Float, obstacles:Array<FloorObstacle>, objects:Array<GripObject>, project:ProjectDocumentSession, cellRobots:Array<AssemblyRobot>, ?cellInput:String->Null<EventValue>, ?cellOutput:String->Bool->Void, ?jointMotionPolicy:JointMotionPolicy, weldAcceleration:Float = ARM_ACCELERATION) {
    if (!Math.isFinite(weldAcceleration) || weldAcceleration <= 0.0 || weldAcceleration > ARM_ACCELERATION)
      throw "Invalid welding planning acceleration";
    if (mission.steps.length == 0) throw "A mission needs steps";
    this.mission = mission;
    this.robot = robot;
    this.cellRobots = cellRobots.copy();
    this.timestep = timestep;
    this.cellInput = cellInput;
    this.cellOutput = cellOutput;
    this.jointMotionPolicy = jointMotionPolicy == null ? JointMotionPolicy.mission() : jointMotionPolicy;
    this.obstacles = standing(obstacles);
    this.simulation = simulation;
    this.weldAcceleration = weldAcceleration;
    this.objects = objects;
    this.robotIndex = robotIndex;
    this.project = project;
    assemblyParts = new SimulationAssemblyParts(simulation, robot, objects, project);
    var physical = project.projectPhysical;
    metres = physical == null ? 1.0 : physical.metresPerUnit;
    var definition = project.projectAssemblyDefinition;
    assembly = definition == null ? null : AssemblyDefinitionFlattener.flatten(definition);
    placement = definition == null ? null : new AssemblyState(definition, project.projectAssemblyState);
    localization = new SimulationTruthLocalization(simulation, robotIndex, FRAME, robot.rootLink);
    var drives = [for (step in mission.steps) if (step.kind == "goTo") step];
    var base = robot.mobile;
    if (drives.length == 0) {
      costmap = null;
      navigator = null;
      guard = null;
      scanning = null;
      scanner = null;
      wheels = null;
    } else {
      if (base == null) throw "A mission that drives needs the project's wheeled assembly";
      var footprint = base.footprint;
      if (footprint == null) throw "A mission that drives needs the robot's footprint";
      var poses = [for (step in drives) floorPose(step)];
      var initial = simulation.linkPose(robotIndex, 0);
      poses.push(new Transform3(new Vec3(initial.position[0], initial.position[1], initial.position[2]),
        new Quat(initial.rotation[0], initial.rotation[1], initial.rotation[2], initial.rotation[3])).toPose2());
      var margin = robotkit.navigation.FloorMap.navigationMargin(footprint.radius, 0.3, RESOLUTION);
      var map = new Costmap2(floorPlan(obstacles, poses, margin), footprint.radius, true, 0.3, 1.5, MEMORY_SECONDS);
      costmap = map;
      var navigation = new Navigation(base, localization, 0.35, 0.5, 1.2, false);
      var lidars = [for (sensor in project.robotSensors) if (sensor.kind == "lidar") sensor];
      if (lidars.length > 1) throw 'A mission reads one lidar, the robot has ${lidars.length}';
      if (lidars.length == 0) {
        scanner = null;
        scanning = null;
        guard = null;
      } else {
        var sensor = robot.blueprint.sensorById(lidars[0].id);
        if (sensor == null) throw 'The robot has no sensor "${lidars[0].id}"';
        scanner = sensor.id;
        scanning = new FrameAwarePerception(LidarObstaclePerception.fromSensor(sensor, SCAN_RADIUS), localization, null,
          LidarMapFilter.fromSensor(map.grid, sensor, MAP_TOLERANCE), LidarFreeSpace.fromSensor(sensor));
        guard = new MotionGuard(navigation, footprint, GUARD_REACTION, base.motionLimits.maxLinearAcceleration, GUARD_MARGIN,
          GUARD_SLOWDOWN, map);
      }
      navigator = new Navigator(navigation, new AStarPlanner(map), map, 0.5, guard);
      wheels = new WheelOdometryLocalization(base, FRAME, robot.rootLink);
    }
    var handles = [for (step in mission.steps) if (step.kind == "pick" || step.kind == "place") step].length > 0;
    var welds = [for (step in mission.steps) if (step.kind == "weld" || step.kind == "findWork") step].length > 0;
    if (handles && welds) throw "A mission picks and places, or it welds: the robot works with one tool on its arm";
    var suctions = [for (tool in project.robotTools) if (tool.kind == "suction") tool];
    var grippers = [for (tool in project.robotTools) if (tool.kind == "gripper") tool];
    var torches = [for (tool in project.robotTools) if (tool.kind == "torch") tool];
    if (handles) {
      if (suctions.length > 0 && grippers.length > 0)
        throw "A handling mission cannot mix suction and physical gripper tools";
      var definitions = suctions.concat(grippers);
      if (definitions.length == 0) throw "A handling mission needs a suction tool or physical gripper";
      for (tool in definitions) {
        if (tool.channel == null || tool.channel.length == 0 || handlingTools.exists(tool.channel))
          throw 'Handling tool channel "${tool.channel}" is missing or duplicated';
        if (tool.sensor == null || tool.sensor.length == 0)
          throw 'Handling tool "${tool.channel}" has no grasp sensor';
        var selectedTool = tool;
        var isGripper = selectedTool.kind == "gripper";
        if (isGripper && selectedTool.gripper == null)
          throw 'Physical gripper "${selectedTool.channel}" has no jaw limits';
        var arm = toolArm(selectedTool);
        var carrier = robot.part("project:" + selectedTool.contact.occurrence);
        var tip = robot.toolFrames.get(selectedTool.contact.occurrence);
        if (tip == null) throw 'The robot has no frame for handling tool "${selectedTool.channel}"';
        toolKinematics.set(selectedTool.channel, {link: carrier.linkIndex, tip: {
          x: tip.position[0], y: tip.position[1], z: tip.position[2], qx: tip.rotation[0],
          qy: tip.rotation[1], qz: tip.rotation[2], qw: tip.rotation[3]}});
        var selectedArm = arm;
        var makeRunner:Void->HandlingPlanRunner = () -> {
          var state = robot.runtime.snapshot();
          return HandlingPlanRunner.create(robot.robot, selectedArm, () -> robot.runtime.pollEvents(),
            selectedTool.channel,
            PlanningLimits.ofGroup(selectedArm, new SteadyLoads(), ARM_ACCELERATION, null,
              state.modelRevision, state.calibrationRevision),
            HandlingPlanRunner.DEFAULT_APPROACH_HEIGHT, 0.3, 0.08, 0.4, isGripper ? 0.0 : 0.003);
        };
        handlingTools.set(selectedTool.channel, selectedTool);
        if (isGripper) gripperTools.set(selectedTool.channel, selectedTool);
        newHandlingByChannel.set(selectedTool.channel, makeRunner);
        handlingRunners.set(selectedTool.channel, makeRunner());
      }
    }
    if (!welds) {
      welding = null;
      newWelding = null;
      weldSensor = null;
      newRegistration = null;
    } else {
      if (torches.length != 1) throw 'A mission that welds needs one torch, the robot has ${torches.length}';
      var tool = torches[0];
      var torch = tool.torch;
      if (torch == null || tool.sensor == null) throw "The robot's torch needs its welder and weld sensor";
      var sensorId:String = tool.sensor;
      weldSensor = sensorId;
      var arm = toolArm(tool);
      var carrier = robot.part("project:" + tool.contact.occurrence);
      var tip = robot.toolFrames.get(tool.contact.occurrence);
      if (tip == null) throw 'The robot has no frame for torch "${tool.channel}"';
      toolKinematics.set(tool.channel, {link: carrier.linkIndex, tip: {
        x: tip.position[0], y: tip.position[1], z: tip.position[2], qx: tip.rotation[0],
        qy: tip.rotation[1], qz: tip.rotation[2], qw: tip.rotation[3]}});
      var weldKinematics = toolKinematics.get(tool.channel);
      if (weldKinematics == null) throw 'The robot has no kinematics for torch "${tool.channel}"';
      var channels = {arc: tool.channel, wireSpeed: torch.wireSpeedChannel, voltage: torch.voltageChannel};
      // Welds are planned clear of everything the simulation collides with except the weld metal, which is the bead.
      var metal = [for (step in mission.steps) if (step.kind == "weld") cast(step.weld, SceneArtifactWeld).metal];
      newWelding = () -> {
        var state = robot.runtime.snapshot();
        var planned = weldClearance(arm, tool.contact.occurrence, metal);
        clearance = planned;
        return WeldingPlanRunner.create(robot.robot, arm, () -> robot.runtime.pollEvents(), channels,
          PlanningLimits.ofGroup(arm, new SteadyLoads(), weldAcceleration, null, state.modelRevision, state.calibrationRevision), 3, planned);
      };
      welding = newWelding();
      newRegistration = (job) -> {
        var odometry = wheels;
        var observed = odometry == null ? null : odometry.state();
        if (observed == null) throw "Contact registration requires an initialized wheel pose estimate";
        var nominal = job.nominal;
        var rootWork = Transform3.fromPose2(observed.pose).inverse().compose(new Transform3(
          new Vec3(nominal.x, nominal.y, nominal.z), new Quat(nominal.qx, nominal.qy, nominal.qz, nominal.qw)));
        var bodies = weldCollisionBodies(arm, tool.contact.occurrence, metal);
        var planned = armClearance(arm, bodies);
        var tip = weldKinematics.tip;
        var wire = new processkit.ProbeWireClearance(arm, robot.model.links[weldKinematics.link].id,
          new Transform3(new Vec3(tip.x, tip.y, tip.z), new Quat(tip.qx, tip.qy, tip.qz, tip.qw)),
          torch.wireDiameterMm * 0.001, torch.stickoutMm * 0.001, bodies);
        clearance = planned;
        var planning = WeldingPlanRunner.planning(arm, weldAcceleration, planned);
        var indices = arm.jointIndices();
        var motion = new ManipulatorMotion(robot.robot, planning.compiler, (_) -> null, () -> robot.runtime.pollEvents(), indices);
        var probes = new processkit.ProbeMotionPlanner(arm, planning.compiler, planned, wire);
        var selection = new machinekit.welding.WeldContactSelection(job, probes, () -> {
          var snapshot = robot.robot.snapshot();
          return [for (index in indices) snapshot.positions.get(index)];
        });
        var probe = new processkit.ContactProbeRunner(motion, probes, channels, sensorId,
          () -> new motionkit.robot.ServoSession(robot.robot, arm));
        return new processkit.ContactRegistrationRunner(probe, selection.sequence(rootWork));
      };
    }
    configureHoming();
  }

  /**
   * The clearance a weld is planned with: the collision hulls the simulation uses, each on its link, the torch's marked as
   * the tool, and the arm's pose now as the one where neighbouring links touch by design. Welding keeps 3 mm between bodies in air
   * and 0.5 mm between tool and work at the seam; the arm's installed gearheads have 4.5 mm designed clearance to the next tube. `ignored` are occurrences that
   * are no obstacle (the weld metal).
   */
  function weldClearance(arm:Manipulator, torch:String, ignored:Array<String>):ArmClearance
    return cellClearance(arm, torch.substr(0, torch.lastIndexOf("/") + 1), ignored, processkit.WeldPathPlanner.AIR_MARGIN);

  function weldCollisionBodies(arm:Manipulator, torch:String, ignored:Array<String>):Array<robotkit.manipulation.ArmClearance.ClearanceBodyData>
    return cellCollisionBodies(arm, torch.substr(0, torch.lastIndexOf("/") + 1), ignored);

  /** Fixed cell geometry and other controllers' bodies at the start of planning. */
  function cellClearance(arm:Manipulator, toolPrefix:Null<String>, ignored:Array<String>, margin:Float,
      ?interfaces:Array<robotkit.manipulation.ArmClearance.ClearanceInterfaceData>):ArmClearance {
    return armClearanceAtMargin(arm, cellCollisionBodies(arm, toolPrefix, ignored), margin, interfaces);
  }

  function cellCollisionBodies(arm:Manipulator, toolPrefix:Null<String>, ignored:Array<String>):Array<robotkit.manipulation.ArmClearance.ClearanceBodyData> {
    var links = robot.model.links;
    var bodies = [for (hull in robot.hulls) if (ignored.indexOf(hull.part) < 0)
      {name: hull.part, link: links[hull.link].id, vertices: hull.vertices, tool: toolPrefix != null && StringTools.startsWith(hull.part, toolPrefix)}];
    var base = simulation.linkPose(robotIndex, 0);
    var baseInverse = new Transform3(new Vec3(base.position[0], base.position[1], base.position[2]),
      new Quat(base.rotation[0], base.rotation[1], base.rotation[2], base.rotation[3])).inverse();
    for (entry in objects) if (StringTools.startsWith(entry.id, "project:") && ignored.indexOf(entry.id.substr(8)) < 0) {
      var part = assemblyParts.get(entry.id);
      if (part.vertices.length == 0) continue;
      var live = part.pose();
      var carrier = simulation.objectCarrier(entry.object), attached = -1;
      for (index in 0...links.length)
        if (simulation.linkBody(robotIndex, index) == carrier) {attached = index; break;}
      var inverse = baseInverse;
      if (attached >= 0) {
        var body = simulation.linkPose(robotIndex, attached);
        inverse = new Transform3(new Vec3(body.position[0], body.position[1], body.position[2]),
          new Quat(body.rotation[0], body.rotation[1], body.rotation[2], body.rotation[3])).inverse();
      }
      var pose = inverse.compose(new Transform3(new Vec3(live.position[0], live.position[1], live.position[2]),
        new Quat(live.rotation[0], live.rotation[1], live.rotation[2], live.rotation[3])));
      var vertices:Array<Float> = [];
      for (index in 0...Std.int(part.vertices.length / 3)) {
        var point = pose.transformPoint(new Vec3(part.vertices[index * 3], part.vertices[index * 3 + 1], part.vertices[index * 3 + 2]));
        vertices.push(point.x); vertices.push(point.y); vertices.push(point.z);
      }
      bodies.push({name: entry.id.substr(8), link: links[attached >= 0 ? attached : 0].id,
        vertices: vertices, tool: attached >= 0});
    }
    var included = new Map<String, Bool>();
    for (body in bodies) included.set(body.name, true);
    for (other in cellRobots) if (other != robot && other.parts.length > 0) {
      var index = other.parts[0].robotIndex;
      for (hull in other.hulls) if (!included.exists(hull.part) && ignored.indexOf(hull.part) < 0) {
        var pose = simulation.linkPose(index, hull.link);
        var worldPose = new Transform3(new Vec3(pose.position[0], pose.position[1], pose.position[2]),
          new Quat(pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]));
        var localPose = baseInverse.compose(worldPose), vertices:Array<Float> = [];
        for (corner in 0...Std.int(hull.vertices.length / 3)) {
          var point = localPose.transformPoint(new Vec3(hull.vertices[corner * 3], hull.vertices[corner * 3 + 1], hull.vertices[corner * 3 + 2]));
          vertices.push(point.x); vertices.push(point.y); vertices.push(point.z);
        }
        bodies.push({name: hull.part, link: links[0].id, vertices: vertices, tool: false});
        included.set(hull.part, true);
      }
    }
    for (entry in objects) {
      var prefix = "project:";
      var name = StringTools.startsWith(entry.id, prefix) ? entry.id.substr(prefix.length) : entry.id;
      if (included.exists(name) || ignored.indexOf(name) >= 0) continue;
      var half = entry.object.shape.boundingHalfExtents();
      if (half == null) throw 'Joint clearance needs finite collision geometry for "${entry.id}"';
      var pose = simulation.objectPose(entry.object);
      var worldPose = new Transform3(new Vec3(pose.x, pose.y, pose.z), new Quat(pose.qx, pose.qy, pose.qz, pose.qw));
      var carrier = simulation.objectCarrier(entry.object), attached = -1;
      for (index in 0...links.length)
        if (simulation.linkBody(robotIndex, index) == carrier) {attached = index; break;}
      var inverse = baseInverse;
      if (attached >= 0) {
        var body = simulation.linkPose(robotIndex, attached);
        inverse = new Transform3(new Vec3(body.position[0], body.position[1], body.position[2]),
          new Quat(body.rotation[0], body.rotation[1], body.rotation[2], body.rotation[3])).inverse();
      }
      var localPose = inverse.compose(worldPose), vertices:Array<Float> = [];
      for (corner in 0...8) {
        var point = localPose.transformPoint(new Vec3((corner & 1) == 0 ? -half.x : half.x,
          (corner & 2) == 0 ? -half.y : half.y, (corner & 4) == 0 ? -half.z : half.z));
        vertices.push(point.x); vertices.push(point.y); vertices.push(point.z);
      }
      bodies.push({name: name, link: links[attached >= 0 ? attached : 0].id, vertices: vertices, tool: attached >= 0});
      included.set(name, true);
    }
    for (hull in depositedWeldHulls()) {
      var vertices:Array<Float> = [];
      for (index in 0...Std.int(hull.vertices.length / 3)) {
        var point = baseInverse.transformPoint(new Vec3(hull.vertices[index * 3], hull.vertices[index * 3 + 1], hull.vertices[index * 3 + 2]));
        vertices.push(point.x); vertices.push(point.y); vertices.push(point.z);
      }
      bodies.push({name: hull.name, link: links[0].id, vertices: vertices, tool: false});
    }
    return bodies;
  }

  function armClearance(arm:Manipulator, bodies:Array<robotkit.manipulation.ArmClearance.ClearanceBodyData>):ArmClearance {
    return armClearanceAtMargin(arm, bodies, processkit.WeldPathPlanner.AIR_MARGIN);
  }

  function armClearanceAtMargin(arm:Manipulator,
      bodies:Array<robotkit.manipulation.ArmClearance.ClearanceBodyData>, margin:Float,
      ?interfaces:Array<robotkit.manipulation.ArmClearance.ClearanceInterfaceData>):ArmClearance {
    var positions = robot.robot.snapshot().positions;
    var indices = [for (target in arm.toJointTargets([for (_ in 0...arm.dofCount()) 0.0])) target.joint];
    clearanceJoints = indices;
    return new ArmClearance(arm, bodies, [for (index in indices) positions.get(index)], margin,
      ArmClearance.CONTACT_MARGIN, null, interfaces);
  }

  /**
   * The first pair of bodies closer than a weld is planned to keep them with the arm where it is now, or null (also when
   * the mission does not weld). With `contact` the torch is where it works, and may come as close as the contact margin.
   */
  public function clearanceViolation(contact:Bool):Null<ClearanceViolation> {
    var checked = clearance;
    if (checked == null) return null;
    var positions = robot.robot.snapshot().positions;
    return checked.violation([for (index in clearanceJoints) positions.get(index)], contact);
  }

  /** The arm that works the tool: its tool frame is the tool's contact connector, on the link that carries it. */
  function toolArm(tool:SceneArtifactRobotTool):Manipulator {
    // The arm's tool frame: the tool's contact connector on the link that carries it, made with the robot.
    var tcp = robot.toolFrames.get(tool.contact.occurrence);
    if (tcp == null) throw 'The robot has no frame for tool "${tool.contact.occurrence}"';
    var model = robot.model;
    return new Manipulator(model, model.links[0].id, tcp.id);
  }

  /** The boxes that stand in the robot's way: above the floor and below its height; the floor itself and overhead boxes are not. */
  public static function standing(boxes:Array<FloorObstacle>):Array<FloorObstacle>
    return robotkit.navigation.FloorMap.standing(boxes, CLEARANCE);

  /**
   * An occupancy grid covering the obstacles and `poses` with a margin: a cell is occupied when its
   * square overlaps a box that stands within the robot's height.
   */
  public static function floorPlan(obstacles:Array<FloorObstacle>, poses:Array<Pose2>, margin:Float = MARGIN):OccupancyGrid2
    return robotkit.navigation.FloorMap.rasterize(obstacles, poses, RESOLUTION, margin, CLEARANCE, FRAME);

  function configureHoming():Void {
    var axes:Array<MotionAxisBlueprint> = [], axisJoints = new Map<String, Int>();
    var names = robot.robot.description().joints;
    for (joint in 0...robot.blueprint.jointCount) {
      var required = false;
      for (contact in robot.blueprint.switches)
        if (contact.role == "home" && contact.joint == names[joint]) required = true;
      if (!required) continue;
      var physical = robot.blueprint.joints[joint];
      if (physical.maxRate == null || physical.maxAcceleration == null)
        throw "Mission home axis requires physical drive limits";
      axes.push(new MotionAxisBlueprint(names[joint], [names[joint]],
        physical.lowerLimit, physical.upperLimit, physical.maxRate, physical.maxAcceleration,
        Math.max(physical.lowerLimit, Math.min(physical.upperLimit, 0.0)), [1.0], [0.0]));
      axisJoints.set(names[joint], joint);
    }
    homingStarted = false; homingSeconds = 0.0;
    if (axes.length == 0) { homing = null; homingComplete = true; return; }
    var encoders = new EncoderMonitor(robot.model, robot.runtime.snapshot().q.toArray());
    var slip = new StepperSlip(robot.robot.description().couplings, axisJoints,
      (joint, offset) -> simulation.setJointSlip(robotIndex, joint, offset));
    homingEncoders = encoders; homingSlip = slip;
    var view = new MotionSystem(robot.robot, new MotionSystemBlueprint(robot.model, robot.blueprint, axes, timestep));
    view.configureRuntimeHoming(robot.runtime, () -> {
      slip.rebaseAfterHoming();
      encoders.reset(robot.runtime.snapshot().q.toArray());
    }, simulation.homingSides(robotIndex));
    homing = view; homingComplete = false;
  }

  /** User-facing progress, including the startup sequence before the first mission step. */
  public function statusLabel(detailed:Bool = false):String {
    if (failure != null) return detailed ? "Mission failed: " + failure : "Mission failed";
    if (finished) return "Mission complete";
    var homeView = homing;
    if (!homingComplete && homeView != null) {
      if (!homingStarted) return "Waiting for home switches";
      var axis = homeView.homingAxis();
      var label = "Homing" + (axis == null ? "" : " " + axis.split("/").pop().toUpperCase());
      if (!detailed) return label;
      var phase = switch homeView.homingStatus() {
        case "Seek": "finding switch";
        case "Backoff": "backing off switch";
        case "Approach": "locating switch edge";
        case "Return": "returning to home";
        default: "waiting for drive confirmation";
      };
      return label + " · " + phase;
    }
    return "Step " + (completed + 1) + " / " + mission.steps.length + " · " + mission.steps[stepIndex].kind;
  }

  public function feed():Void {
    if (recoveryPending) {
      advanceRecovery();
      return;
    }
    if (failure != null || finished) return;
    var runtimeState = robot.runtime.snapshot();
    if (runtimeState.safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT ||
        runtimeState.safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP ||
        runtimeState.endpoint == RobotKitRuntimeConstants.RK_ENDPOINT_FAULT) {
      failMission("Robot safety stop or runtime fault", MissionAlarmCode.EmergencyStop);
      return;
    }
    var readCellInput = cellInput;
    if (readCellInput != null) switch readCellInput(cnckit.controller.RobotInterfaceChannels.ALARM) {
      case Digital(true):
        failMission("CNC controller alarm", MissionAlarmCode.CncAlarm);
        return;
      case _:
    }
    var homeView = homing;
    if (!homingComplete && homeView != null) {
      try {
        if (!homingStarted) {
          var available = new Map<String, Bool>();
          for (frame in robot.robot.snapshot().sensors.toArray()) available.set(frame.sensorId, true);
          for (contact in robot.blueprint.switches)
            if (contact.role == "home" && !available.exists(contact.id)) return;
          homeView.home(); homingStarted = true;
          return;
        }
        homingSeconds += timestep;
        homeView.update(timestep);
        if (homeView.homingStatus() != "Complete") return;
        homingComplete = true;
        for (channel in newHandlingByChannel.keys()) {
          var make = newHandlingByChannel.get(channel);
          if (make != null) handlingRunners.set(channel, make());
        }
        var makeWelding = newWelding;
        if (makeWelding != null) welding = makeWelding();
      } catch (error:Dynamic) {
        failMission('Mission homing: $error', MissionAlarmCode.MotionFailure);
        return;
      }
    }
    var snapshot = robot.robot.snapshot();
    var activeStep = mission.steps[stepIndex];
    var releasing = activeStep.kind == "place" ? handlingToolChannel(activeStep) : null;
    for (channel in carryingTools.keys()) if (carryingTools.get(channel) == true && channel != releasing) {
      var tool = handlingTools.get(channel);
      if (tool == null || !toolSensorActive(snapshot, tool)) {
        var held = heldParts.get(channel);
        failMission('Grasp sensor for "$channel" opened while carrying ${held == null ? "a part" : held.occurrence}',
          MissionAlarmCode.PartDropped);
        return;
      }
    }
    localization.update(snapshot);
    trackWheels(snapshot);
    switch runner.status() {
      case Running:
      case _:
        if (runner.start(skillFor(mission.steps[stepIndex])) != Running) {
          settle();
          return;
        }
    }
    runner.update(snapshot, timestep);
    settle();
  }

  /** The step that was running stops through the robot's runtime, which still answers until the session resets. */
  public function beforeReset():Void {
    runner.cancel();
    deenergizeMissionOutputs();
    if (homing != null && homing.isMoving()) homing.abort();
  }

  /** Back to the first step; the next tick starts it from wherever the reset put the robot. */
  public function reset():Void {
    deenergizeMissionOutputs();
    runner = new SkillRunner();
    jointMotions = new Map<String, ManipulatorMotion>();
    registeredWork = new Map<String, Transform3>();
    stepIndex = 0;
    completed = 0;
    failure = null;
    alarmCode = null;
    failedStepIndex = -1;
    carryingTools.clear();
    recoveryPending = false;
    recoveryPulsePending = false;
    recoveryOutputPending = false;
    recoveryElapsed = 0.0;
    recoveryStep = -1;
    finished = false;
    heldParts.clear();
    currentGrasps.clear();
    heldToolOffsets.clear();
    heldToolRotations.clear();
    heldToolYaws.clear();
    heldApproaches.clear();
    handlingRunners.clear();
    for (channel in newHandlingByChannel.keys()) {
      var make = newHandlingByChannel.get(channel);
      if (make != null) handlingRunners.set(channel, make());
    }
    wheelsSeeded = false;
    scanned = null;
    sensed = new PerceptionSnapshot();
    configureHoming();
    var makeWelding = newWelding;
    if (makeWelding != null) welding = makeWelding();
  }

  public function robotId():robotkit.core.RobotId return robot.robot.id();

  /** The index of the weld step the robot is welding now, or -1 when it is not welding. */
  public function weldingStep():Int {
    if (failure != null || runner.status() != Running || mission.steps[stepIndex].kind != "weld") return -1;
    return stepIndex;
  }

  /** The last weld planned: how long it took, and how the torch comes in. */
  public function planReport():String {
    if (welding == null) return "";
    var active = cast(welding, WeldingPlanRunner);
    var plan = active.lastPlan();
    if (plan == null) return "";
    if(plan.selected)return '${Math.round(active.planningSeconds * 100) / 100} s, ${plan.checked} poses, globally selected joint path';
    return '${Math.round(active.planningSeconds * 100) / 100} s, ${plan.checked} poses, rolls ${[for (roll in plan.rolls) Math.round(roll * 180 / Math.PI)].join("/")} deg, in ${plan.entry.name}, out ${plan.exitName}';
  }

  /** How many times the arc was lost and the weld restarted, in the weld running or last run. */
  public function weldRestarts():Int {
    var active = welding;
    return active == null ? 0 : active.restarts();
  }

  public function present():Void {
    if (!recoveryPulsePending) return;
    recoveryPulsePending = false;
    try writeMissionOutput(cnckit.controller.RobotInterfaceChannels.FAULT_RESET, false) catch (_:Dynamic) {}
  }

  function failMission(message:String, code:MissionAlarmCode):Void {
    if (failure != null) return;
    failure = message;
    alarmCode = code;
    failedStepIndex = stepIndex;
    recoveryPending = false;
    recoveryPulsePending = false;
    recoveryOutputPending = false;
    // A latched mission fault must also abort the active motion or handling
    // plan; otherwise the robot would keep executing while the operator sees
    // a stopped mission.
    runner.cancel();
    deenergizeMissionOutputs();
  }

  function alarmForFailure(step:SceneArtifactMissionStep, message:String):MissionAlarmCode {
    var state = robot.runtime.snapshot();
    var emergencyStopped = state.safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT ||
        state.safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP ||
        state.endpoint == RobotKitRuntimeConstants.RK_ENDPOINT_FAULT;
    return MissionAlarmClassifier.classify(step, message, emergencyStopped, hasCarriedPart());
  }

  function toolSensorActive(snapshot:RobotSnapshot, tool:SceneArtifactRobotTool):Bool {
    var sensor = tool.sensor;
    var threshold = tool.kind == "gripper" ? 0.5 : 40.0;
    if (sensor == null) return false;
    for (index in 0...snapshot.sensors.length) {
      var frame = snapshot.sensors.get(index);
      if (frame.sensorId == sensor) return frame.values.length > 0 && frame.values.get(0) >= threshold;
    }
    return false;
  }

  function hasCarriedPart():Bool {
    for (channel in carryingTools.keys()) if (carryingTools.get(channel) == true) return true;
    return false;
  }

  /**
   * Clear an operator-corrected fault and continue at its owning command. A failed
   * wait can point to the command it protects; the finished-step count and object
   * state are deliberately retained.
   */
  public function retryFailedStep():Bool {
    if (failure == null || failedStepIndex < 0 || recoveryPending) return false;
    var step = mission.steps[failedStepIndex];
    var resume = failedStepIndex;
    if (step.kind == "waitFor" && step.waitFor != null && step.waitFor.retryStep != null)
      resume = step.waitFor.retryStep;
    if (resume < 0 || resume > failedStepIndex) return false;
    // The safety-reset command is asynchronous. Writing an energised controller
    // output in this call can race the runtime while it still reports the stop.
    // Defer the machine reset pulse until a later tick observes a ready robot.
    recoveryOutputPending = true;
    recoveryPulsePending = false;
    recoveryElapsed = 0.0;
    recoveryStep = resume;
    recoveryPending = true;
    return true;
  }

  function advanceRecovery():Void {
    recoveryElapsed += timestep;
    var read = cellInput;
    var activeAlarm = false;
    if (read != null) switch read(cnckit.controller.RobotInterfaceChannels.ALARM) {
      case Digital(value): activeAlarm = value;
      case _:
    }
    var state = robot.runtime.snapshot();
    var safetyStopped = state.safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT ||
      state.safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP ||
      state.endpoint == RobotKitRuntimeConstants.RK_ENDPOINT_FAULT;
    if (recoveryOutputPending && !safetyStopped) {
      try writeMissionOutput(cnckit.controller.RobotInterfaceChannels.FAULT_RESET, true) catch (error:Dynamic) {
        recoveryPending = false;
        recoveryOutputPending = false;
        recoveryStep = -1;
        failure = 'Could not request machine fault reset: $error';
        alarmCode = MissionAlarmCode.RecoveryTimeout;
        return;
      }
      recoveryOutputPending = false;
      recoveryPulsePending = true;
      return;
    }
    if (!activeAlarm && !safetyStopped && !recoveryPulsePending) {
      failure = null;
      alarmCode = null;
      failedStepIndex = -1;
      stepIndex = recoveryStep;
      recoveryStep = -1;
      recoveryPending = false;
      recoveryOutputPending = false;
      runner = new SkillRunner();
      jointMotions = new Map<String, ManipulatorMotion>();
      return;
    }
    if (recoveryElapsed >= 5.0) {
      recoveryPending = false;
      recoveryOutputPending = false;
      recoveryStep = -1;
      failure = "Fault reset did not clear the machine alarm; remove the cause and retry";
      alarmCode = MissionAlarmCode.RecoveryTimeout;
    }
  }

  /** After a tick: a finished step hands over to the next, a failed one stops the mission. */
  function settle():Void switch runner.status() {
    case Succeeded:
      var completedStep = mission.steps[stepIndex];
      if (completedStep.kind == "pick" || completedStep.kind == "place") {
        var channel = handlingToolChannel(completedStep);
        if (completedStep.kind == "pick") {
          carryingTools.set(channel, true);
          if (gripperTools.exists(channel)) captureHeldGripperTransform(channel);
        } else {
          carryingTools.remove(channel);
          heldParts.remove(channel);
          currentGrasps.remove(channel);
          heldToolOffsets.remove(channel);
          heldToolRotations.remove(channel);
          heldToolYaws.remove(channel);
          heldApproaches.remove(channel);
        }
      }
      // Different motion runners own their last commanded coordinates. A joint
      // move changes the handling station; a handling/process step invalidates
      // cached joint-move anchors before another controller takes over.
      if (mission.steps[stepIndex].kind == "moveJoints" || mission.steps[stepIndex].kind == "moveVia") {
        for (channel in newHandlingByChannel.keys()) {
          var make = newHandlingByChannel.get(channel);
          if (make != null) handlingRunners.set(channel, make());
        }
      } else {
        jointMotions = new Map<String, ManipulatorMotion>();
      }
      completed++;
      if (stepIndex + 1 < mission.steps.length || mission.loop == true)
        stepIndex = (stepIndex + 1) % mission.steps.length;
      else {
        finished = true;
        deenergizeMissionOutputs();
      }
    case Failed(message):
      var code = alarmForFailure(mission.steps[stepIndex], message);
      failMission('step $stepIndex (${mission.steps[stepIndex].kind}): $message', code);
    case _:
  }

  function configurationFor(step:SceneArtifactMissionStep):Null<motionkit.kinematics.SixAxisConfiguration> {
    if(step.configuration==null)return null;
    var pin:materia.project.SceneArtifact.SceneArtifactArmConfiguration=cast step.configuration;
    return new motionkit.kinematics.SixAxisConfiguration(pin.shoulder,pin.elbow,pin.wrist,pin.turns);
  }

  function skillFor(step:SceneArtifactMissionStep):Skill {
    switch step.kind {
      case "moveJoints":
        return withTimeout(step, jointMove(step));
      case "moveVia":
        return withTimeout(step, viaMove(step));
      case "signal":
        var command = step.signal;
        if (command == null) throw "Signal mission step has no command";
        return new MissionSignalSkill(command.name, command.value, writeMissionOutput);
      case "waitFor":
        var condition = step.waitFor;
        if (condition == null) throw "Wait mission step has no input condition";
        return new MissionWaitSkill(condition.name, condition.value, condition.timeout,
          missionInput);
      case "stow":
        return stow();
      case "goTo":
        registeredWork = new Map<String, Transform3>();
        var activeNavigator:Navigator = cast navigator;
        return new GoTo(activeNavigator, new NavigationGoal(floorPose(step), FRAME, POSITION_TOLERANCE, HEADING_TOLERANCE),
          observe);
      case "pick":
        var at = step.at;
        if (at == null) throw "Pick needs a mission frame";
        var channel = handlingToolChannel(step), tool = handlingTools.get(channel);
        if (tool == null) throw 'Pick selects unknown handling tool "$channel"';
        heldParts.set(channel, at);
        var pickRunner = handlingRunner(step, channel);
        pickRunner.configuration = configurationFor(step);
        return withTimeout(step, HandlePart.pick(pickRunner, localization,
          () -> graspPoint(at, channel), tool.sensor, graspThreshold(tool),
          handlingOrientation(step, channel), handlingApproach(step, channel)));
      case "place":
        var at = step.at;
        if (at == null) throw "Place needs a mission frame";
        var channel = handlingToolChannel(step), tool = handlingTools.get(channel);
        if (tool == null) throw 'Place selects unknown handling tool "$channel"';
        var placeRunner = handlingRunner(step, channel);
        placeRunner.configuration = configurationFor(step);
        return withTimeout(step, HandlePart.place(placeRunner, localization,
          () -> placeContact(at, step.yaw, channel), tool.sensor, graspThreshold(tool),
          handlingOrientation(step, channel), handlingApproach(step, channel)));
      case "weld":
        var weld = step.weld;
        if (weld == null) throw "Weld mission step has no weld definition";
        return new processkit.skill.WeldPasses([for (pass in weld.passes)
          () -> weldPassSkill(weld, pass,configurationFor(step))],
          [for (pass in weld.passes) pass.interpassDwell]);
      case "findWork":
        var job = step.contactWork;
        if (job == null) throw "Contact mission step has no work definition";
        registeredWork.remove(job.frame);
        var factory = newRegistration;
        if (factory == null) throw "Contact mission has no registration runner";
        return new processkit.skill.FindWeldWork(() -> factory(job), (frame) -> registeredWork.set(job.frame, frame));
      default:
        throw 'Mission step kind "${step.kind}" is not supported';
    }
  }

  function withTimeout(step:SceneArtifactMissionStep, skill:Skill):Skill {
    var seconds = step.timeoutSeconds;
    return seconds == null ? skill : new MissionTimedSkill(skill, step.kind, seconds);
  }

  function handlingToolChannel(step:SceneArtifactMissionStep):String {
    if (step.tool != null) return step.tool;
    var selected:Null<String> = null;
    for (channel in handlingTools.keys()) {
      if (selected != null) throw "A multi-tool handling step must select its tool channel";
      selected = channel;
    }
    if (selected == null) throw "Handling step has no configured tool";
    return selected;
  }

  static function graspThreshold(tool:SceneArtifactRobotTool):Float
    return tool.kind == "gripper" ? 0.5 : 40.0;

  function handlingRunner(step:SceneArtifactMissionStep, channel:String):HandlingPlanRunner {
    var active = handlingRunners.get(channel);
    if (active == null) throw 'Handling tool "$channel" has no arm runner';
    var approachDistance = step.approachDistance;
    return approachDistance == null ? active : new HandlingPlanRunner(active.motion, active.channel,
      active.home, active.homePose, approachDistance, active.travelSpeed, active.contactSpeed,
      active.dwell, active.pressDepth);
  }

  /** Plans absolute mechanical coordinates on the chain ending at the named connector. */
  function jointMove(step:SceneArtifactMissionStep):Skill {
    var at = step.at;
    if (at == null) throw "Joint motion needs a mission frame";
    var context = jointMotionContext(at);
    var q = startingJointPositions(context.motion, context.indices);
    setJointTargets(q, step.joints, context);
    context.motion.compiler.planValidator = planValidator(context.arm);
    return new MotionProgramSkill(context.motion, new MotionProgram([
      MotionOp.MoveJ(MoveTarget.JointTarget(q), new MotionOptions(), Blend.ExactStop)]));
  }

  /** A sequence of exact-stop Cartesian or joint points certified as timed arm motion. */
  function viaMove(step:SceneArtifactMissionStep):Skill {
    var at = step.at;
    var waypoints = step.poses;
    if (at == null || waypoints == null || waypoints.length == 0) throw "Via motion needs a frame and waypoints";
    var context = jointMotionContext(at);
    var q = startingJointPositions(context.motion, context.indices);
    var ops:Array<MotionOp> = [];
    for (waypoint in waypoints) {
      if (waypoint.joints != null) {
        setJointTargets(q, waypoint.joints, context);
        ops.push(MotionOp.MoveJ(MoveTarget.JointTarget(q.copy()), new MotionOptions(), Blend.ExactStop));
      } else {
        var pose = waypoint.pose;
        if (pose == null) throw "Via waypoint has no target";
        ops.push(MotionOp.MoveJ(MoveTarget.PoseTarget(Pose3.fromArrays(pose.position, pose.rotation), FRAME, null),
          new MotionOptions(), Blend.ExactStop));
      }
    }
    context.motion.compiler.planValidator = planValidator(context.arm);
    return new MotionProgramSkill(context.motion, new MotionProgram(ops));
  }

  function jointMotionContext(at:SceneArtifactPlace):JointMotionContext {
    var name = AssemblyRobot.missionFrameName(at);
    var frame = robot.missionFrames.get(name);
    if (frame == null) throw 'The robot has no mission frame "$name"';
    var arm = new Manipulator(robot.model, robot.model.links[0].id, frame.id);
    var count = arm.group.count();
    var indices = [for (target in arm.toJointTargets([for (_ in 0...count) 0.0])) target.joint];
    var motion = jointMotions.get(name);
    if (motion == null) {
      var limits = new ValidationLimits(count, Int64.ofInt(1), Int64.ofInt(0));
      var velocities:Array<Float> = [];
      var accelerations:Array<Float> = [];
      var jerks:Array<Float> = [];
      for (i in 0...count) {
        var bound = arm.group.limitsOf(i);
        if (bound.lower < bound.upper) limits.position(i, bound.lower, bound.upper);
        var rates = jointMotionPolicy.rates(robot.model.joints[indices[i]].type, bound);
        velocities.push(rates.velocity);
        accelerations.push(rates.acceleration);
        jerks.push(rates.jerk);
        limits.velocity(i, rates.velocity);
        limits.acceleration(i, rates.acceleration);
        limits.jerk(i, rates.jerk);
      }
      var compiler = new ProgramCompiler(new ManipulatorKinematics(arm, 1e-8), limits, FRAME,
        velocities, accelerations, jerks,
        StartTolerances.uniform(count, 0.005, 0.02, 0.2));
      motion = new ManipulatorMotion(robot.robot, compiler,
        channel -> cellInput == null ? null : cellInput(channel), () -> robot.runtime.pollEvents(), indices);
      jointMotions.set(name, motion);
    }
    motion.reset();
    return {motion: motion, arm: arm, indices: indices};
  }

  function startingJointPositions(motion:ManipulatorMotion, indices:Array<Int>):Array<Float> {
    var q = motion.commandedPositions();
    if (q == null) {
      var positions = robot.robot.snapshot().positions;
      q = [for (index in indices) positions.get(index)];
    }
    return q;
  }

  function setJointTargets(q:Array<Float>, targets:Array<materia.project.SceneArtifact.SceneArtifactJointTarget>,
      context:JointMotionContext):Void {
    if (targets == null) throw "Joint motion has no targets";
    for (target in targets) {
      var found = false;
      for (i in 0...context.indices.length) {
        var joint = robot.model.joints[context.indices[i]];
        if (joint.name != target.joint) continue;
        if (placement == null || assembly == null) throw "Joint motion needs an assembly placement";
        var authored = [for (item in assembly.joints) if (item.id == target.joint) item];
        if (authored.length != 1) throw 'No mechanical joint "${target.joint}"';
        var scale = Std.string(authored[0].type) == "prismatic" ? metres : 1.0;
        q[i] = target.position - placement.joint(target.joint) * scale;
        found = true;
      }
      if (!found) throw 'Joint "${target.joint}" is outside the selected mission chain';
    }
  }

  function planValidator(arm:Manipulator):ArmPlanClearance {
    var interfaces:Array<robotkit.manipulation.ArmClearance.ClearanceInterfaceData> = [
      for (pair in robot.jointInterfaces) {first: pair.first, second: pair.second, margin: ArmClearance.INTERFACE_MARGIN}
    ];
    return new ArmPlanClearance(cellClearance(arm, null, [], ArmClearance.MARGIN, interfaces));
  }

  function writeMissionOutput(name:String, value:Bool):Void {
    var writer = cellOutput;
    if (writer == null) throw "Signal mission step has no cell output bus";
    writer(name, value);
    if (value) missionOutputs.set(name, true) else missionOutputs.remove(name);
  }

  /** Read a controller-to-controller wire first, then this robot's local field sensors. */
  function missionInput(name:String):Null<EventValue> {
    var read = cellInput;
    if (read != null) {
      var value = read(name);
      if (value != null) return value;
    }
    var sensors = robot.runtime.snapshot().sensors;
    for (index in 0...sensors.length) {
      var frame = sensors.get(index);
      if (frame.sensorId == name && frame.values.length > 0) {
        var value = frame.values.get(0);
        return EventValue.Digital(value >= 0.5);
      }
    }
    return null;
  }

  function deenergizeMissionOutputs():Void {
    var writer = cellOutput;
    if (writer == null) { missionOutputs = new Map<String, Bool>(); return; }
    for (name in missionOutputs.keys()) try writer(name, false) catch (_:Dynamic) {}
    missionOutputs = new Map<String, Bool>();
  }

  /** Plan the mobile welding stow from measured joints against the live work and deposited bead geometry. */
  function stow():Skill {
    var torches = [for (tool in project.robotTools) if (tool.kind == "torch") tool];
    if (torches.length != 1) throw 'A welding stow needs one torch, the robot has ${torches.length}';
    var tool = torches[0];
    var arm = toolArm(tool);
    var metal = [for (step in mission.steps) if (step.kind == "weld") cast(step.weld, SceneArtifactWeld).metal];
    var planned = weldClearance(arm, tool.contact.occurrence, metal);
    clearance = planned;
    var planning = WeldingPlanRunner.planning(arm, weldAcceleration, planned);
    var indices = arm.jointIndices();
    var observed = robot.robot.snapshot().positions;
    var from = [for (index in indices) observed.get(index)];
    // CadBridge bakes the assembly's authored home joints into the model; zero is its CAD ready posture.
    var ready = [for (_ in indices) 0.0];
    var program = WeldStowPlanner.plan(arm, planning.compiler, planned, from, ready);
    var motion = new ManipulatorMotion(robot.robot, planning.compiler, (_) -> null,
      () -> robot.runtime.pollEvents(), indices);
    return new MotionProgramSkill(motion, program);
  }

  /** Builds one weld pass using the shared welding runner. */
  function weldPassSkill(weld:SceneArtifactWeld, pass:materia.project.SceneArtifact.SceneArtifactWeldPass,?configuration:motionkit.kinematics.SixAxisConfiguration):Skill {
    var factory = newWelding;
    if (factory == null) throw "Weld mission has no welding runner";
    welding = factory();
    var active:Null<WeldingPlanRunner> = welding;
    var sensor = weldSensor;
    if (active == null || sensor == null) throw "Weld mission has no active welding runner or sensor";
    active.configuration = configuration;
    if (requiresRegistration(weld)) {
      if (wheels == null) throw "Registered welding has no wheel pose estimate";
      return new WeldSeam(active, wheels, () -> weldPlan(weld, pass), sensor);
    }
    return new WeldSeam(active, localization, () -> weldPlan(weld, pass), sensor);
  }

  function requiresRegistration(weld:SceneArtifactWeld):Bool {
    for (step in mission.steps) if (step.kind == "findWork" && step.contactWork != null && step.contactWork.frame == weld.frame) return true;
    return false;
  }

  /** The world pose of a weld's reference member now, or the world's own when the weld names none. */
  function referenceFrame(weld:SceneArtifactWeld):Transform3 {
    if (weld.frame == null || weld.frame == "") return Transform3.identity();
    if (requiresRegistration(weld)) {
      var measured = registeredWork.get(weld.frame);
      var estimate = wheels == null ? null : wheels.state();
      if (measured == null || estimate == null) throw "Weld work has no accepted registration at this station";
      return Transform3.fromPose2(estimate.pose).compose(measured);
    }
    var live = assemblyParts.get("project:" + weld.frame).pose();
    return new Transform3(new Vec3(live.position[0], live.position[1], live.position[2]),
      new Quat(live.rotation[0], live.rotation[1], live.rotation[2], live.rotation[3]));
  }

  /**
   * The weld a mission step describes, as a plan in the map frame (the world). Its path is given relative to the
   * workpiece's reference member, and that member is found where it stands when the step starts, as a pick finds its part,
   * so the weld follows a workpiece that is not where it was designed. A weld with no frame is in the assembly as designed.
   */
  function weldPlan(weld:SceneArtifactWeld, pass:materia.project.SceneArtifact.SceneArtifactWeldPass):WeldPlan
    return processkit.WeldScenePlan.pass(weld, pass, referenceFrame(weld));

  /**
   * What the lidar sees beyond the map, in the map frame, read once per scan: a scan's obstacles stay where
   * they were seen until the next one, however far the robot moves meanwhile.
   */
  function observe(snapshot:RobotSnapshot):PerceptionSnapshot {
    var reading = scanning;
    var name = scanner;
    if (reading == null || name == null) return idle;
    var sequence:Null<Int64> = null;
    for (frame in snapshot.streamSequences) if (frame.streamId == name) sequence = frame.sequence;
    if (sequence == null) return idle;
    var latest:Int64 = cast sequence;
    var before = scanned;
    if (before != null && Int64.compare(before, latest) == 0) return sensed;
    scanned = latest;
    sensed = reading.observeRobotSnapshot(snapshot, robot.model, robot.blueprint, robot.rootLink, robot.robot.streams().latestFrames());
    return sensed;
  }

  /** Runs wheel odometry beside the truth, starting from where the truth stands. */
  function trackWheels(snapshot:RobotSnapshot):Void {
    var odometry = wheels;
    var truth = localization.state();
    if (odometry == null || truth == null) return;
    if (!wheelsSeeded) {
      odometry.reset(truth.pose);
      wheelsSeeded = true;
    }
    odometry.update(snapshot);
  }

  /** The route, map, sensed obstacles, odometry and guard as a view draws them; null when the mission does not drive. */
  public function overlay():Null<MissionOverlay> {
    var map = costmap;
    var driver = navigator;
    var base = robot.mobile;
    if (map == null || driver == null || base == null || base.footprint == null) return null;
    var path = driver.activePath;
    var estimate = wheels == null ? null : wheels.state();
    var outline:Array<Pose2> = [];
    if (estimate != null) {
      var footprint:robotkit.mobile.Footprint = cast base.footprint;
      outline = [for (corner in footprint.vertices()) estimate.pose.compose(new Pose2(corner.x, corner.y))];
    }
    return {route: path == null ? [] : path.poses(), costmap: map, obstacles: map.dynamicLayer(),
      odometry: estimate == null ? null : estimate.pose, outline: outline,
      guard: guard == null ? MotionGuardState.Clear : guard.state};
  }

  /** What the navigator is doing about the current goal. */
  public function navigating():String return navigator == null ? "idle" : Std.string(navigator.status);

  /** How many times the navigator has replanned the current goal. */
  public function replans():Int return navigator == null ? 0 : navigator.replanCount;

  /** Where the grasp connector `at` of a free part is now, in metres. */
  function graspPoint(at:SceneArtifactPlace, channel:String):Vec3 {
    if (gripperTools.exists(channel)) {
      var selected = findGrasp(at, channel);
      currentGrasps.set(channel, selected);
      var frame = partFrame(at.occurrence);
      var point = AssemblyFrames.transformPoint(frame, selected.contact.x / 1000,
        selected.contact.y / 1000, selected.contact.z / 1000);
      return new Vec3(point.x, point.y, point.z);
    }
    var frame = partFrame(at.occurrence);
    var local = connectorFrame(at);
    var point = AssemblyFrames.transformPoint(frame, local.x * metres, local.y * metres, local.z * metres);
    return new Vec3(point.x, point.y, point.z);
  }

  /**
   * Where the tool's contact must be to set the held part down on the seat `at`: the seat, offset by
   * where the tool's contact now stands from the part's origin, which rests on the seat. Both are read
   * as they are, so a grip a little off the grasp point still sets the part down where it belongs.
   */
  function placeContact(at:SceneArtifactPlace, ?yaw:Float, channel:String):Vec3 {
    var held = heldParts.get(channel);
    if (held == null) throw "Nothing was picked to place";
    var seat = AssemblyFrames.compose(staticPose(at.occurrence), connectorFrame(at));
    if (gripperTools.exists(channel)) {
      var offset = heldToolOffsets.get(channel);
      if (offset == null) throw "The gripper has no measured tool-to-part transform";
      var rotation = placementRotation(at.occurrence, yaw);
      var shifted = rotation.rotate(offset);
      return new Vec3(seat.x * metres + shifted.x, seat.y * metres + shifted.y, seat.z * metres + shifted.z);
    }
    var origin = partFrame(held.occurrence);
    var tool = toolContact(channel);
    var offset = new Vec3(tool.x - origin.x, tool.y - origin.y, tool.z - origin.z);
    if (yaw != null) offset = yawRotation(yaw - frameYaw(origin)).rotate(offset);
    return new Vec3(seat.x * metres + offset.x, seat.y * metres + offset.y, seat.z * metres + offset.z);
  }

  static function frameYaw(frame:AssemblyFrame):Float {
    var x = new Quat(frame.qx, frame.qy, frame.qz, frame.qw).rotate(new Vec3(1, 0, 0));
    return Math.atan2(x.y, x.x);
  }

  static function yawRotation(yaw:Float):Quat return Quat.fromAxisAngle(new Vec3(0, 0, 1), yaw);

  /** The target tool roll is absolute; a held part carries the roll used when it was grasped. */
  function toolYawDelta(channel:String, targetYaw:Null<Float>):Float {
    if (targetYaw == null) return 0.0;
    var pickedYaw = heldToolYaws.get(channel);
    return targetYaw - (pickedYaw == null ? 0.0 : pickedYaw);
  }

  /** Fix the part's heading while preserving the tool-to-part grasp rotation. */
  function handlingOrientation(step:SceneArtifactMissionStep, channel:String):Null<Void -> Quat> {
    if (gripperTools.exists(channel)) {
      if (step.kind == "pick") return () -> {
        var selected = currentGrasps.get(channel);
        var at = step.at;
        if (selected == null || at == null) throw "Gripper grasp frame was not derived before motion";
        var frame = partFrame(at.occurrence);
        var part = frameRotation(frame);
        heldToolRotations.set(channel, selected.toolRotation);
        heldApproaches.set(channel, selected.approachAxis);
        var targetYaw = step.toolYaw;
        if (targetYaw == null) {
          heldToolYaws.remove(channel);
          return part.multiply(selected.toolRotation);
        }
        heldToolYaws.set(channel, targetYaw);
        return yawRotation(targetYaw).multiply(part.multiply(selected.toolRotation));
      };
      if (step.kind == "place") return () -> {
        var at = step.at;
        var relative = heldToolRotations.get(channel);
        if (at == null || relative == null) throw "Gripper place has no measured grasp orientation";
        var placed = placementRotation(at.occurrence, step.yaw).multiply(relative);
        var correction = toolYawDelta(channel, step.toolYaw);
        return correction == 0.0 ? placed : yawRotation(correction).multiply(placed);
      };
      return null;
    }
    var yaw = step.yaw;
    var toolYaw = step.toolYaw;
    if (yaw == null && toolYaw == null) return null;
    return () -> {
      if (step.kind == "pick") {
        var runner = handlingRunner(step, channel);
        var home = runner.homePose;
        var state = localization.state();
        if (state == null) throw "Handling orientation needs a localized base";
        var worldHome = Transform3.fromPose2(state.pose).rotation.multiply(
          new Quat(home.qx, home.qy, home.qz, home.qw));
        var at = step.at;
        if (at == null) throw "Pick needs a mission frame";
        var target = yaw == null ? worldHome
          : yawRotation(yaw - frameYaw(staticPose(at.occurrence))).multiply(worldHome);
        if (toolYaw == null) {
          heldToolYaws.remove(channel);
          return target;
        }
        heldToolYaws.set(channel, toolYaw);
        return yawRotation(toolYaw).multiply(target);
      }
      var held = heldParts.get(channel);
      var kinematics = toolKinematics.get(channel);
      if (held == null || kinematics == null) throw "A heading-constrained place needs a held part and tool";
      var tip = kinematics.tip;
      var link = simulation.linkPose(robotIndex, kinematics.link);
      var rotation = new Quat(link.rotation[0], link.rotation[1], link.rotation[2], link.rotation[3])
        .multiply(new Quat(tip.qx, tip.qy, tip.qz, tip.qw));
      var target = yaw == null ? rotation
        : yawRotation(yaw - frameYaw(partFrame(held.occurrence))).multiply(rotation);
      if (toolYaw == null) return target;
      var correction = toolYawDelta(channel, toolYaw);
      return correction == 0.0 ? target : yawRotation(correction).multiply(target);
    };
  }

  /** The world-space direction away from the grasp surface, used for approach and retract. */
  function handlingApproach(step:SceneArtifactMissionStep, channel:String):Null<Void -> Vec3> {
    if (!gripperTools.exists(channel)) return null;
    if (step.kind == "pick") return () -> {
      var selected = currentGrasps.get(channel);
      var at = step.at;
      if (selected == null || at == null) throw "Gripper approach was not derived before motion";
      return frameRotation(partFrame(at.occurrence)).rotate(selected.approachAxis).normalized();
    };
    if (step.kind == "place") return () -> {
      var at = step.at;
      var approach = heldApproaches.get(channel);
      if (at == null || approach == null) throw "Gripper place has no grasp approach frame";
      var result = placementRotation(at.occurrence, step.yaw).rotate(approach);
      var correction = toolYawDelta(channel, step.toolYaw);
      if (correction != 0.0) result = yawRotation(correction).rotate(result);
      return result.normalized();
    };
    return null;
  }

  function findGrasp(at:SceneArtifactPlace, channel:String):GraspFrame {
    var tool = gripperTools.get(channel);
    if (tool == null) throw "Mission gripper has no physical jaw limits";
    var limit = tool.gripper;
    if (limit == null) throw "Mission gripper has no physical jaw limits";
    var definitionId = definitionOf(at.occurrence);
    var measuredFaces = project.assemblyPlanarFacesFor(definitionId);
    var physical = project.projectPhysical;
    if (measuredFaces == null || physical == null)
      throw 'Part "${at.occurrence}" has no saved planar-face and collision-hull data for grasp planning';
    var parts = [for (part in physical.parts) if (part.id == definitionId) part];
    if (parts.length != 1)
      throw 'Part "${at.occurrence}" has no usable convex hull for grasp planning';
    var vertices = parts[0].collisionHull;
    if (vertices == null || vertices.length < 12 || vertices.length % 3 != 0)
      throw 'Part "${at.occurrence}" has no usable convex hull for grasp planning';
    var millimetresPerUnit = metres * 1000;
    var faces:Array<GraspPlane> = [for (face in measuredFaces) {
      origin: new Vec3(face.origin.x * millimetresPerUnit, face.origin.y * millimetresPerUnit,
        face.origin.z * millimetresPerUnit),
      normal: new Vec3(face.normal.x, face.normal.y, face.normal.z)
    }];
    var hull:Array<Vec3> = [];
    for (index in 0...Std.int(vertices.length / 3))
      hull.push(new Vec3(vertices[index * 3] * millimetresPerUnit,
        vertices[index * 3 + 1] * millimetresPerUnit, vertices[index * 3 + 2] * millimetresPerUnit));
    var connector = connectorFrame(at);
    // Face connectors define their outward normal on local +Y (ConnectorRole.Face),
    // including the top face's +Z normal. +Z is the gripper finger insertion axis.
    var approach = AssemblyFrames.transformVector(connector, 0, 1, 0);
    return Grasp.find(faces, hull, new Vec3(approach.x, approach.y, approach.z),
      limit.minimumPartWidthMm, limit.nominalPartWidthMm, limit.maximumPartWidthMm, limit.fingerLengthMm);
  }

  function captureHeldGripperTransform(channel:String):Void {
    var at = heldParts.get(channel);
    if (at == null) throw "Gripper picked a part without a grasp occurrence";
    var part = partFrame(at.occurrence);
    var rotation = frameRotation(part);
    var origin = new Vec3(part.x, part.y, part.z);
    heldToolOffsets.set(channel, rotation.conjugate().rotate(toolContact(channel).sub(origin)));
    heldToolRotations.set(channel, rotation.conjugate().multiply(toolRotation(channel)));
  }

  function toolRotation(channel:String):Quat {
    var kinematics = toolKinematics.get(channel);
    if (kinematics == null) throw 'The mission has no tool frame for "$channel"';
    var tip = kinematics.tip;
    var link = simulation.linkPose(robotIndex, kinematics.link);
    return new Quat(link.rotation[0], link.rotation[1], link.rotation[2], link.rotation[3])
      .multiply(new Quat(tip.qx, tip.qy, tip.qz, tip.qw));
  }

  function placementRotation(occurrence:String, ?yaw:Float):Quat {
    var pose = staticPose(occurrence);
    var result = frameRotation(pose);
    return yaw == null ? result : yawRotation(yaw - frameYaw(pose)).multiply(result);
  }

  static function frameRotation(frame:AssemblyFrame):Quat
    return new Quat(frame.qx, frame.qy, frame.qz, frame.qw);

  /** Where the tool's contact is now, in metres. */
  function toolContact(channel:String):Vec3 {
    var kinematics = toolKinematics.get(channel);
    if (kinematics == null) throw 'The mission has no tool frame for "$channel"';
    var tip = kinematics.tip;
    var link = simulation.linkPose(robotIndex, kinematics.link);
    var at:AssemblyFrame = {x: link.position[0], y: link.position[1], z: link.position[2],
      qx: link.rotation[0], qy: link.rotation[1], qz: link.rotation[2], qw: link.rotation[3]};
    var point = AssemblyFrames.transformPoint(at, tip.x, tip.y, tip.z);
    return new Vec3(point.x, point.y, point.z);
  }

  /** The live frame of a free part's own origin, in metres: its simulated pose less its geometry's centre. */
  function partFrame(occurrence:String):AssemblyFrame {
    var entry = [for (item in objects) if (item.id == "project:" + occurrence) item];
    if (entry.length != 1) throw 'Part "$occurrence" is not a free object of the simulation';
    var pose = simulation.objectPose(entry[0].object);
    var centre = project.assemblyPreviewCenter(definitionOf(occurrence));
    if (centre == null) throw 'Part "$occurrence" has no preview centre';
    var at:AssemblyFrame = {x: pose.x, y: pose.y, z: pose.z, qx: pose.qx, qy: pose.qy, qz: pose.qz, qw: pose.qw};
    return AssemblyFrames.compose(at, AssemblyFrames.translation(-centre[0] * metres, -centre[1] * metres, -centre[2] * metres));
  }

  /** A held part's frame in the assembly as designed, in CAD units. */
  function staticPose(occurrence:String):AssemblyFrame {
    var state = placement;
    if (state == null) throw "The mission has no assembly to place on";
    return state.worldPose(occurrence);
  }

  /** The connector named by `at`, in its occurrence's frame and CAD units. */
  function connectorFrame(at:SceneArtifactPlace):AssemblyFrame {
    var flat = assembly;
    if (flat == null) throw "The mission has no assembly";
    return AssemblyRobot.connectorFrame(flat, at);
  }

  function definitionOf(occurrence:String):String {
    var flat = assembly;
    if (flat == null) throw "The mission has no assembly";
    for (item in flat.occurrences) if (item.id == occurrence) return item.definition;
    throw 'The assembly has no occurrence "$occurrence"';
  }

  static function floorPose(step:SceneArtifactMissionStep):Pose2 {
    var pose = step.pose;
    if (pose == null) throw 'Mission step "${step.kind}" needs a pose';
    return new Pose2(pose.x, pose.y, pose.yaw);
  }
}
