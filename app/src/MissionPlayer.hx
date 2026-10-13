package app;

import robotkit.collision.CollisionSceneBuilder;
import cadbridge.CellCollisionSnapshot;
import robotkit.collision.CollisionSceneBuilder.CollisionOwner;

import collisionkit.native.NativeCollisionWorld;
import motionkit.robot.CollisionPlannerSpace;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import motionkit.robot.ManipulatorPlannerSpace;
import motionkit.planner.MotionPlanner;
import motionkit.planner.PlannerSpace;
import motionkit.planner.RrtConnect;
import app.SimulatedTools.GripObject;
import cadkit.modeling.AssemblyState;
import haxe.Int64;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactMission;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
import materia.project.SceneArtifact.SceneArtifactMobileBase;
import materia.project.SceneArtifact.SceneArtifactMobileEnergy;
import materia.project.SceneArtifact.SceneArtifactPlace;
import materia.project.SceneArtifact.SceneArtifactRobotTool;
import materia.project.SceneArtifact.SceneArtifactTorch;
import materia.project.SceneArtifact.SceneArtifactTorchPose;
import materia.project.SceneArtifact.SceneArtifactMissionWaypoint;
import materia.project.SceneArtifact.SceneArtifactWeld;
import materia.project.SceneContactRegistration.SceneContactWork;
import motionkit.robot.HandlingPlanRunner;
import motionkit.kinematics.IkTolerance;
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
import motionkit.robot.JointMotionPolicy;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.robot.StepperSlip;
import robotkit.runtime.EncoderMonitor;
import motionkit.robot.PlanningLimits;
import processkit.WeldingPlanRunner;
import processkit.WeldStowPlanner;
import processkit.skill.WeldPlan;
import processkit.skill.WeldSeam;
import processkit.tool.WeldArcModel;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.collision.CollisionClearance;
import robotkit.manipulation.ClearanceViolation;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.Grasp;
import robotkit.manipulation.Grasp.GraspFrame;
import robotkit.manipulation.Grasp.GraspPlane;
import robotkit.model.Frame;
import robotkit.model.SteadyLoads;
import robotkit.skill.HandlePart;
import robotkit.spatial.Vec3;
import robotkit.localization.SimulationTruthLocalization;
import robotkit.localization.Localization;
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
import cnckit.controller.RobotInterfaceAlarm;
import cnckit.controller.RobotInterfaceChannels;

/** A box held where it stands: centre, half extents and heading, in metres and radians. */
typedef FloorObstacle = robotkit.navigation.FloorMap.FloorBox;

private typedef InterfaceAlarmRead = {
  var valid:Bool;
  var code:RobotInterfaceAlarm;
}

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

private typedef CachedJointMotion = {
  var motion:ManipulatorMotion;
  var space:Null<ManipulatorPlannerSpace>;
}

private typedef HandlingToolKinematics = {var link:Int; var tip:AssemblyFrame;}

typedef MissionProgress = {
  var phase:String;
  var stepIndex:Int;
  var completedInLoop:Int;
  var loop:Int;
  var failure:Null<String>;
}

private enum EnergyTransferStage {
  NoEnergyTransfer;
  StowingForDock;
  ToChargingDock;
  SettlingAtDock;
  ChargingAtDock;
  ReturningFromDock;
  SettlingAtStation;
}

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
  var jointMotions = new Map<String, CachedJointMotion>();
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
  /** Contact-registered work frames in the map frame, retained across dock excursions. */
  var registeredWorkWorld = new Map<String, Transform3>();
  /** What welds are planned clear of, and where the arm's joints are among the robot's. */
  var clearance:Null<CollisionClearance> = null;
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
  /** Map localization shared by navigation and accepted work frames. */
  final localization:Localization;
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
  final energyPower:Null<SimulatedMobilePower>;
  var energyStage:EnergyTransferStage = NoEnergyTransfer;
  var energyRunner:Null<SkillRunner> = null;
  var energyReturnPose:Null<Pose2> = null;
  var energyHandoffAt:Int64 = Int64.ofInt(0);
  var energyTargetWattHours:Float = 0.0;
  var energyChargeStartWattHours:Float = 0.0;
  var energyReadyStep:Int = -1;
  public var energyDockTrips(default, null):Int = 0;
  public var energyChargedWattHours(default, null):Float = 0.0;
  public var energyChargeSeconds(default, null):Float = 0.0;

  public function new(mission:SceneArtifactMission, robot:AssemblyRobot, simulation:Simulation, robotIndex:Int,
      timestep:Float, obstacles:Array<FloorObstacle>, objects:Array<GripObject>, project:ProjectDocumentSession, cellRobots:Array<AssemblyRobot>, ?cellInput:String->Null<EventValue>, ?cellOutput:String->Bool->Void, ?jointMotionPolicy:JointMotionPolicy, weldAcceleration:Float = ARM_ACCELERATION, ?energyPower:SimulatedMobilePower) {
    if (!Math.isFinite(weldAcceleration) || weldAcceleration <= 0.0 || weldAcceleration > ARM_ACCELERATION)
      throw "Invalid welding planning acceleration";
    if (mission.steps.length == 0) throw "A mission needs steps";
    this.mission = mission;
    this.robot = robot;
    this.cellRobots = cellRobots.copy();
    if (this.cellRobots.indexOf(robot) < 0) this.cellRobots.push(robot);
    this.timestep = timestep;
    this.cellInput = cellInput;
    this.cellOutput = cellOutput;
    this.jointMotionPolicy = jointMotionPolicy == null ? JointMotionPolicy.mission() : jointMotionPolicy;
    this.obstacles = standing(obstacles);
    this.simulation = simulation;
    this.weldAcceleration = weldAcceleration;
    this.energyPower = energyPower;
    this.objects = objects;
    this.robotIndex = robotIndex;
    this.project = project;
    assemblyParts = new SimulationAssemblyParts(simulation, this.cellRobots, objects, project);
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
      var lidars = [for (sensor in project.robotSensors) if (sensor.kind == "lidar") sensor];
      if (lidars.length > 1) throw 'A mission reads one lidar, the robot has ${lidars.length}';
      // SceneArtifact lidar sensors scan the full circle. That coverage permits a guarded
      // reverse correction at a path end; without it the mission remains forward-only.
      var navigation = new Navigation(base, localization, 0.35, 0.5, 1.2, lidars.length == 1);
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
            HandlingPlanRunner.DEFAULT_APPROACH_HEIGHT, 0.3, 0.08, 0.4, isGripper ? 0.0 : 0.003, isGripper,
            isGripper ? HandlingPlanRunner.gripperTolerance() : null);
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
        var scene = weldCollisionScene(arm, tool.contact.occurrence, metal);
        var tip = weldKinematics.tip;
        var wire = processkit.ProbeWire.describe(scene, arm, robot.model.links[weldKinematics.link].id,
          new Transform3(new Vec3(tip.x, tip.y, tip.z), new Quat(tip.qx, tip.qy, tip.qz, tip.qw)),
          torch.wireDiameterMm * 0.001, torch.stickoutMm * 0.001,
          scene.memberObjects(tool.members), scene.memberObjects(weldingMembers()));
        var planned = scene.clearance(() -> new NativeCollisionWorld());
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
   * the tool, and the arm's current pose. Welding keeps 3 mm between installed mechanism bodies,
   * 5 mm to unrelated cell obstacles and 0.5 mm between tool and named work at the seam.
   * The arm's installed gearheads have 4.5 mm designed clearance to the next tube. `ignored` are occurrences that
   * are no obstacle (the weld metal).
   */
  function weldClearance(arm:Manipulator, torch:String, ignored:Array<String>):CollisionClearance
    return weldCollisionScene(arm, torch, ignored).clearance(() -> new NativeCollisionWorld());

  function weldingMembers():Array<String> {
    var members:Array<String> = [];
    for (step in mission.steps) if (step.weld != null)
      for (segment in step.weld.path) for (face in segment.faces)
        if (members.indexOf(face.member) < 0) members.push(face.member);
    for (step in mission.steps) if (step.contactWork != null)
      for (face in step.contactWork.faces) if (members.indexOf(face.member) < 0) members.push(face.member);
    return members;
  }

  function weldCollisionScene(arm:Manipulator, torch:String, ignored:Array<String>):CollisionSceneBuilder {
    var scene = cellCollisionSnapshot(ignored).describe(arm, observedJoints(arm), CollisionClearance.MARGIN);
    processkit.WeldCollisionPolicy.apply(scene, scene.memberObjects(weldingMembers()), processkit.WeldPathPlanner.AIR_MARGIN, CollisionClearance.CONTACT_MARGIN);
    return scene;
  }

  /** Read live poses and carrier facts; CadBridge owns the collision geometry collection. */
  function cellCollisionSnapshot(ignored:Array<String>):CellCollisionSnapshot {
    function linkPose(index:Int, link:Int):Transform3 {
      var pose = simulation.linkPose(index, link);
      return Transform3.fromArrays(pose.position, pose.rotation);
    }
    var captured = new CellCollisionSnapshot(robot.robot.id(), linkPose(robotIndex, 0), ignored);
    var toolMembers:Array<String> = [];
    for (tool in project.robotTools) for (member in tool.members) if (toolMembers.indexOf(member) < 0) toolMembers.push(member);
    captured.mechanism(robot.robot.id(), robot.model, robot.shapes,
      [for (index in 0...robot.model.links.length) linkPose(robotIndex, index)], toolMembers,
      robotkit.kinematics.RobotKinematics.observedState(robot.model, robot.robot.snapshot().positions.toArray()));
    var heldCarriers = new Map<String, String>();
    for (channel in carryingTools.keys()) if (carryingTools.get(channel) == true) {
      var held = heldParts.get(channel), tool = toolKinematics.get(channel);
      if (held != null && tool != null) heldCarriers.set(held.occurrence, robot.model.links[tool.link].id);
    }
    function carrier(entry:GripObject):Null<String> {
      var name = StringTools.startsWith(entry.id, "project:") ? entry.id.substr(8) : entry.id;
      var confirmed = heldCarriers.get(name);
      if (confirmed != null) return confirmed;
      var body = simulation.objectCarrier(entry.object);
      for (index in 0...robot.model.links.length)
        if (simulation.linkBody(robotIndex, index) == body) return robot.model.links[index].id;
      return null;
    }
    for (other in cellRobots) if (other != robot && other.parts.length > 0) {
      var index = other.parts[0].robotIndex;
      captured.mechanism(other.robot.id(), other.model, other.shapes,
        [for (link in 0...other.model.links.length) linkPose(index, link)]);
    }
    for (entry in objects) if (StringTools.startsWith(entry.id, "project:")) {
      if (captured.includes(entry.id.substr(8))) continue;
      var part = assemblyParts.get(entry.id);
      if (part == null || part.pieces.length == 0) continue;
      var pose = part.pose();
      captured.object(entry.id.substr(8), part.pieces, Transform3.fromArrays(pose.position, pose.rotation), carrier(entry),
        entry.object.motion == nativekit.sim.MotionType.Static ? CollisionOwner.Fixture : CollisionOwner.Loose);
    }
    for (entry in objects) {
      var name = StringTools.startsWith(entry.id, "project:") ? entry.id.substr(8) : entry.id;
      if (captured.includes(name) || ignored.indexOf(name) >= 0) continue;
      var vertices = entry.object.shape.convexVertices();
      if (vertices.length == 0) {
        var half = entry.object.shape.boundingHalfExtents();
        if (half == null) throw 'Joint clearance needs finite geometry for "${entry.id}"';
        vertices = collisionkit.CollisionBounds.of(collisionkit.CollisionGeometry.Box(half.x, half.y, half.z), collisionkit.CollisionPose.identity()).corners();
      }
      var pose = simulation.objectPose(entry.object);
      captured.object(name, [vertices], new Transform3(new Vec3(pose.x, pose.y, pose.z),
        new Quat(pose.qx, pose.qy, pose.qz, pose.qw)), carrier(entry),
        entry.object.motion == nativekit.sim.MotionType.Static ? CollisionOwner.Fixture : CollisionOwner.Loose);
    }
    for (hull in depositedWeldHulls()) captured.fixture(hull.name, hull.vertices, Transform3.identity());
    return captured;
  }


  function observedJoints(arm:Manipulator):Array<Float> {
    var positions = robot.robot.snapshot().positions;
    var indices = arm.jointIndices();
    clearanceJoints = indices;
    return [for (index in indices) positions.get(index)];
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
    return "Step " + (stepIndex + 1) + " / " + mission.steps.length + " · " + mission.steps[stepIndex].kind;
  }

  /** Structured presentation state; startup is separate from the first authored step. */
  public function progress():MissionProgress {
    var phase = failure != null ? "failed" : finished ? "complete" :
      !homingComplete && homing != null ? "startup" : "executing";
    return {phase:phase, stepIndex:stepIndex, completedInLoop:finished ? mission.steps.length : stepIndex,
      loop:mission.loop == true ? Std.int(completed / mission.steps.length) + 1 : 1, failure:failure};
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
    if (readCellInput != null) switch readCellInput(RobotInterfaceChannels.ALARM) {
      case Digital(true):
        var alarm = readInterfaceAlarm(readCellInput);
        var partPresent = switch readCellInput(RobotInterfaceChannels.PART_PRESENT) {
          case Digital(value): value;
          case _: false;
        };
        var alarmName = alarm.valid ? Std.string(alarm.code) : "with an invalid code";
        var missionAlarm = alarm.valid ? MissionAlarmClassifier.fromInterfaceAlarm(alarm.code, partPresent)
          : MissionAlarmCode.CncAlarm;
        failMission('CNC controller alarm $alarmName', missionAlarm);
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
    if (energyStage != NoEnergyTransfer) {
      advanceEnergyTransfer(snapshot);
      return;
    }
    if (runner.status() != Running && beginEnergyTransfer(snapshot)) return;
    switch runner.status() {
      case Running:
        case _:
        var activeStep = mission.steps[stepIndex];
        var skill:Skill;
        try {
          skill = skillFor(activeStep);
        } catch (error:Dynamic) {
          var message = Std.string(error);
          failMission('step $stepIndex (${activeStep.kind}) could not be planned: $message',
            alarmForFailure(activeStep, message));
          return;
        }
        if (runner.start(skill) != Running) {
          settle();
          return;
        }
    }
    runner.update(snapshot, timestep);
    settle();
  }

  /** Read the four discrete CNC alarm-code outputs as one validated PLC alarm value. */
  function readInterfaceAlarm(read:String->Null<EventValue>):InterfaceAlarmRead {
    var code = 0;
    var bits = RobotInterfaceChannels.alarmCodeBits();
    for (index in 0...bits.length) switch read(bits[index]) {
      case Digital(on): if (on) code |= 1 << index;
      case _: return {valid: false, code: RobotInterfaceAlarm.None};
    }
    if (code < RobotInterfaceAlarm.ConflictingRequests || code > RobotInterfaceAlarm.ToolChangeRequired)
      return {valid: false, code: RobotInterfaceAlarm.None};
    var alarm:RobotInterfaceAlarm = code;
    return {valid: true, code: alarm};
  }

  /** Reserve one complete weld and the two navigation legs needed to dock, charge and resume. */
  function beginEnergyTransfer(snapshot:RobotSnapshot):Bool {
    var energy = energyPower;
    var mobile = project.mobileBase;
    if (energy == null || mobile == null || mobile.energy == null || mission.steps[stepIndex].kind != "weld")
      return false;
    if (energyReadyStep == stepIndex) return false;
    var energyProfile:SceneArtifactMobileEnergy = cast mobile.energy;
    var state = localization.state();
    if (state == null || navigator == null) {
      failure = "Energy reserve needs a localized mobile base and dock route planner";
      return true;
    }
    var dock = energyProfile.dockPose;
    var dockPose = new Pose2(dock.x, dock.y, dock.yaw);
    var resumePose = energyResumePose(state.pose);
    var outbound:Null<robotkit.navigation.Path> = null, inbound:Null<robotkit.navigation.Path> = null;
    try {
      outbound = navigator.planner.plan(state.pose, dockPose);
    } catch (error:Dynamic) {
      failure = 'Energy reserve route planning to dock (${poseText(dockPose)}) failed: ${Std.string(error)}';
      return true;
    }
    try {
      inbound = navigator.planner.plan(dockPose, resumePose);
    } catch (error:Dynamic) {
      failure = 'Energy reserve return route to station (${poseText(resumePose)}) failed: ${Std.string(error)}';
      return true;
    }
    if (outbound == null || inbound == null) {
      failure = "Energy reserve cannot plan a route between the seam station and charging dock";
      return true;
    }
    var outboundWh = routeEnergyWh(outbound.length, state.pose.yaw, dockPose.yaw, mobile);
    var inboundWh = routeEnergyWh(inbound.length, dockPose.yaw, resumePose.yaw, mobile);
    var weldWh:Float;
    try weldWh = weldEnergyWh(cast mission.steps[stepIndex].weld, energyProfile)
    catch (error:Dynamic) {
      failure = 'Energy reserve could not estimate seam $stepIndex: ${Std.string(error)}';
      return true;
    }
    var reserveWh = Math.max(5.0, energyProfile.capacityWh * 0.03);
    var voltageSupportWh = minimumVoltageSupportWh(cast mission.steps[stepIndex].weld, energyProfile);
    energyTargetWattHours = inboundWh + weldWh + outboundWh + reserveWh + voltageSupportWh;
    var beforeDockWh = outboundWh + energyTargetWattHours;
    var remaining = energy.batteryState();
    if (remaining == null) {
      failure = "Energy reserve has no battery observation";
      return true;
    }
    var remainingWh = remaining.remainingEnergyWattHours == null ? 0.0 :
      (cast remaining.remainingEnergyWattHours:Float);
    if (energyTargetWattHours > energyProfile.capacityWh) {
      failure = 'Seam ${stepIndex} requires ${roundWh(energyTargetWattHours)} Wh after docking, beyond pack capacity';
      return true;
    }
    if (remainingWh >= beforeDockWh) return false;
    if (remainingWh <= outboundWh + reserveWh) {
      failure = 'Pack has ${roundWh(remainingWh)} Wh and cannot reserve ${roundWh(outboundWh + reserveWh)} Wh to reach the dock';
      return true;
    }

    energyReturnPose = new Pose2(resumePose.x, resumePose.y, resumePose.yaw);
    energyDockTrips++;
    energyPower.requestCharging(false);
    energyStage = StowingForDock;
    energyRunner = new SkillRunner();
    energyRunner.start(stow());
    return true;
  }

  /** Advance the internal dock/charge/return transfer without consuming the mission's weld step. */
  function advanceEnergyTransfer(snapshot:RobotSnapshot):Bool {
    if (energyStage == NoEnergyTransfer) return false;
    var power = energyPower;
    if (power == null) {
      failure = "Energy transfer lost its battery model";
      energyStage = NoEnergyTransfer;
      return true;
    }
    if (energyStage == StowingForDock) {
      var stowing = energyRunner;
      if (stowing == null) {
        failure = "Energy transfer lost the clearance-checked torch stow";
        energyStage = NoEnergyTransfer;
        return true;
      }
      if (stowing.status() == Running) stowing.update(snapshot, timestep);
      switch stowing.status() {
        case Running: return true;
        case Succeeded:
          energyRunner = null;
          var mobile = project.mobileBase;
          var energy:SceneArtifactMobileEnergy = mobile == null || mobile.energy == null ? null : cast mobile.energy;
          if (energy == null) {
            failure = "Energy transfer lost the CAD charging dock profile after stow";
            energyStage = NoEnergyTransfer;
            return true;
          }
          var dock = energy.dockPose;
          energyStage = ToChargingDock;
          startEnergyNavigation(new Pose2(dock.x, dock.y, dock.yaw));
        case Failed(message):
          failure = 'Energy reserve could not clearance-check the torch stow: $message';
          energyRunner = null;
          energyStage = NoEnergyTransfer;
        case Cancelled:
          failure = "Energy reserve torch stow was cancelled";
          energyRunner = null;
          energyStage = NoEnergyTransfer;
        case Idle:
          failure = "Energy reserve torch stow returned to idle";
          energyRunner = null;
          energyStage = NoEnergyTransfer;
      }
      return true;
    }
    if (energyStage == SettlingAtDock || energyStage == SettlingAtStation) {
      if (snapshot.sourceTimestampNs <= energyHandoffAt) return true;
      if (snapshot.sourceTimestampNs - energyHandoffAt > Int64.fromFloat(5e9)) {
        failure = energyStage == SettlingAtDock
          ? "Mobile drive did not settle at the CAD charging dock"
          : "Mobile drive did not settle at the welding station after charging";
        energyStage = NoEnergyTransfer;
        return true;
      }
      if (!energyDriveStopped(snapshot)) return true;
      energyHandoffAt = Int64.ofInt(0);
      if (energyStage == SettlingAtDock) {
        if (!power.atDock()) {
          failure = "Mobile drive left the CAD charging dock while settling";
          energyStage = NoEnergyTransfer;
          return true;
        }
        var mobile = project.mobileBase;
        if (mobile == null || mobile.energy == null) {
          failure = "CAD charging dock lost its mobile energy profile";
          energyStage = NoEnergyTransfer;
          return true;
        }
        var state = power.batteryState();
        energyChargeStartWattHours = state == null || state.remainingEnergyWattHours == null
          ? 0.0 : cast state.remainingEnergyWattHours;
        energyStage = ChargingAtDock;
        power.requestCharging(true);
      } else finishPostDockReturn(snapshot);
      return true;
    }
    if (energyStage == ChargingAtDock) {
      if (!power.atDock()) {
        power.requestCharging(false);
        failure = "Charging stopped because the robot left its CAD dock pose";
        energyStage = NoEnergyTransfer;
        return true;
      }
      power.requestCharging(true);
      energyChargeSeconds += timestep;
      var state = power.batteryState();
      var remaining:Float = state == null || state.remainingEnergyWattHours == null
        ? 0.0 : cast state.remainingEnergyWattHours;
      if (remaining + 1e-6 >= energyTargetWattHours) {
        energyChargedWattHours += Math.max(0.0, remaining - energyChargeStartWattHours);
        power.requestCharging(false);
        energyStage = ReturningFromDock;
        var pose = energyReturnPose;
        if (pose == null) {
          failure = "Energy transfer lost the saved seam station pose";
          energyStage = NoEnergyTransfer;
          return true;
        }
        startEnergyNavigation(pose);
      }
      return true;
    }

    var active = energyRunner;
    if (active == null) {
      failure = "Energy transfer has no navigation runner";
      energyStage = NoEnergyTransfer;
      return true;
    }
    if (active.status() == Running) active.update(snapshot, timestep);
    if (active.status() == Running) return true;
    switch active.status() {
      case Running: return true;
      case Succeeded:
        if (energyStage == ToChargingDock) {
          if (!power.atDock()) {
            failure = "Navigation reported success outside the CAD charging dock tolerance";
            energyStage = NoEnergyTransfer;
            return true;
          }
          energyStage = SettlingAtDock;
          energyHandoffAt = snapshot.sourceTimestampNs;
        } else if (energyStage == ReturningFromDock) {
          power.requestCharging(false);
          energyStage = SettlingAtStation;
          energyHandoffAt = snapshot.sourceTimestampNs;
        }
      case Failed(message):
        power.requestCharging(false);
        failure = 'Energy dock navigation failed: $message';
        energyStage = NoEnergyTransfer;
      case Cancelled:
        power.requestCharging(false);
        failure = "Energy dock navigation was cancelled";
        energyStage = NoEnergyTransfer;
      case Idle:
        power.requestCharging(false);
        failure = "Energy dock navigation returned to idle";
        energyStage = NoEnergyTransfer;
    }
    return true;
  }

  function energyDriveStopped(snapshot:RobotSnapshot):Bool {
    var mobile = project.mobileBase;
    if (mobile == null) return false;
    var names = robot.robot.description().joints;
    var left = names.indexOf(mobile.leftWheel), right = names.indexOf(mobile.rightWheel);
    return left >= 0 && right >= 0 && Math.abs(snapshot.velocities.get(left)) <= 1e-5 &&
      Math.abs(snapshot.velocities.get(right)) <= 1e-5;
  }

  function finishPostDockReturn(snapshot:RobotSnapshot):Void {
    var job = energyContactWork(stepIndex);
    var odometry = wheels;
    var localized = localization.state();
    if (odometry == null || localized == null || localized.referenceFrame != FRAME ||
        localized.bodyFrame != robot.rootLink || localized.sequence != snapshot.sourceSequence) {
      failure = "Energy return needs a fresh map pose before resuming the saved work frame";
      energyStage = NoEnergyTransfer;
      return;
    }
    if (switch localized.quality { case robotkit.localization.LocalizationQuality.Invalid: true; case _: false; }) {
      failure = "Energy return localization is invalid";
      energyStage = NoEnergyTransfer;
      return;
    }
    if (job != null && !registeredWorkWorld.exists(job.frame)) {
      failure = "Energy return lost the map-anchored contact registration for the clamped workpiece";
      energyStage = NoEnergyTransfer;
      return;
    }
    if (job != null) {
      var covariance = localized.covariance;
      var planarTrace = covariance.xx + covariance.yy;
      var planarSpread = Math.sqrt(Math.max(0.0,
        (covariance.xx - covariance.yy) * (covariance.xx - covariance.yy) + 4.0 * covariance.xy * covariance.xy));
      var positionThreeSigma = 3.0 * Math.sqrt(Math.max(0.0, 0.5 * (planarTrace + planarSpread)));
      var headingThreeSigma = 3.0 * Math.sqrt(Math.max(0.0, covariance.yawYaw));
      var positionEnvelope = job.translation.length < 2 ? 0.0 : Math.min(job.translation[0], job.translation[1]);
      var headingEnvelope = job.rotation.length < 3 ? 0.0 : job.rotation[2];
      if (positionThreeSigma > positionEnvelope || headingThreeSigma > headingEnvelope) {
        failure = 'Energy return localization uncertainty exceeds the saved contact envelope: ' +
          '3σ position=${positionThreeSigma} m / ${positionEnvelope} m, ' +
          'heading=${headingThreeSigma} rad / ${headingEnvelope} rad';
        energyStage = NoEnergyTransfer;
        return;
      }
    }
    // The clamped workpiece stays fixed in the map while the robot charges. Dock
    // alignment is only an energy connection; fresh map localization re-anchors
    // wheel odometry, while the accepted contact frame remains map-anchored.
    odometry.rebase(localized, snapshot);
    energyReadyStep = stepIndex;
    finishEnergyTransfer();
  }

  function energyContactWork(weldStep:Int):Null<SceneContactWork> {
    if (weldStep < 0 || weldStep >= mission.steps.length || mission.steps[weldStep].kind != "weld") return null;
    var weld:SceneArtifactWeld = cast mission.steps[weldStep].weld;
    var index = weldStep - 1;
    while (index >= 0) {
      var step = mission.steps[index];
      if (step.kind == "findWork" && step.contactWork != null) {
        var job:SceneContactWork = cast step.contactWork;
        if (job.frame == weld.frame) return job;
      }
      index--;
    }
    return null;
  }

  function finishEnergyTransfer():Void {
    energyStage = NoEnergyTransfer;
    energyRunner = null;
    energyReturnPose = null;
    energyTargetWattHours = 0.0;
    energyHandoffAt = Int64.ofInt(0);
  }

  function startEnergyNavigation(pose:Pose2):Void {
    var activeNavigator = navigator;
    if (activeNavigator == null) throw "Energy docking requires a Navigator";
    var mobile = project.mobileBase;
    var energy:SceneArtifactMobileEnergy = mobile == null || mobile.energy == null ? null : cast mobile.energy;
    if (energy == null) throw "Energy docking requires the CAD dock alignment profile";
    var atDock = energyStage == ToChargingDock;
    var tolerances = atDock
      ? [energy.dockPositionToleranceMeters, energy.dockHeadingToleranceRadians]
      : contactNavigationTolerances(energyContactWork(stepIndex));
    energyRunner = new SkillRunner();
    energyRunner.start(new GoTo(activeNavigator,
      new NavigationGoal(pose, FRAME, tolerances[0], tolerances[1]), observe));
  }

  /**
   * A station approach precedes touch registration. Spend at most half of its
   * declared translation/heading envelope on base parking error so probing
   * retains clearance and reach margin for the actual part displacement.
   */
  function contactNavigationTolerances(contact:Null<SceneContactWork>):Array<Float> {
    var position = POSITION_TOLERANCE, heading = HEADING_TOLERANCE;
    if (contact != null) {
      if (contact.translation.length >= 2) {
        var planar = Math.min(contact.translation[0], contact.translation[1]);
        if (Math.isFinite(planar) && planar > 0.0)
          position = Math.min(position, planar * 0.5);
      }
      if (contact.rotation.length >= 3 && Math.isFinite(contact.rotation[2]) && contact.rotation[2] > 0.0)
        heading = Math.min(heading, contact.rotation[2] * 0.5);
    }
    return [position, heading];
  }

  /** Return to the authored station goal; measured arrival can sit in a blocked neighboring costmap cell. */
  function energyResumePose(fallback:Pose2):Pose2 {
    var index = stepIndex - 1;
    while (index >= 0) {
      var step = mission.steps[index];
      if (step.kind == "goTo") return floorPose(step);
      index--;
    }
    return new Pose2(fallback.x, fallback.y, fallback.yaw);
  }

  static function poseText(pose:Pose2):String return '(${pose.x},${pose.y},${pose.yaw})';

  /** Conservative energy for a full CAD seam, its passes, pauses and arm load envelope. */
  function weldEnergyWh(weld:SceneArtifactWeld, energy:materia.project.SceneArtifact.SceneArtifactMobileEnergy):Float {
    var tools = project.robotTools;
    var torches = [for (tool in tools) if (tool.kind == "torch") tool];
    if (torches.length != 1 || torches[0].torch == null) throw "Energy reservation needs the CAD torch process profile";
    var torch:SceneArtifactTorch = cast torches[0].torch;
    var arc = new WeldArcModel({maxCurrentA: torch.maxCurrentA, efficiency: torch.efficiency,
      wireDiameterMm: torch.wireDiameterMm, stickoutMm: torch.stickoutMm});
    var length = 0.0;
    for (segment in weld.path) {
      var a = segment.start.position, b = segment.stop.position;
      length += Math.sqrt(Math.pow(b[0] - a[0], 2) + Math.pow(b[1] - a[1], 2) + Math.pow(b[2] - a[2], 2));
    }
    var maxArmMechanicalWatts = maximumJointPower(false, project.mobileBase);
    var armBusWatts = maxArmMechanicalWatts / (energy.armDriveEfficiency * energy.armSupplyEfficiency);
    var computerWatts = energy.computerLoadWatts / energy.computerSupplyEfficiency;
    var wattHours = 0.0;
    for (pass in weld.passes) {
      var weaveFactor = pass.weave == null ? 1.0 : 1.0 + 2.0 * pass.weave.amplitude * pass.weave.cyclesPerMetre;
      var seconds = length * weaveFactor / pass.process.travelSpeed + pass.process.startDwell +
        pass.process.craterDwell + pass.process.burnback + WeldArcModel.IGNITION_DELAY;
      var current = arc.currentFor(pass.process.wireSpeed);
      var volts = WeldArcModel.arcVoltage(pass.process.voltage, WeldArcModel.NOMINAL_ARC_LENGTH);
      var mainsWatts = current * volts / torch.efficiency;
      var dcWatts = mainsWatts / energy.inverterEfficiency;
      wattHours += (dcWatts + armBusWatts + computerWatts) * seconds / 3600.0;
      wattHours += computerWatts * pass.interpassDwell / 3600.0;
    }
    return wattHours * 1.25;
  }

  /** Keep enough open-circuit voltage after the seam and return reserve to survive its peak load. */
  function minimumVoltageSupportWh(weld:SceneArtifactWeld,
      energy:SceneArtifactMobileEnergy):Float {
    var torch:SceneArtifactTorch = null;
    for (tool in project.robotTools) if (tool.kind == "torch" && tool.torch != null) torch = cast tool.torch;
    if (torch == null) throw "Voltage reserve needs the CAD torch process profile";
    var arc = new WeldArcModel({maxCurrentA: torch.maxCurrentA, efficiency: torch.efficiency,
      wireDiameterMm: torch.wireDiameterMm, stickoutMm: torch.stickoutMm});
    var peakPackWatts = 0.0;
    var armWatts = maximumJointPower(false, project.mobileBase) /
      (energy.armDriveEfficiency * energy.armSupplyEfficiency);
    var computerWatts = energy.computerLoadWatts / energy.computerSupplyEfficiency;
    for (pass in weld.passes) {
      var arcWatts = arc.currentFor(pass.process.wireSpeed) *
        WeldArcModel.arcVoltage(pass.process.voltage, WeldArcModel.NOMINAL_ARC_LENGTH) /
        (torch.efficiency * energy.inverterEfficiency);
      peakPackWatts = Math.max(peakPackWatts, arcWatts + armWatts + computerWatts);
    }
    var requiredOpenCircuit = energy.cutoffVoltage +
      peakPackWatts * energy.internalResistanceOhms / energy.cutoffVoltage;
    if (requiredOpenCircuit >= energy.fullVoltage) return energy.capacityWh + 1.0;
    var fraction = Math.max(0.0, (requiredOpenCircuit - energy.emptyVoltage) /
      (energy.fullVoltage - energy.emptyVoltage));
    return fraction * energy.capacityWh;
  }

  /** Rated joint power comes from the CAD-mapped runtime drive limits. */
  function maximumJointPower(wheelsOnly:Bool, mobile:Null<SceneArtifactMobileBase>):Float {
    var names = robot.robot.description().joints;
    var power = 0.0;
    for (index in 0...names.length) {
      var wheel = mobile != null && (names[index] == mobile.leftWheel || names[index] == mobile.rightWheel);
      if (wheel != wheelsOnly) continue;
      var joint = robot.blueprint.joints[index];
      if (joint.maxEffort != null && joint.maxRate != null)
        power += (cast joint.maxEffort:Float) * (cast joint.maxRate:Float);
    }
    return power;
  }

  function routeEnergyWh(distance:Float, startYaw:Float, endYaw:Float,
      mobile:SceneArtifactMobileBase):Float {
    var energy:SceneArtifactMobileEnergy = cast mobile.energy;
    if (energy == null) return 0.0;
    var driveMechanicalWatts = maximumJointPower(true, mobile);
    var driveBusWatts = driveMechanicalWatts / (energy.driveMotorEfficiency * energy.driveGearEfficiency *
      energy.driveSupplyEfficiency);
    var computerWatts = energy.computerLoadWatts / energy.computerSupplyEfficiency;
    var turn = Math.atan2(Math.sin(endYaw - startYaw), Math.cos(endYaw - startYaw));
    var travelSeconds = distance / mobile.maxLinearSpeed * 1.8 + Math.abs(turn) / mobile.maxAngularSpeed * 1.8 + 2.0;
    return (driveBusWatts + computerWatts) * travelSeconds / 3600.0 * 1.25;
  }

  static function roundWh(value:Float):Float return Math.round(value * 10.0) / 10.0;

  /** The step that was running stops through the robot's runtime, which still answers until the session resets. */
  public function beforeReset():Void {
    runner.cancel();
    if (energyRunner != null) energyRunner.cancel();
    var power = energyPower;
    if (power != null) power.requestCharging(false);
    deenergizeMissionOutputs();
    if (homing != null && homing.isMoving()) homing.abort();
    clearJointMotions();
  }

  /** Back to the first step; the next tick starts it from wherever the reset put the robot. */
  public function reset():Void {
    deenergizeMissionOutputs();
    runner = new SkillRunner();
    clearJointMotions();
    registeredWorkWorld = new Map<String, Transform3>();
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
    energyStage = NoEnergyTransfer;
    energyRunner = null;
    energyReturnPose = null;
    energyHandoffAt = Int64.ofInt(0);
    energyTargetWattHours = 0.0;
    energyChargeStartWattHours = 0.0;
    energyReadyStep = -1;
    energyDockTrips = 0;
    energyChargedWattHours = 0.0;
    energyChargeSeconds = 0.0;
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

  /** Controller pulses are timed in feed(), never on render-frame presentation. */
  public function present():Void {}

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
    return MissionAlarmClassifier.classify(step, message, emergencyStopped);
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
    // CellSignals samples cabinet outputs before this member runs. Keep reset high
    // until the following controller image has carried it to the CNC cabinet.
    if (recoveryPulsePending) {
      recoveryPulsePending = false;
      try writeMissionOutput(cnckit.controller.RobotInterfaceChannels.FAULT_RESET, false) catch (_:Dynamic) {}
    }
    var state = robot.runtime.snapshot();
    var safetyStopped = state.safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT ||
      state.safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP ||
      state.endpoint == RobotKitRuntimeConstants.RK_ENDPOINT_FAULT;
    var safetyReady = !safetyStopped && state.safety == RobotKitRuntimeConstants.RK_SAFETY_READY;
    if (safetyReady) for (channel in carryingTools.keys())
      if (carryingTools.get(channel) == true && gripperTools.exists(channel))
        robot.runtime.writeDigitalOutput(channel, true);
    if (recoveryOutputPending && safetyReady) {
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
    if (!activeAlarm && safetyReady && !recoveryPulsePending) {
      // An operator may have reseated a dropped part. Keep its identity but
      // derive the placement and payload frame from the confirmed live grasp.
      var graspsConfirmed = true;
      var observed = robot.robot.snapshot();
      for (channel in carryingTools.keys()) if (carryingTools.get(channel) == true && gripperTools.exists(channel)) {
        var tool = handlingTools.get(channel);
        if (tool == null || !toolSensorActive(observed, tool)) graspsConfirmed = false;
      }
      if (graspsConfirmed) {
        for (channel in carryingTools.keys())
          if (carryingTools.get(channel) == true && gripperTools.exists(channel)) captureHeldGripperTransform(channel);
        failure = null;
        alarmCode = null;
        failedStepIndex = -1;
        stepIndex = recoveryStep;
        recoveryStep = -1;
        recoveryPending = false;
        recoveryOutputPending = false;
        runner = new SkillRunner();
        clearJointMotions();
        return;
      }
    }
    if (recoveryElapsed >= 5.0) {
      recoveryPending = false;
      recoveryOutputPending = false;
      recoveryStep = -1;
      failure = activeAlarm ? "Fault reset did not clear the machine alarm; remove the cause and retry"
        : !safetyReady ? "The robot did not become ready for recovery; correct its safety state and retry"
        : "A carried gripper has no confirmed grasp; reseat the part and retry";
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
        clearJointMotions();
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

  /** Disposes the native collision worlds owned by cached joint-motion planners. */
  function clearJointMotions():Void {
    for (name in jointMotions.keys()) {
      var cached = jointMotions.get(name);
      if (cached == null) continue;
      if (cached.motion.shutdownPlanning() != 0)
        throw 'Joint-motion planner "$name" did not stop before collision-world teardown';
      if (cached.space != null) cached.space.dispose();
    }
    jointMotions = new Map<String, CachedJointMotion>();
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
        registeredWorkWorld = new Map<String, Transform3>();
        var activeNavigator:Navigator = cast navigator;
        var contact:Null<SceneContactWork> = null;
        if (stepIndex + 1 < mission.steps.length) {
          var next = mission.steps[stepIndex + 1];
          if (next.kind == "findWork" && next.contactWork != null) contact = cast next.contactWork;
        }
        var tolerances = contactNavigationTolerances(contact);
        return new GoTo(activeNavigator, new NavigationGoal(floorPose(step), FRAME, tolerances[0], tolerances[1]), observe);
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
        // A contact grip can slip during transit. Placement uses the currently
        // observed part-to-tool transform, frozen for this handling program.
        if (gripperTools.exists(channel)) captureHeldGripperTransform(channel);
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
        registeredWorkWorld.remove(job.frame);
        var factory = newRegistration;
        if (factory == null) throw "Contact mission has no registration runner";
        return new processkit.skill.FindWeldWork(() -> factory(job), (frame) -> acceptRegisteredWork(job, frame));
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
    return approachDistance == null ? active : active.withApproachDistance(approachDistance);
  }

  function acceptRegisteredWork(job:SceneContactWork, measured:Transform3):Void {
    var estimate = localization.state();
    var snapshot = robot.robot.snapshot();
    if (estimate == null || estimate.referenceFrame != FRAME || estimate.bodyFrame != robot.rootLink ||
        estimate.sequence != snapshot.sourceSequence)
      throw "Contact registration cannot anchor its measured work frame without a fresh valid map pose";
    if (switch estimate.quality { case robotkit.localization.LocalizationQuality.Invalid: true; case _: false; })
      throw "Contact registration cannot anchor its measured work frame without a fresh valid map pose";
    var covariance = estimate.covariance;
    var planarTrace = covariance.xx + covariance.yy;
    var planarSpread = Math.sqrt(Math.max(0.0,
      (covariance.xx - covariance.yy) * (covariance.xx - covariance.yy) + 4.0 * covariance.xy * covariance.xy));
    var positionThreeSigma = 3.0 * Math.sqrt(Math.max(0.0, 0.5 * (planarTrace + planarSpread)));
    var headingThreeSigma = 3.0 * Math.sqrt(Math.max(0.0, covariance.yawYaw));
    var positionEnvelope = job.translation.length < 2 ? 0.0 : Math.min(job.translation[0], job.translation[1]);
    var headingEnvelope = job.rotation.length < 3 ? 0.0 : job.rotation[2];
    if (positionThreeSigma > positionEnvelope || headingThreeSigma > headingEnvelope)
      throw 'Contact registration map uncertainty exceeds the saved contact envelope: ' +
        '3σ position=${positionThreeSigma} m / ${positionEnvelope} m, ' +
        'heading=${headingThreeSigma} rad / ${headingEnvelope} rad';
    registeredWorkWorld.set(job.frame, Transform3.fromPose2(estimate.pose).compose(measured));
  }

  /** Plans absolute mechanical coordinates on the chain ending at the named connector. */
  function jointMove(step:SceneArtifactMissionStep):Skill {
    var at = step.at;
    if (at == null) throw "Joint motion needs a mission frame";
    var context = jointMotionContext(at);
    var q = startingJointPositions(context.motion, context.indices);
    setJointTargets(q, step.joints, context);
    return new MotionProgramSkill(context.motion, new MotionProgram([
      MotionOp.MoveJ(MoveTarget.JointTarget(q), new MotionOptions(0, 0, 0, true), Blend.ExactStop)]));
  }

  /** A sequence of exact-stop Cartesian or joint points certified as timed arm motion. */
  function viaMove(step:SceneArtifactMissionStep):Skill {
    var at = step.at;
    var waypoints = step.poses;
    if (at == null || waypoints == null || waypoints.length == 0) throw "Via motion needs a frame and waypoints";
    var context = jointMotionContext(at);
    var q = startingJointPositions(context.motion, context.indices);
    var ops:Array<MotionOp> = [];
    for (waypointIndex in 0...waypoints.length) {
      var waypoint = waypoints[waypointIndex];
      if (waypoint.joints != null) {
        var goal = q.copy();
        setJointTargets(goal, waypoint.joints, context);
        ops.push(MotionOp.MoveJ(MoveTarget.JointTarget(goal), new MotionOptions(0, 0, 0, true), Blend.ExactStop));
        q = goal;
      } else {
        var pose = waypoint.pose;
        if (pose == null) throw "Via waypoint has no target";
        ops.push(MotionOp.MoveJ(MoveTarget.PoseTarget(Pose3.fromArrays(pose.position, pose.rotation), FRAME, null),
          new MotionOptions(), Blend.ExactStop));
      }
    }
    return new MotionProgramSkill(context.motion, new MotionProgram(ops));
  }

  function jointMotionContext(at:SceneArtifactPlace):JointMotionContext {
    var name = AssemblyRobot.missionFrameName(at);
    var frame = robot.missionFrames.get(name);
    if (frame == null) throw 'The robot has no mission frame "$name"';
    var arm = new Manipulator(robot.model, robot.model.links[0].id, frame.id);
    var count = arm.group.count();
    var indices = [for (target in arm.toJointTargets([for (_ in 0...count) 0.0])) target.joint];
    var revision = robot.runtime.snapshot();
    var cached = jointMotions.get(name);
    if (cached != null && (Int64.compare(cached.motion.compiler.limits.modelRevision, revision.modelRevision) != 0 ||
        Int64.compare(cached.motion.compiler.limits.calibrationRevision, revision.calibrationRevision) != 0)) {
      if (cached.motion.shutdownPlanning() != 0)
        throw 'Joint-motion planner "$name" did not stop before updating its reference';
      cached.motion.reset();
      if (cached.space != null) cached.space.dispose();
      jointMotions.remove(name);
      cached = null;
    }
    var motion = cached == null ? null : cached.motion;
    if (motion == null) {
      var limits = new ValidationLimits(count, revision.modelRevision, revision.calibrationRevision);
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
      // moveVia compiles planned MoveJ operations. The CL7 collisionkit motion
      // planner certifies each joint-space edge and the timed rest-to-rest
      // trajectory follows those same edges; do not re-check it with the
      // process-clearance world, whose adjacent-link policy is different.
      var compiler = new ProgramCompiler(new ManipulatorKinematics(arm, 1e-8), limits, FRAME,
        velocities, accelerations, jerks,
        StartTolerances.uniform(count, 0.005, 0.02, 0.2));
      motion = new ManipulatorMotion(robot.robot, compiler,
        channel -> cellInput == null ? null : cellInput(channel), () -> robot.runtime.pollEvents(), indices);
      cached = {motion: motion, space: null};
      jointMotions.set(name, cached);
    }
    if (cached == null) throw "Joint-motion cache was not created";
    if (motion.shutdownPlanning() != 0)
      throw 'Joint-motion planner "$name" did not stop before replacing its collision world';
    if (cached.space != null) cached.space.dispose();
    cached.space = null;
    motion.reset();
    var reference = startingJointPositions(motion, indices);
    // The controller and its limits are reusable; the door, loose inventory
    // and confirmed payload attachments belong to this move's live scene.
    var planned = missionMotionPlanner(arm, reference, cellCollisionSnapshot([]));
    cached.space = planned.space;
    motion.compiler.motionPlanner = planned.planner;
    return {motion: motion, arm: arm, indices: indices};
  }

  function startingJointPositions(motion:ManipulatorMotion, indices:Array<Int>):Array<Float> {
    var q = motion.commandedPositions();
    if (q == null) {
      // MotionProgramSkill compiles its first move from commanded setpoints.
      // The clearance partition must use that same start state; measured
      // positions can lag by a small tracking error and describe a different path.
      var positions = robot.robot.snapshot().setpointPositions;
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
        var requested = target.position - placement.joint(target.joint) * scale;
        var limits = context.arm.group.limitsOf(i);
        if (limits.lower < limits.upper) {
          // Converting an authored absolute coordinate through its assembly
          // offset can put an exact stop target one ULP outside the model range.
          // Snap only that floating-point residue; reject any real overtravel.
          var tolerance = 1e-12 * Math.max(1.0,
            Math.max(Math.abs(limits.lower), Math.abs(limits.upper)));
          if (requested < limits.lower) {
            if (limits.lower - requested > tolerance)
              throw 'Joint "${target.joint}" target $requested is below its limit ${limits.lower}';
            requested = limits.lower;
          } else if (requested > limits.upper) {
            if (requested - limits.upper > tolerance)
              throw 'Joint "${target.joint}" target $requested is above its limit ${limits.upper}';
            requested = limits.upper;
          }
        }
        q[i] = requested;
        found = true;
      }
      if (!found) throw 'Joint "${target.joint}" is outside the selected mission chain';
    }
  }

  function missionMotionPlanner(arm:Manipulator, reference:Array<Float>,
      snapshot:CellCollisionSnapshot):{planner:MotionPlanner, space:ManipulatorPlannerSpace} {
    var scene = snapshot.describe(arm, reference, CollisionClearance.MARGIN);
    var members = weldingMembers();
    if (members.length > 0)
      processkit.WeldCollisionPolicy.apply(scene, scene.memberObjects(members), processkit.WeldPathPlanner.AIR_MARGIN, CollisionClearance.CONTACT_MARGIN);
    for (channel in carryingTools.keys()) if (carryingTools.get(channel) == true) {
      var held = heldParts.get(channel), tool = handlingTools.get(channel);
      if (held == null || tool == null) throw "Confirmed payload lost its named tool or workpiece";
      var holder = tool.contact.occurrence;
      var gripper = tool.gripper;
      if (tool.kind == "gripper" && gripper == null) throw "Held stock lost its typed gripper contact members";
      var pads = scene.contacts(gripper == null ? [holder] : gripper.padMembers);
      for (name in scene.contacts([held.occurrence]))
        scene.payload(name, pads, CollisionPairRule.Allow, CollisionPairStatus.ProcessContact, 0.0);
    }
    var build = scene.build(new NativeCollisionWorld());
    var collisionSpace = new CollisionPlannerSpace(scene.description, build, scene.bodies, scene.margins());
    var space = new ManipulatorPlannerSpace(collisionSpace, arm, reference, scene.bodies.reference.q);
    var planner = new RrtConnect(space, 17);
    return {planner: planner, space: space};
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
      return new WeldSeam(active, localization, () -> weldPlan(weld, pass), sensor);
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
      var measured = registeredWorkWorld.get(weld.frame);
      if (measured == null) throw "Weld work has no accepted map-frame registration at this station";
      return measured;
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

  /** The current energy-transfer phase, for the mission view and bounded execution diagnostics. */
  public function energyTransferPhase():String return switch energyStage {
    case NoEnergyTransfer: "idle";
    case StowingForDock: "stowing-for-dock";
    case ToChargingDock: "to-dock";
    case SettlingAtDock: "settling-at-dock";
    case ChargingAtDock: "charging";
    case ReturningFromDock: "returning-from-dock";
    case SettlingAtStation: "settling-at-station";
  };

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
    var seat = liveConnectorFrame(at);
    if (gripperTools.exists(channel)) {
      var offset = heldToolOffsets.get(channel);
      if (offset == null) throw "The gripper has no measured tool-to-part transform";
      var rotation = placementRotation(at.occurrence, yaw);
      var shifted = rotation.rotate(offset);
      return new Vec3(seat.x + shifted.x, seat.y + shifted.y, seat.z + shifted.z);
    }
    var origin = partFrame(held.occurrence);
    var tool = toolContact(channel);
    var offset = new Vec3(tool.x - origin.x, tool.y - origin.y, tool.z - origin.z);
    if (yaw != null) offset = yawRotation(yaw - frameYaw(origin)).rotate(offset);
    return new Vec3(seat.x + offset.x, seat.y + offset.y, seat.z + offset.z);
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
          : yawRotation(yaw - frameYaw(partFrame(at.occurrence))).multiply(worldHome);
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
    var pose = partFrame(occurrence);
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

  /** A CAD occurrence's live origin in metres, resolved across controller owners and free bodies. */
  function partFrame(occurrence:String):AssemblyFrame {
    var pose = assemblyParts.get("project:" + occurrence).pose();
    return {x: pose.position[0], y: pose.position[1], z: pose.position[2],
      qx: pose.rotation[0], qy: pose.rotation[1], qz: pose.rotation[2], qw: pose.rotation[3]};
  }

  /** The named station follows its actual carrier; only connector translations need CAD-unit conversion. */
  function liveConnectorFrame(at:SceneArtifactPlace):AssemblyFrame {
    var connector = connectorFrame(at);
    return AssemblyFrames.compose(partFrame(at.occurrence), {x: connector.x * metres,
      y: connector.y * metres, z: connector.z * metres,
      qx: connector.qx, qy: connector.qy, qz: connector.qz, qw: connector.qw});
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
