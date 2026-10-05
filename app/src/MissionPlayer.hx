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
import materia.project.SceneArtifact.SceneArtifactWeld;
import motionkit.robot.HandlingPlanRunner;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.ManipulatorKinematics;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.StartTolerances;
import motionkit.robot.MotionProgramSkill;
import motionkit.trajectory.ValidationLimits;
import motionkit.program.MotionProgram;
import motionkit.program.MotionOp;
import motionkit.program.MoveTarget;
import motionkit.program.Blend;
import motionkit.MotionOptions;
import motionkit.robot.MotionSystem;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.robot.StepperSlip;
import robotkit.runtime.EncoderMonitor;
import motionkit.robot.PlanningLimits;
import processkit.WeldingPlanRunner;
import processkit.skill.WeldPlan;
import processkit.skill.WeldSeam;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.ArmClearance.ClearanceViolation;
import robotkit.manipulation.Manipulator;
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
import robotkit.skill.SkillRunner;
import robotkit.skill.SkillStatus;
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

/**
 * The project's mission, run by its assembly robot when the simulation runs: each step becomes a
 * RobotKit skill, advanced one tick at a time by a `SkillRunner`, and the next starts when it
 * succeeds; the round starts over after its last step when it loops, and from its first step when
 * the session resets. `goTo` is RobotKit's `GoTo`: `Navigator` plans on a map drawn from the
 * robot's surroundings (the boxes it collides with as held objects, so nothing is drawn by hand)
 * and follows the route with pure pursuit, the robot's place coming from the simulation. `pick` and
 * `place` are RobotKit's `HandlePart` with the robot's suction tool: the arm runs a MotionKit program
 * down onto the part's grasp connector, or onto its seat with the held part, read where they are when
 * the step starts; the program switches the tool's channel on the robot, the simulated tool holds or
 * lets go, and the skill reads the outcome on the tool's vacuum sensor. `weld` is RobotKit's `WeldSeam` with the robot's
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
  /** Every step is done and the mission does not loop: nothing more to run until a reset. */
  public var finished(default, null):Bool = false;

  final robot:AssemblyRobot;
  var homing:Null<MotionSystem> = null;
  var homingStarted:Bool = false;
  var homingComplete:Bool = false;
  public var homingSeconds(default, null):Float = 0.0;
  var homingEncoders:Null<EncoderMonitor> = null;
  var homingSlip:Null<StepperSlip> = null;
  var runner = new SkillRunner();
  var jointMotions = new Map<String, ManipulatorMotion>();
  /** The arm and its suction tool's vacuum sensor, when the mission picks and places. */
  var handling:Null<HandlingPlanRunner>;
  /** Makes the arm's handling runner afresh: a reset starts the robot's runtime over, plans and all. */
  final newHandling:Null<Void->HandlingPlanRunner>;
  /** The arm and its torch, when the mission welds; and the torch's weld sensor. */
  var welding:Null<WeldingPlanRunner>;
  final newWelding:Null<Void->WeldingPlanRunner>;
  final weldSensor:Null<String>;
  /** What welds are planned clear of, and where the arm's joints are among the robot's. */
  var clearance:Null<ArmClearance> = null;
  var clearanceJoints:Array<Int> = [];
  /** Runtime weld metal in world coordinates, read before each pass is planned. */
  public var depositedWeldHulls:Void -> Array<{name:String, vertices:Array<Float>}> = () -> [];
  /** Where the tool's contact rides: its link and its frame there, in metres. */
  var toolLink:Int = -1;
  var toolTip:Null<AssemblyFrame> = null;
  final robotIndex:Int;
  final vacuumSensor:Null<String>;
  final simulation:Simulation;
  final objects:Array<GripObject>;
  final project:ProjectDocumentSession;
  final assembly:Null<AssemblyDefinition>;
  final placement:Null<AssemblyState>;
  final metres:Float;
  /** The grasp connector of the part the last pick took hold of. */
  var grasped:Null<SceneArtifactPlace> = null;
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
  final idle = new PerceptionSnapshot();

  public function new(mission:SceneArtifactMission, robot:AssemblyRobot, simulation:Simulation, robotIndex:Int,
      timestep:Float, obstacles:Array<FloorObstacle>, objects:Array<GripObject>, project:ProjectDocumentSession, weldAcceleration:Float = ARM_ACCELERATION) {
    if (!Math.isFinite(weldAcceleration) || weldAcceleration <= 0.0 || weldAcceleration > ARM_ACCELERATION)
      throw "Invalid welding planning acceleration";
    if (mission.steps.length == 0) throw "A mission needs steps";
    this.mission = mission;
    this.robot = robot;
    this.timestep = timestep;
    this.obstacles = standing(obstacles);
    this.simulation = simulation;
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
    var welds = [for (step in mission.steps) if (step.kind == "weld") step].length > 0;
    if (handles && welds) throw "A mission picks and places, or it welds: the robot works with one tool on its arm";
    var suctions = [for (tool in project.robotTools) if (tool.kind == "suction") tool];
    var torches = [for (tool in project.robotTools) if (tool.kind == "torch") tool];
    if (!handles) {
      handling = null;
      newHandling = null;
      vacuumSensor = null;
    } else {
      if (suctions.length != 1) throw 'A mission that picks needs one suction tool, the robot has ${suctions.length}';
      var tool = suctions[0];
      vacuumSensor = tool.sensor;
      var arm = toolArm(tool);
      newHandling = () -> {
        var state = robot.runtime.snapshot();
        return HandlingPlanRunner.create(robot.robot, arm, () -> robot.runtime.pollEvents(), tool.channel,
          PlanningLimits.ofGroup(arm, new SteadyLoads(), ARM_ACCELERATION, null, state.modelRevision, state.calibrationRevision));
      };
      handling = newHandling();
    }
    if (!welds) {
      welding = null;
      newWelding = null;
      weldSensor = null;
    } else {
      if (torches.length != 1) throw 'A mission that welds needs one torch, the robot has ${torches.length}';
      var tool = torches[0];
      var torch = tool.torch;
      if (torch == null || tool.sensor == null) throw "The robot's torch needs its welder and weld sensor";
      weldSensor = tool.sensor;
      var arm = toolArm(tool);
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
    }
    configureHoming();
  }

  /**
   * The clearance a weld is planned with: the collision hulls the simulation uses, each on its link, the torch's marked as
   * the tool, and the arm's pose now as the one where neighbouring links touch by design. Welding keeps 3 mm between bodies in air
   * and 0.5 mm between tool and work at the seam; the arm's installed gearheads have 4.5 mm designed clearance to the next tube. `ignored` are occurrences that
   * are no obstacle (the weld metal).
   */
  function weldClearance(arm:Manipulator, torch:String, ignored:Array<String>):ArmClearance {
    var links = robot.model.links;
    // The tool is everything the torch's assembly holds (the plate on the flange, the torch and its neck).
    var toolPrefix = torch.substr(0, torch.lastIndexOf("/") + 1);
    var bodies = [for (hull in robot.hulls) if (ignored.indexOf(hull.part) < 0)
      {name: hull.part, link: links[hull.link].id, vertices: hull.vertices, tool: StringTools.startsWith(hull.part, toolPrefix)}];
    var base = simulation.linkPose(robotIndex, 0);
    var baseInverse = new Transform3(new Vec3(base.position[0], base.position[1], base.position[2]),
      new Quat(base.rotation[0], base.rotation[1], base.rotation[2], base.rotation[3])).inverse();
    for (entry in objects) if (StringTools.startsWith(entry.id, "project:") && ignored.indexOf(entry.id.substr(8)) < 0) {
      var part = assemblyParts.get(entry.id);
      if (part.vertices.length == 0) continue;
      var live = part.pose();
      var pose = baseInverse.compose(new Transform3(new Vec3(live.position[0], live.position[1], live.position[2]),
        new Quat(live.rotation[0], live.rotation[1], live.rotation[2], live.rotation[3])));
      var vertices:Array<Float> = [];
      for (index in 0...Std.int(part.vertices.length / 3)) {
        var point = pose.transformPoint(new Vec3(part.vertices[index * 3], part.vertices[index * 3 + 1], part.vertices[index * 3 + 2]));
        vertices.push(point.x); vertices.push(point.y); vertices.push(point.z);
      }
      bodies.push({name: entry.id.substr(8), link: links[0].id, vertices: vertices, tool: false});
    }
    for (hull in depositedWeldHulls()) {
      var vertices:Array<Float> = [];
      for (index in 0...Std.int(hull.vertices.length / 3)) {
        var point = baseInverse.transformPoint(new Vec3(hull.vertices[index * 3], hull.vertices[index * 3 + 1], hull.vertices[index * 3 + 2]));
        vertices.push(point.x); vertices.push(point.y); vertices.push(point.z);
      }
      bodies.push({name: hull.name, link: links[0].id, vertices: vertices, tool: false});
    }
    var positions = robot.robot.snapshot().positions;
    var indices = [for (target in arm.toJointTargets([for (_ in 0...arm.dofCount()) 0.0])) target.joint];
    clearanceJoints = indices;
    return new ArmClearance(arm, bodies, [for (index in indices) positions.get(index)], processkit.WeldPathPlanner.AIR_MARGIN);
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
    var part = robot.part("project:" + tool.contact.occurrence);
    var tcp = robot.toolFrames.get(tool.contact.occurrence);
    if (tcp == null) throw 'The robot has no frame for tool "${tool.contact.occurrence}"';
    toolLink = part.linkIndex;
    toolTip = {x: tcp.position[0], y: tcp.position[1], z: tcp.position[2],
      qx: tcp.rotation[0], qy: tcp.rotation[1], qz: tcp.rotation[2], qw: tcp.rotation[3]};
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

  public function feed():Void {
    if (failure != null || finished) return;
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
        var make = newHandling;
        if (make != null) handling = make();
        var makeWelding = newWelding;
        if (makeWelding != null) welding = makeWelding();
      } catch (error:Dynamic) {
        failure = 'Mission homing: $error';
        return;
      }
    }
    var snapshot = robot.robot.snapshot();
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
    if (homing != null && homing.isMoving()) homing.abort();
  }

  /** Back to the first step; the next tick starts it from wherever the reset put the robot. */
  public function reset():Void {
    runner = new SkillRunner();
    jointMotions = new Map<String, ManipulatorMotion>();
    stepIndex = 0;
    completed = 0;
    failure = null;
    finished = false;
    grasped = null;
    wheelsSeeded = false;
    scanned = null;
    sensed = new PerceptionSnapshot();
    configureHoming();
    var make = newHandling;
    if (make != null) handling = make();
    var makeWelding = newWelding;
    if (makeWelding != null) welding = makeWelding();
  }

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
    return '${Math.round(active.planningSeconds * 100) / 100} s, ${plan.checked} poses, rolls ${[for (roll in plan.rolls) Math.round(roll * 180 / Math.PI)].join("/")} deg, in ${plan.entry.name}, out ${plan.exitName}';
  }

  /** How many times the arc was lost and the weld restarted, in the weld running or last run. */
  public function weldRestarts():Int {
    var active = welding;
    return active == null ? 0 : active.restarts();
  }

  public function present():Void {}

  /** After a tick: a finished step hands over to the next, a failed one stops the mission. */
  function settle():Void switch runner.status() {
    case Succeeded:
      // Different motion runners own their last commanded coordinates. A joint
      // move changes the handling station; a handling/process step invalidates
      // cached joint-move anchors before another controller takes over.
      if (mission.steps[stepIndex].kind == "moveJoints") {
        var make = newHandling;
        if (make != null) handling = make();
      } else {
        jointMotions = new Map<String, ManipulatorMotion>();
      }
      completed++;
      if (stepIndex + 1 < mission.steps.length || mission.loop == true)
        stepIndex = (stepIndex + 1) % mission.steps.length;
      else
        finished = true;
    case Failed(message):
      failure = 'step $stepIndex (${mission.steps[stepIndex].kind}): $message';
    case _:
  }

  function skillFor(step:SceneArtifactMissionStep):Skill {
    switch step.kind {
      case "moveJoints":
        return jointMove(step);
      case "goTo":
        var activeNavigator:Navigator = cast navigator;
        return new GoTo(activeNavigator, new NavigationGoal(floorPose(step), FRAME, POSITION_TOLERANCE, HEADING_TOLERANCE),
          observe);
      case "pick":
        var at:SceneArtifactPlace = cast step.at;
        grasped = at;
        return HandlePart.pick(cast handling, localization, () -> graspPoint(at), vacuumSensor, 40.0, handlingOrientation(step));
      case "place":
        var at:SceneArtifactPlace = cast step.at;
        return HandlePart.place(cast handling, localization, () -> placeContact(at, step.yaw), vacuumSensor, 40.0, handlingOrientation(step));
      case "weld":
        var weld:SceneArtifactWeld = cast step.weld;
        return new processkit.skill.WeldPasses([for (pass in weld.passes)
          () -> weldPassSkill(weld, pass)],
          [for (pass in weld.passes) pass.interpassDwell]);
      default:
        throw 'Mission step kind "${step.kind}" is not supported';
    }
  }

  /** Plans absolute mechanical coordinates on the chain ending at the named connector. */
  function jointMove(step:SceneArtifactMissionStep):Skill {
    var at:SceneArtifactPlace = cast step.at;
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
      for (i in 0...count) {
        var bound = arm.group.limitsOf(i);
        if (bound.lower < bound.upper) limits.position(i, bound.lower, bound.upper);
        var speedLimit:Float = bound.velocity == null ? 0.0 : bound.velocity;
        var speed = speedLimit > 0 ? speedLimit : 2.0;
        velocities.push(speed);
        limits.velocity(i, speed);
        var accelerationLimit:Float = bound.maxAcceleration == null ? 0.0 : bound.maxAcceleration;
        var acceleration = accelerationLimit > 0 ? Math.min(ARM_ACCELERATION, accelerationLimit) : ARM_ACCELERATION;
        accelerations.push(acceleration);
        limits.acceleration(i, acceleration);
        limits.jerk(i, 20.0);
      }
      var compiler = new ProgramCompiler(new ManipulatorKinematics(arm, 1e-8), limits, FRAME,
        velocities, accelerations, [for (_ in 0...count) 20.0],
        StartTolerances.uniform(count, 0.005, ARM_ACCELERATION * 0.01, 0.2));
      motion = new ManipulatorMotion(robot.robot, compiler, (_) -> null, () -> robot.runtime.pollEvents(), indices);
      jointMotions.set(name, motion);
    }
    // A different skill may have moved this chain since its last joint program completed.
    // Keep the compiler and plan sequence, but acquire the next start from the live robot.
    motion.reset();
    var q = motion.commandedPositions();
    if (q == null) {
      var positions = robot.robot.snapshot().positions;
      q = [for (index in indices) positions.get(index)];
    }
    for (target in step.joints) {
      var found = false;
      for (i in 0...count) {
        var joint = robot.model.joints[indices[i]];
        if (joint.name != target.joint) continue;
        if (placement == null || assembly == null) throw "Joint motion needs an assembly placement";
        var authored = [for (item in assembly.joints) if (item.id == target.joint) item];
        if (authored.length != 1) throw 'No mechanical joint "${target.joint}"';
        var scale = Std.string(authored[0].type) == "prismatic" ? metres : 1.0;
        q[i] = target.position - placement.joint(target.joint) * scale;
        found = true;
      }
      if (!found) throw 'Joint "${target.joint}" is outside the chain ending at "$name"';
    }
    return new MotionProgramSkill(motion, new MotionProgram([
      MotionOp.MoveJ(MoveTarget.JointTarget(q), new MotionOptions(), Blend.ExactStop)]));
  }

  /** Builds one weld pass using the shared welding runner. */
  function weldPassSkill(weld:SceneArtifactWeld, pass:materia.project.SceneArtifact.SceneArtifactWeldPass):Skill {
    var factory = newWelding;
    if (factory == null) throw "Weld mission has no welding runner";
    welding = factory();
    return new WeldSeam(cast welding, localization, () -> weldPlan(weld, pass), cast weldSensor);
  }

  /** The world pose of a weld's reference member now, or the world's own when the weld names none. */
  function referenceFrame(weld:SceneArtifactWeld):Transform3 {
    if (weld.frame == null || weld.frame == "") return Transform3.identity();
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
  function graspPoint(at:SceneArtifactPlace):Vec3 {
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
  function placeContact(at:SceneArtifactPlace, ?yaw:Float):Vec3 {
    var held = grasped;
    if (held == null) throw "Nothing was picked to place";
    var seat = AssemblyFrames.compose(staticPose(at.occurrence), connectorFrame(at));
    var origin = partFrame(held.occurrence);
    var tool = toolContact();
    var offset = new Vec3(tool.x - origin.x, tool.y - origin.y, tool.z - origin.z);
    if (yaw != null) offset = yawRotation(yaw - frameYaw(origin)).rotate(offset);
    return new Vec3(seat.x * metres + offset.x, seat.y * metres + offset.y, seat.z * metres + offset.z);
  }

  static function frameYaw(frame:AssemblyFrame):Float {
    var x = new Quat(frame.qx, frame.qy, frame.qz, frame.qw).rotate(new Vec3(1, 0, 0));
    return Math.atan2(x.y, x.x);
  }

  static function yawRotation(yaw:Float):Quat return Quat.fromAxisAngle(new Vec3(0, 0, 1), yaw);

  /** Fix the part's heading while preserving the tool-to-part grasp rotation. */
  function handlingOrientation(step:SceneArtifactMissionStep):Null<Void -> Quat> {
    var yaw = step.yaw;
    if (yaw == null) return null;
    return () -> {
      if (step.kind == "pick") {
        var runner:HandlingPlanRunner = cast handling;
        var home = runner.homePose;
        var state = localization.state();
        if (state == null) throw "Handling orientation needs a localized base";
        var worldHome = Transform3.fromPose2(state.pose).rotation.multiply(
          new Quat(home.qx, home.qy, home.qz, home.qw));
        var at:SceneArtifactPlace = cast step.at;
        return yawRotation(yaw - frameYaw(staticPose(at.occurrence))).multiply(worldHome);
      }
      var held = grasped;
      var tip = toolTip;
      if (held == null || tip == null) throw "A heading-constrained place needs a held part and tool";
      var link = simulation.linkPose(robotIndex, toolLink);
      var rotation = new Quat(link.rotation[0], link.rotation[1], link.rotation[2], link.rotation[3])
        .multiply(new Quat(tip.qx, tip.qy, tip.qz, tip.qw));
      return yawRotation(yaw - frameYaw(partFrame(held.occurrence))).multiply(rotation);
    };
  }

  /** Where the tool's contact is now, in metres. */
  function toolContact():Vec3 {
    var tip = toolTip;
    if (tip == null) throw "The mission has no tool";
    var link = simulation.linkPose(robotIndex, toolLink);
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
