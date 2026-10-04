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
import processkit.WeldingPlanRunner;
import processkit.skill.WeldPlan;
import processkit.skill.WeldSeam;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.manipulation.Manipulator;
import robotkit.model.Frame;
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
typedef FloorObstacle = {
  var id:String;
  var x:Float;
  var y:Float;
  var z:Float;
  var halfX:Float;
  var halfY:Float;
  var halfZ:Float;
  var yaw:Float;
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
  public static inline var RESOLUTION:Float = 0.05;
  /** Room left round the obstacles' extent so the robot's own place is always on the map. */
  public static inline var MARGIN:Float = 1.0;
  /** Boxes this far above the floor or higher pass over the robot. */
  public static inline var CLEARANCE:Float = 1.0;
  public static inline var FRAME:String = "map";
  /** How close a `goTo` stands to its pose, in metres and radians. */
  public static inline var POSITION_TOLERANCE:Float = 0.05;
  public static inline var HEADING_TOLERANCE:Float = 0.05;
  /** Joint acceleration the arm programs plan with, rad/s². */
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
  var runner = new SkillRunner();
  /** The arm and its suction tool's vacuum sensor, when the mission picks and places. */
  var handling:Null<HandlingPlanRunner>;
  /** Makes the arm's handling runner afresh: a reset starts the robot's runtime over, plans and all. */
  final newHandling:Null<Void->HandlingPlanRunner>;
  /** The arm and its torch, when the mission welds; and the torch's weld sensor. */
  var welding:Null<WeldingPlanRunner>;
  final newWelding:Null<Void->WeldingPlanRunner>;
  final weldSensor:Null<String>;
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
      timestep:Float, obstacles:Array<FloorObstacle>, objects:Array<GripObject>, project:ProjectDocumentSession) {
    if (mission.steps.length == 0) throw "A mission needs steps";
    this.mission = mission;
    this.robot = robot;
    this.timestep = timestep;
    this.obstacles = standing(obstacles);
    this.simulation = simulation;
    this.objects = objects;
    this.robotIndex = robotIndex;
    this.project = project;
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
      var map = new Costmap2(floorPlan(obstacles, [for (step in drives) floorPose(step)]), footprint.radius, true, 0.3, 1.5, MEMORY_SECONDS);
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
      newHandling = () -> HandlingPlanRunner.create(robot.robot, arm, () -> robot.runtime.pollEvents(), tool.channel,
        ARM_ACCELERATION);
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
      newWelding = () -> WeldingPlanRunner.create(robot.robot, arm, () -> robot.runtime.pollEvents(), channels, ARM_ACCELERATION);
      welding = newWelding();
    }
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
    return [for (box in boxes) if (box.z - box.halfZ < CLEARANCE && box.z + box.halfZ > 0.01) box];

  /**
   * An occupancy grid covering the obstacles and `poses` with a margin: a cell is occupied when its
   * square overlaps a box that stands within the robot's height.
   */
  public static function floorPlan(obstacles:Array<FloorObstacle>, poses:Array<Pose2>):OccupancyGrid2 {
    var minX = Math.POSITIVE_INFINITY, minY = Math.POSITIVE_INFINITY;
    var maxX = Math.NEGATIVE_INFINITY, maxY = Math.NEGATIVE_INFINITY;
    function extend(x:Float, y:Float):Void {
      minX = Math.min(minX, x); maxX = Math.max(maxX, x);
      minY = Math.min(minY, y); maxY = Math.max(maxY, y);
    }
    var blocking = standing(obstacles);
    for (box in blocking) {
      var reach = Math.sqrt(box.halfX * box.halfX + box.halfY * box.halfY);
      extend(box.x - reach, box.y - reach);
      extend(box.x + reach, box.y + reach);
    }
    for (pose in poses) extend(pose.x, pose.y);
    if (!Math.isFinite(minX)) throw "A mobile mission needs somewhere to drive";
    var width = Math.ceil((maxX - minX + 2 * MARGIN) / RESOLUTION);
    var height = Math.ceil((maxY - minY + 2 * MARGIN) / RESOLUTION);
    var grid = new OccupancyGrid2(RESOLUTION, new Pose2(minX - MARGIN, minY - MARGIN), width, height, FRAME,
      OccupancyCell.Free);
    // A cell square overlaps a box when its centre lies within the box grown by half a cell's diagonal
    // projected on each box axis; growing by the full half-diagonal is the safe side of that.
    var grow = RESOLUTION * Math.sqrt(0.5);
    for (box in blocking) {
      var c = Math.cos(box.yaw), s = Math.sin(box.yaw);
      var reach = Math.sqrt(box.halfX * box.halfX + box.halfY * box.halfY) + grow;
      var low = grid.worldToCell(new Pose2(box.x - reach, box.y - reach));
      var high = grid.worldToCell(new Pose2(box.x + reach, box.y + reach));
      if (low == null || high == null) throw 'Obstacle "${box.id}" is off the map';
      for (cx in low.x...high.x + 1) for (cy in low.y...high.y + 1) {
        var centre = grid.cellCenter(cx, cy);
        var dx = centre.x - box.x, dy = centre.y - box.y;
        var along = dx * c + dy * s, across = -dx * s + dy * c;
        if (Math.abs(along) <= box.halfX + grow && Math.abs(across) <= box.halfY + grow)
          grid.setCell(cx, cy, OccupancyCell.Occupied);
      }
    }
    return grid;
  }

  public function feed():Void {
    if (failure != null || finished) return;
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
  public function beforeReset():Void runner.cancel();

  /** Back to the first step; the next tick starts it from wherever the reset put the robot. */
  public function reset():Void {
    runner = new SkillRunner();
    stepIndex = 0;
    completed = 0;
    failure = null;
    finished = false;
    grasped = null;
    wheelsSeeded = false;
    scanned = null;
    sensed = new PerceptionSnapshot();
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

  /** How many times the arc was lost and the weld restarted, in the weld running or last run. */
  public function weldRestarts():Int {
    var active = welding;
    return active == null ? 0 : active.restarts();
  }

  public function present():Void {}

  /** After a tick: a finished step hands over to the next, a failed one stops the mission. */
  function settle():Void switch runner.status() {
    case Succeeded:
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
      case "goTo":
        var activeNavigator:Navigator = cast navigator;
        return new GoTo(activeNavigator, new NavigationGoal(floorPose(step), FRAME, POSITION_TOLERANCE, HEADING_TOLERANCE),
          observe);
      case "pick":
        var at:SceneArtifactPlace = cast step.at;
        grasped = at;
        return HandlePart.pick(cast handling, localization, () -> graspPoint(at), vacuumSensor);
      case "place":
        var at:SceneArtifactPlace = cast step.at;
        return HandlePart.place(cast handling, localization, () -> placeContact(at), vacuumSensor);
      case "weld":
        var weld:SceneArtifactWeld = cast step.weld;
        return new WeldSeam(cast welding, localization, () -> weldPlan(weld), cast weldSensor);
      default:
        throw 'Mission step kind "${step.kind}" is not supported';
    }
  }

  /** The world pose of a weld's reference member now, or the world's own when the weld names none. */
  function referenceFrame(weld:SceneArtifactWeld):Transform3 {
    if (weld.frame == null || weld.frame == "") return Transform3.identity();
    var live = AssemblyRobot.partPose(simulation, robot.part("project:" + weld.frame));
    return new Transform3(new Vec3(live.position[0], live.position[1], live.position[2]),
      new Quat(live.rotation[0], live.rotation[1], live.rotation[2], live.rotation[3]));
  }

  /**
   * The weld a mission step describes, as a plan in the map frame (the world). Its path is given relative to the
   * workpiece's reference member, and that member is found where it stands when the step starts, as a pick finds its part,
   * so the weld follows a workpiece that is not where it was designed. A weld with no frame is in the assembly as designed.
   */
  function weldPlan(weld:SceneArtifactWeld):WeldPlan {
    var frame = referenceFrame(weld);
    function pose(torch:SceneArtifactTorchPose):Transform3
      return frame.compose(new Transform3(new Vec3(torch.position[0], torch.position[1], torch.position[2]),
        new Quat(torch.rotation[0], torch.rotation[1], torch.rotation[2], torch.rotation[3])));
    var process = weld.process;
    return new WeldPlan([for (segment in weld.path) new WeldSegment(pose(segment.start), pose(segment.stop))],
      {wireSpeed: process.wireSpeed, voltage: process.voltage, travelSpeed: process.travelSpeed, approach: process.approach,
        startDwell: process.startDwell, craterDwell: process.craterDwell, burnback: process.burnback});
  }

  /**
   * What the lidar sees beyond the map, in the map frame, read once per scan: a scan's obstacles stay where
   * they were seen until the next one, however far the robot moves meanwhile.
   */
  function observe(snapshot:RobotSnapshot):PerceptionSnapshot {
    var reading = scanning;
    var name = scanner;
    if (reading == null || name == null) return idle;
    var sequence:Null<Int64> = null;
    for (frame in snapshot.sensors.toArray()) if (frame.sensorId == name) sequence = frame.sequence;
    if (sequence == null) return idle;
    var latest:Int64 = cast sequence;
    var before = scanned;
    if (before != null && Int64.compare(before, latest) == 0) return sensed;
    scanned = latest;
    sensed = reading.observeRobotSnapshot(snapshot, robot.model, robot.blueprint, robot.rootLink);
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
  function placeContact(at:SceneArtifactPlace):Vec3 {
    var held = grasped;
    if (held == null) throw "Nothing was picked to place";
    var seat = AssemblyFrames.compose(staticPose(at.occurrence), connectorFrame(at));
    var origin = partFrame(held.occurrence);
    var tool = toolContact();
    return new Vec3(seat.x * metres + tool.x - origin.x, seat.y * metres + tool.y - origin.y,
      seat.z * metres + tool.z - origin.z);
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
