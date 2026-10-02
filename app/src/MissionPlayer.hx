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
import robotkit.skill.WeldPlan;
import robotkit.skill.WeldSeam;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.manipulation.Manipulator;
import robotkit.model.Frame;
import robotkit.skill.HandlePart;
import robotkit.spatial.Vec3;
import robotkit.localization.SimulationTruthLocalization;
import robotkit.mobile.Pose2;
import robotkit.navigation.AStarPlanner;
import robotkit.navigation.Costmap2;
import robotkit.navigation.Navigation;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Navigator;
import robotkit.navigation.OccupancyCell;
import robotkit.navigation.OccupancyGrid2;
import robotkit.perception.PerceptionSnapshot;
import robotkit.runtime.Simulation;
import robotkit.skill.GoTo;
import robotkit.skill.Skill;
import robotkit.skill.SkillRunner;
import robotkit.skill.SkillStatus;
import robotkit.world.RobotSnapshot;

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

  public final mission:SceneArtifactMission;
  /** The navigation map, when the mission drives. */
  public final costmap:Null<Costmap2>;
  /** The boxes the map was drawn from. */
  public final obstacles:Array<FloorObstacle>;
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
  final timestep:Float;
  final idle = new PerceptionSnapshot();

  public function new(mission:SceneArtifactMission, robot:AssemblyRobot, simulation:Simulation, robotIndex:Int,
      timestep:Float, obstacles:Array<FloorObstacle>, objects:Array<GripObject>, project:ProjectDocumentSession) {
    if (mission.steps.length == 0) throw "A mission needs steps";
    this.mission = mission;
    this.robot = robot;
    this.timestep = timestep;
    this.obstacles = obstacles.copy();
    this.simulation = simulation;
    this.objects = objects;
    this.robotIndex = robotIndex;
    this.project = project;
    var physical = project.projectPhysical;
    metres = physical == null ? 1.0 : physical.metresPerUnit;
    var definition = project.projectAssemblyDefinition;
    assembly = definition == null ? null : AssemblyDefinitionFlattener.flatten(definition);
    placement = definition == null ? null : new AssemblyState(definition, project.projectAssemblyState);
    localization = new SimulationTruthLocalization(simulation, robotIndex, FRAME, "base");
    var drives = [for (step in mission.steps) if (step.kind == "goTo") step];
    var base = robot.mobile;
    if (drives.length == 0) {
      costmap = null;
      navigator = null;
    } else {
      if (base == null) throw "A mission that drives needs the project's wheeled assembly";
      var footprint = base.footprint;
      if (footprint == null) throw "A mission that drives needs the robot's footprint";
      costmap = new Costmap2(floorPlan(obstacles, [for (step in drives) floorPose(step)]), footprint.radius, true, 0.3,
        1.5);
      navigator = new Navigator(new Navigation(base, localization, 0.35, 0.5, 1.2, false), new AStarPlanner(costmap),
        costmap, 0.5);
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
    var part = robot.part("project:" + tool.contact.occurrence);
    var contact = connectorFrame(tool.contact);
    var tip = AssemblyFrames.compose(part.offset, {x: contact.x * metres, y: contact.y * metres, z: contact.z * metres,
      qx: contact.qx, qy: contact.qy, qz: contact.qz, qw: contact.qw});
    toolLink = part.linkIndex;
    toolTip = tip;
    var model = robot.model;
    var tcp = model.addFrame(new Frame("mission tool", model.links[part.linkIndex]));
    tcp.position = [tip.x, tip.y, tip.z];
    tcp.rotation = [tip.qx, tip.qy, tip.qz, tip.qw];
    return new Manipulator(model, model.links[0].id, tcp.id);
  }

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
    var standing = [for (box in obstacles) if (box.z - box.halfZ < CLEARANCE && box.z + box.halfZ > 0.01) box];
    for (box in standing) {
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
    for (box in standing) {
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

  /**
   * Back to the first step; the next tick starts it from wherever the reset put the robot. The
   * session has already started the robot's runtime over, so the step that was running is dropped,
   * not cancelled: a cancel would stop the fresh runtime.
   */
  public function reset():Void {
    runner = new SkillRunner();
    stepIndex = 0;
    completed = 0;
    failure = null;
    finished = false;
    grasped = null;
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

  /**
   * The weld a mission step describes, as a plan in the map frame (the world). Its path is given relative to the
   * workpiece's reference member, and that member is found where it stands when the step starts, as a pick finds its part,
   * so the weld follows a workpiece that is not where it was designed. A weld with no frame is in the assembly as designed.
   */
  /** The world pose of a weld's reference member now, or the world's own when the weld names none. */
  function referenceFrame(weld:SceneArtifactWeld):Transform3 {
    if (weld.frame == null || weld.frame == "") return Transform3.identity();
    var live = AssemblyRobot.partPose(simulation, robot.part("project:" + weld.frame));
    return new Transform3(new Vec3(live.position[0], live.position[1], live.position[2]),
      new Quat(live.rotation[0], live.rotation[1], live.rotation[2], live.rotation[3]));
  }

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

  /** No sensed obstacles beyond the map yet; the robot's place is updated every tick. */
  function observe(snapshot:RobotSnapshot):PerceptionSnapshot return idle;

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
    var definitionId = definitionOf(at.occurrence);
    for (component in flat.definitions) if (component.id == definitionId)
      for (connector in component.connectors) if (connector.name == at.connector) return connector.frame;
    throw 'Occurrence "${at.occurrence}" has no connector "${at.connector}"';
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
