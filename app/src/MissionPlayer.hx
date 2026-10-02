package app;

import materia.project.SceneArtifact.SceneArtifactMission;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
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
 * and follows the route with pure pursuit, the robot's place coming from the simulation.
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

  final robot:AssemblyRobot;
  final runner = new SkillRunner();
  final navigator:Null<Navigator>;
  final localization:SimulationTruthLocalization;
  final timestep:Float;
  final idle = new PerceptionSnapshot();

  public function new(mission:SceneArtifactMission, robot:AssemblyRobot, simulation:Simulation, robotIndex:Int,
      timestep:Float, obstacles:Array<FloorObstacle>) {
    if (mission.steps.length == 0) throw "A mission needs steps";
    this.mission = mission;
    this.robot = robot;
    this.timestep = timestep;
    this.obstacles = obstacles.copy();
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
    if (failure != null) return;
    var snapshot = robot.robot.snapshot();
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

  /** Back to the first step; the next tick starts it from wherever the reset put the robot. */
  public function reset():Void {
    runner.cancel();
    stepIndex = 0;
    completed = 0;
    failure = null;
    // The cancelled runner starts the first step on the next tick.
  }

  public function present():Void {}

  /** After a tick: a finished step hands over to the next, a failed one stops the mission. */
  function settle():Void switch runner.status() {
    case Succeeded:
      completed++;
      if (stepIndex + 1 < mission.steps.length || mission.loop == true)
        stepIndex = (stepIndex + 1) % mission.steps.length;
      else
        failure = null;
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
      default:
        throw 'Mission step kind "${step.kind}" is not supported';
    }
  }

  /** Where the robot is, from the simulation; no sensed obstacles beyond the map yet. */
  function observe(snapshot:RobotSnapshot):PerceptionSnapshot {
    localization.update(snapshot);
    return idle;
  }

  static function floorPose(step:SceneArtifactMissionStep):Pose2 {
    var pose = step.pose;
    if (pose == null) throw 'Mission step "${step.kind}" needs a pose';
    return new Pose2(pose.x, pose.y, pose.yaw);
  }
}
