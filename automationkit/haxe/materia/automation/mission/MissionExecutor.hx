package materia.automation.mission;

import materia.automation.facility.Facility;
import materia.automation.facility.FacilityRoute;
import materia.automation.facility.FacilityRouter;
import materia.automation.fleet.Fleet;
import materia.automation.fleet.FleetAssignment;
import materia.automation.fleet.TrafficManager;
import robotkit.skill.Skill;
import robotkit.skill.SkillStatus;
import robotkit.world.Robot;
import materia.automation.task.TaskKind;
import materia.automation.task.Transport;

/** Advances one assigned mission by composing its ordered tasks from RobotKit skills. */
class MissionExecutor {
  public final fleet:Fleet;
  public final assignment:FleetAssignment;
  public final facility:Facility;
  public final skillFactory:TaskSkillFactory;
  public final trafficManager:Null<TrafficManager>;
  public final trafficPriority:Int;
  public var status(default, null):MissionExecutionStatus = Idle;
  public var currentSkill(default, null):Null<Skill> = null;

  var hasStarted = false;
  var currentSkillStarted = false;
  var currentTrafficRoute:Null<FacilityRoute> = null;

  public function new(fleet:Fleet, assignment:FleetAssignment,
      facility:Facility, skillFactory:TaskSkillFactory,
      ?trafficManager:TrafficManager, ?trafficPriority:Int = 0) {
    if (fleet == null || assignment == null || facility == null || skillFactory == null)
      throw "MissionExecutor requires a fleet assignment, facility, and task skill factory";
    if (fleet.assignmentForMission(assignment.mission.id) != assignment)
      throw "MissionExecutor assignment is not active in the supplied fleet";
    if (trafficManager != null && (trafficManager.fleet != fleet ||
        trafficManager.facility != facility))
      throw "MissionExecutor traffic manager must use the same fleet and facility";
    this.fleet = fleet;
    this.assignment = assignment;
    this.facility = facility;
    this.skillFactory = skillFactory;
    this.trafficManager = trafficManager;
    this.trafficPriority = trafficPriority;
  }

  public function start():Void {
    if (hasStarted) throw "MissionExecutor can only be started once";
    hasStarted = true;
    if (assignment.mission.status != MissionStatus.Running) {
      fail("assigned mission is not running");
      return;
    }
    status = Running;
    startCurrentTask();
  }

  /** Advances the current skill with the latest snapshot from its assigned robot. */
  public function update(durationSeconds:Float):MissionExecutionStatus {
    if (status != Running) return status;
    if (!Math.isFinite(durationSeconds) || durationSeconds <= 0.0)
      return fail("mission update duration must be finite and positive");
    var robotValue = assignedRobot();
    if (robotValue == null) return fail("assigned robot is no longer attached to RobotWorld");
    if (currentSkill == null) return fail("mission has no skill for its current task");
    var robot:Robot = cast robotValue;
    if (!currentSkillStarted) {
      if (!startCurrentSkillIfReserved()) {
        if (status != Running) return status;
        return status;
      }
    }
    var skill:Skill = cast currentSkill;
    var skillStatus:SkillStatus;
    try skillStatus = skill.update(robot.snapshot(), durationSeconds)
    catch (error:Dynamic) return fail(Std.string(error));

    switch skillStatus {
      case Running:
      case Succeeded:
        releaseCurrentTrafficRoute();
        try assignment.mission.completeCurrentTask()
        catch (error:Dynamic) return fail(Std.string(error));
        if (assignment.mission.isTerminal()) {
          status = Succeeded;
          fleet.release(assignment.mission.id);
        } else {
          currentSkill = null;
          startCurrentTask();
        }
      case Failed(message): fail(message);
      case Cancelled:
        releaseCurrentTrafficRoute();
        assignment.mission.cancel();
        currentSkill = null;
        currentSkillStarted = false;
        status = Cancelled;
        fleet.release(assignment.mission.id);
      case Idle: fail("task skill returned to idle");
    }
    return status;
  }

  public function cancel():Void {
    if (status != Running) return;
    if (currentSkill != null) {
      try currentSkill.cancel() catch (_:Dynamic) {}
    }
    releaseCurrentTrafficRoute();
    if (!assignment.mission.isTerminal()) assignment.mission.cancel();
    currentSkill = null;
    currentSkillStarted = false;
    status = Cancelled;
    fleet.release(assignment.mission.id);
  }

  function startCurrentTask():Void {
    if (status != Running) return;
    currentSkillStarted = false;
    currentTrafficRoute = null;
    var taskValue = assignment.mission.currentTask();
    if (taskValue == null) {
      fail("running mission has no current task");
      return;
    }
    var task:materia.automation.task.Task = cast taskValue;
    var robotValue = assignedRobot();
    if (robotValue == null) {
      fail("assigned robot is no longer attached to RobotWorld");
      return;
    }
    var robot:Robot = cast robotValue;
    try {
      currentSkill = skillFactory.create(task, robot, facility);
      if (currentSkill == null) {
        fail('task skill factory returned null for "${task.id}"');
        return;
      }
      currentTrafficRoute = routeFor(task);
      startCurrentSkillIfReserved();
    } catch (error:Dynamic) {
      fail('could not start task "${task.id}": ${Std.string(error)}');
    }
  }

  function routeFor(task:materia.automation.task.Task):Null<FacilityRoute> {
    if (trafficManager == null) return null;
    return switch task.kind {
      case TaskKind.Transport:
        var transport:Transport = cast task;
        new FacilityRouter(facility).route(transport.pickupStationId,
          transport.destinationStationId);
      case _: null;
    };
  }

  /** Starts the task only after its complete route reservation is available. */
  function startCurrentSkillIfReserved():Bool {
    if (currentSkillStarted) return true;
    if (currentTrafficRoute != null) {
      var manager:TrafficManager = cast trafficManager;
      var route:FacilityRoute = cast currentTrafficRoute;
      try {
        if (!manager.reserveRoute(route, assignment.robotId, trafficPriority))
          return false;
      } catch (error:Dynamic) {
        fail('could not reserve facility route: ${Std.string(error)}');
        return false;
      }
    }
    var skill:Skill = cast currentSkill;
    try skill.start() catch (error:Dynamic) {
      fail('could not start task skill: ${Std.string(error)}');
      return false;
    }
    currentSkillStarted = true;
    return true;
  }

  function releaseCurrentTrafficRoute():Void {
    if (currentTrafficRoute != null && trafficManager != null) {
      var manager:TrafficManager = cast trafficManager;
      manager.releaseRoute(cast currentTrafficRoute, assignment.robotId);
    }
    currentTrafficRoute = null;
  }

  function assignedRobot():Null<Robot> {
    if (!fleet.containsRobot(assignment.robotId)) return null;
    return fleet.world.robot(assignment.robotId);
  }

  function fail(message:String):MissionExecutionStatus {
    if (currentSkill != null) {
      try currentSkill.cancel() catch (_:Dynamic) {}
    }
    releaseCurrentTrafficRoute();
    if (!assignment.mission.isTerminal()) {
      switch assignment.mission.status {
        case MissionStatus.Running: assignment.mission.fail(message);
        case MissionStatus.Pending: assignment.mission.cancel();
        case _: // The mission reached a terminal state outside this executor.
      }
    }
    currentSkill = null;
    currentSkillStarted = false;
    status = Failed(message);
    fleet.release(assignment.mission.id);
    return status;
  }
}
