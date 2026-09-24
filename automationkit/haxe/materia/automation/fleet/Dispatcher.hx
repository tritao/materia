package materia.automation.fleet;

import materia.automation.mission.Mission;

/** Deterministic first-available assignment policy for a single fleet. */
class Dispatcher {
  public final fleet:Fleet;

  public function new(fleet:Fleet) {
    if (fleet == null) throw "Dispatcher requires a fleet";
    this.fleet = fleet;
  }

  /** Returns null when no ready, unassigned fleet robot is available. */
  public function dispatch(mission:Mission):Null<FleetAssignment> {
    if (mission == null || mission.status != materia.automation.mission.MissionStatus.Pending)
      throw "Dispatcher requires a pending mission";
    var available = fleet.availableRobotIds();
    if (available.length == 0) return null;
    return fleet.assign(available[0], mission);
  }

  public function releaseCompleted(mission:Mission):Void {
    if (mission == null || !mission.isTerminal()) throw "Only a terminal mission can be released";
    fleet.release(mission.id);
  }
}
