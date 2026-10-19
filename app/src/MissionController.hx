package app;

import app.MissionPlayer.MissionProgress;
import materia.project.SceneArtifact.SceneArtifactMission;

typedef MissionViewState = {
  var generation:Int;
  var mission:Null<SceneArtifactMission>;
  /** Every program's robot controller, in authored order; empty for the one implicit controller. */
  var programs:Array<String>;
  var selectedProgram:String;
  var progress:Null<MissionProgress>;
  var status:String;
}

/** Runtime mission execution and progress. Project variants are managed separately. */
class MissionController {
  final session:ProjectDocumentSession;
  final simulation:ApplicationSimulation;
  final inspect:String->Void;
  final changed:Void->Void;
  /** The robot controller whose program the views follow; null follows the first. */
  var program:Null<String> = null;

  public function new(session:ProjectDocumentSession,simulation:ApplicationSimulation,changed:Void->Void,inspect:String->Void) {
    this.session=session;this.simulation=simulation;this.changed=changed;this.inspect=inspect;
  }

  public function snapshot():MissionViewState {
    // The one implicit controller of a scene without modelled controllers is "".
    var programs:Array<String> = session.missions == null ? [] :
      [for (work in session.missions) work.controllerId == null ? "" : work.controllerId];
    var chosen:String = program == null || programs.indexOf(program) < 0 ? (programs.length == 0 ? "" : programs[0]) : program;
    var player = programs.length == 0 ? null : chosen == "" ? simulation.missionPlayer() : simulation.missionPlayer(chosen);
    var progress:Null<MissionProgress> = player == null ? null : player.progress();
    var status = player == null ? "Ready · Play to start" : player.statusLabel(true);
    if (progress != null && !simulation.isRunning() && progress.phase != "complete" && progress.phase != "failed")
      status = "Paused · " + status;
    var mission:Null<SceneArtifactMission> = player != null ? player.mission : null;
    if (mission == null && session.missions != null)
      for (work in session.missions) if (mission == null && (work.controllerId == null ? "" : work.controllerId) == chosen) mission = work;
    if (mission != null && mission.loop == true) status += " · Loop " + (progress == null ? 1 : progress.loop);
    if(mission==null)status="No mission";
    return {generation:session.generation,mission:mission,programs:programs,selectedProgram:chosen,
      progress:progress,status:status};
  }

  /** Follow the program of robot controller `id`. */
  public function selectProgram(id:String):Void {
    if (session.missions == null || [for (work in session.missions) if (work.controllerId == id) work].length != 1)
      throw 'No program runs on controller "$id"';
    program = id;
    changed();
  }

  public function inspectStep(index:Int):Void {
    var mission = snapshot().mission;
    if (mission == null || index < 0 || index >= mission.steps.length) return;
    var target = MissionStepPresentation.targetOf(mission.steps[index]);
    if (target != null) inspect(target);
  }
}
