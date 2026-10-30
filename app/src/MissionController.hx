package app;

import app.MissionPlayer.MissionProgress;
import materia.project.SceneArtifact.SceneArtifactMission;

typedef MissionViewState = {
  var generation:Int;
  var mission:Null<SceneArtifactMission>;
  var machining:Null<materia.project.SceneArtifact.SceneArtifactMachiningMission>;
  var machiningStatus:String;
  var machiningPreview:Bool;
  var hasWork:Bool;
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
  final inspect:String -> Void;
  final changed:Void -> Void;
  /** The robot controller whose program the views follow; null follows the first. */
  var program:Null<String> = null;

  public function new(
    session:ProjectDocumentSession,
    simulation:ApplicationSimulation,
    changed:Void -> Void,
    inspect:String -> Void
  ) {
    this.session = session;
    this.simulation = simulation;
    this.changed = changed;
    this.inspect = inspect;
  }

  public function generation():Int return session.generation;

  public function selectedProgram():String {
    var authored = session.missions;
    if (authored == null || authored.length == 0) return "";
    if (program != null) for (work in authored) if ((work.controllerId ?? "") == program) return program;
    return authored[0].controllerId ?? "";
  }

  public function currentPlayer():Null<MissionPlayer> {
    if (session.missions == null || session.missions.length == 0) return null;
    var chosen = selectedProgram();
    return chosen == "" ? simulation.missionPlayer() : simulation.missionPlayer(chosen);
  }

  public function currentMission():Null<SceneArtifactMission> {
    var player = currentPlayer();
    if (player != null) return player.mission;
    var authored = session.missions, chosen = selectedProgram();
    if (authored != null) for (work in authored) if ((work.controllerId ?? "") == chosen) return work;
    return null;
  }

  public function activeStep():Int {
    var player = currentPlayer();
    if (player == null) return -1;
    var phase = player.progressPhase();
    return phase == "startup" || phase == "complete" ? -1 : player.stepIndex;
  }

  public function hasWork():Bool return currentMission() != null || session.cncJob != null && session.cncJob.mission != null;

  /** Request a frame for view-local choices without changing execution. */
  public function requestRefresh():Void changed();

  public function observe(context:haxeon.ui.core.BuildContext):Void {
    context.observeInt(generation);
    context.observeInt(() -> session.document.revision);
    var authored = session.missions, job = session.cncJob;
    context.observeBool(() -> session.missions == authored);
    context.observeBool(() -> session.cncJob == job);
    context.observeString(selectedProgram);
    context.observeBool(simulation.isRunning);
    var player = currentPlayer();
    context.observeBool(() -> currentPlayer() == player);
    if (player != null) {
      context.observeString(player.progressPhase);
      context.observeInt(() -> player.stepIndex);
      context.observeInt(() -> player.completed);
      context.observeString(() -> player.failure);
      context.observeString(player.startupAxis);
      context.observeString(player.startupPhase);
    }
    if (job != null && job.mission != null) {
      var cnc = simulation.cncPlayer();
      context.observeBool(() -> simulation.cncPlayer() == cnc);
      if (cnc != null) {
        context.observeString(() -> cnc.failure);
        context.observeInt(() -> cnc.currentLine);
        context.observeBool(cnc.finished);
        context.observeBool(cnc.hasStarted);
        context.observeBool(cnc.held);
        context.observeString(cnc.statusDetail);
      }
    }
  }

  public function snapshot():MissionViewState {
    // The one implicit controller of a scene without modelled controllers is "".
    var programs:Array<String> = session.missions
      == null ? [] : [for (work in session.missions) work.controllerId == null ? "" : work.controllerId];
    var chosen = selectedProgram();
    var player = currentPlayer();
    var progress:Null<MissionProgress> = player == null ? null : player.progress();
    var status = player == null ? "Ready · Play to start" : player.statusLabel(true);
    if (progress != null && !simulation.isRunning()
      && progress.phase != "complete" && progress.phase != "failed") status = "Paused · " + status;
    var mission = currentMission();
    if (mission != null && mission.loop == true) status += " · Loop " + (progress == null ? 1 : progress.loop);
    var cncJob = session.cncJob;
    var machining = cncJob == null ? null : cncJob.mission;
    var machiningStatus = "";
    if (machining != null) {
      var cnc = simulation.cncPlayer();
      machiningStatus = cnc == null ? "Ready · Play to start"
        : cnc.failure != null ? "Stopped · " + ProgramErrorMessage.fromFailure(cnc.failure).title : cnc.finished()
        ? "Completed" : !cnc.hasStarted() ? "Ready · Play to start" : !simulation.isRunning() ? "Paused" : cnc.held()
        ? "Held" : "Running" + (cnc.currentLine > 0 ? " · line " + cnc.currentLine : "") + cnc.statusDetail();
    }
    if (mission == null) status = machining == null ? "No mission" : machining.label + " · " + machiningStatus;
    return {
      generation: session.generation,
      mission: mission,
      machining: machining,
      machiningStatus: machiningStatus,
      machiningPreview: cncJob != null && cncJob.kinematicPreview == true,
      hasWork: mission != null || machining != null,
      programs: programs,
      selectedProgram: chosen,
      progress: progress,
      status: status
    };
  }

  /** Follow the program of robot controller `id`. */
  public function selectProgram(id:String):Void {
    if (session.missions == null || [for (work in session.missions) if (work.controllerId
      == id) work].length != 1) throw 'No program runs on controller "$id"';
    program = id;
    changed();
  }

  public function inspectStep(index:Int):Void {
    var mission = currentMission();
    if (mission == null || index < 0 || index >= mission.steps.length) return;
    var target = MissionStepPresentation.targetOf(mission.steps[index]);
    if (target != null) inspect(target);
    changed();
  }
}
