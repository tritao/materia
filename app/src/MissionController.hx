package app;

import app.ProjectJobs.ProjectJob;
import app.MissionPlayer.MissionProgress;
import materia.project.SceneArtifact.SceneArtifactMission;

typedef MissionViewState = {
  var generation:Int;
  var jobs:Array<ProjectJob>;
  var selectedJob:Null<String>;
  var summary:String;
  var mission:Null<SceneArtifactMission>;
  var progress:Null<MissionProgress>;
  var status:String;
  var switchAllowed:Bool;
  var simulationActive:Bool;
  var loading:Bool;
  var error:Null<String>;
}

/** Shared application state for mission views; job replacement is prepared before installation. */
class MissionController {
  final session:ProjectDocumentSession;
  final simulation:ApplicationSimulation;
  final requestChange:(Void->Void)->Void;
  final changed:Void->Void;
  final installed:Void->Void;
  final blocked:Void->Bool;
  final inspect:String->Void;
  var generation = -1;
  var jobs:Array<ProjectJob> = [];
  var failure:Null<String> = null;
  var loading:Null<ProjectLoadJob> = null;
  var reference:Null<String>;
  var loadGeneration = -1;

  public function new(session:ProjectDocumentSession, simulation:ApplicationSimulation,
      requestChange:(Void->Void)->Void, changed:Void->Void, installed:Void->Void,
      blocked:Void->Bool, inspect:String->Void) {
    this.session = session; this.simulation = simulation; this.requestChange = requestChange;
    this.changed = changed; this.installed = installed; this.blocked = blocked; this.inspect = inspect;
  }

  public function isLoading():Bool return loading != null;

  function sync():Void {
    if (generation == session.generation) return;
    generation = session.generation;
    jobs = []; failure = null;
    var path = session.projectReference;
    if (path != null && !ProjectSourceLoader.isPrebuilt(path))
      try jobs = ProjectJobs.read(path) catch (error:Dynamic) failure = Std.string(error);
  }

  public function snapshot():MissionViewState {
    sync();
    var player = simulation.missionPlayer();
    var progress:Null<MissionProgress> = player == null ? null : player.progress();
    var status = player == null ? "Ready · Play to start" : player.statusLabel(true);
    if (progress != null && !simulation.isRunning() && progress.phase != "complete" && progress.phase != "failed")
      status = "Paused · " + status;
    var mission = player == null ? session.mission : player.mission;
    if (mission != null && mission.loop == true) status += " · Loop " + (progress == null ? 1 : progress.loop);
    var summary = "";
    for (job in jobs) if (job.id == session.projectJob) summary = job.summary;
    var task = loading;
    if (task != null) status = "Loading job · " + task.control.currentPhase();
    return {generation:generation, jobs:jobs.copy(), selectedJob:session.projectJob, summary:summary,
      mission:mission, progress:progress, status:status, switchAllowed:!simulation.isActive() && task == null && !blocked(),
      simulationActive:simulation.isActive(), loading:task != null, error:failure};
  }

  public function selectJob(id:String):Void {
    var state = snapshot();
    if (!state.switchAllowed || id == state.selectedJob) return;
    var valid = false;
    for (job in state.jobs) if (job.id == id) valid = true;
    if (!valid) throw 'Unknown project job "$id"';
    var path = session.projectReference;
    var expectedGeneration = session.generation;
    if (path == null) return;
    requestChange(function() {
      if (simulation.isActive() || isLoading() || session.generation != expectedGeneration) return;
      reference = path; loadGeneration = session.generation; failure = null;
      loading = new ProjectLoadJob(path, id);
      changed();
    });
  }

  public function tick():Void {
    var task = loading;
    if (task == null) return;
    if (session.generation != loadGeneration) { cancel(); return; }
    if (!task.isFinished()) { changed(); return; }
    loading = null;
    if (!task.wasCancelled()) try {
      var candidate = task.take();
      // The document and simulation may have changed while generation was in flight.
      if (simulation.isActive()) throw "Stop simulation before changing jobs";
      var path = reference;
      if (path == null) throw "Job source is missing";
      session.openGeneratedProject(candidate, path);
      sync();
      installed();
    } catch (error:Dynamic) failure = "Could not load job: " + Std.string(error);
    changed();
  }

  public function cancel():Void {
    if (loading != null) loading.control.cancel();
    loading = null;
    changed();
  }

  public function inspectStep(index:Int):Void {
    var mission = snapshot().mission;
    if (mission == null || index < 0 || index >= mission.steps.length) return;
    var target = MissionStepPresentation.targetOf(mission.steps[index]);
    if (target != null) inspect(target);
  }
}
