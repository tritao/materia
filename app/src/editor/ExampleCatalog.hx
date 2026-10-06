package app.editor;

import app.MateriaProjectRunner;
import app.MateriaProjectRunner.GeneratedAssemblyScene;
import app.ProjectLoadJob;
import app.Main.ReferenceEditorApp;
import app.SetupScriptRegistry;
import haxe.Json;
import haxe.io.Path;
import sys.io.File;
import sys.FileSystem;

enum ExampleKind {
  /** A Materia project manifest, compiled and run by Haxeon. */
  Project(path:String);
  /** A registered, compiled setup script (see SetupScriptRegistry). */
  Script(reference:String);
  /** The saved rack-to-table worker document, started in realtime. */
  WorkerRackToTable;
  /** The worker gallery: one lane for each case the worker is built for, run side by side in realtime. */
  WorkerGallery;
}

typedef ExampleEntry = {
  final id:String;
  final title:String;
  /** Short lines of description; each is shown on its own line so cards need no text wrapping. */
  final description:Array<String>;
  final tag:String;
  final kind:ExampleKind;
}

/** Bundled demos offered on the Start page. Entries whose files are missing are not listed. */
@:access(app.Main.ReferenceEditorApp)
class ExampleCatalog {
  public static final entries:Array<ExampleEntry> = [
    {id: "picking-station", title: "Picking station",
      description: ["A generated cell with an", "order workflow in the Inspector"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/picking-station/materia.project.json")},
    {id: "motor-shaft-bearings", title: "Motor, shaft and bearings",
      description: ["Standard parts assembled", "from MachineKit generators"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/materia.project.json")},
    {id: "robot-arm", title: "Six-axis robot arm",
      description: ["A generated arm on a pedestal:", "press Play to run its pick motion"],
      tag: "Simulation · first build ~30 s",
      kind: Project("machinekit/examples/robot-arm/materia.project.json")},
    {id: "cobot-reach500", title: "Cobot 500 mm",
      description: ["3 kg payload size class", "running a six-joint motion loop"],
      tag: "Simulation · first build ~30 s",
      kind: Project("machinekit/examples/cobot-arm/materia.reach500.project.json")},
    {id: "cobot-reach850", title: "Cobot 850 mm",
      description: ["5 kg payload size class", "running a six-joint motion loop"],
      tag: "Simulation · first build ~30 s",
      kind: Project("machinekit/examples/cobot-arm/materia.project.json")},
    {id: "cobot-reach900", title: "Cobot 900 mm",
      description: ["16 kg payload size class", "running a six-joint motion loop"],
      tag: "Simulation · first build ~30 s",
      kind: Project("machinekit/examples/cobot-arm/materia.reach900.project.json")},
    {id: "cobot-reach1300", title: "Cobot 1300 mm",
      description: ["12.5 kg payload size class", "running a six-joint motion loop"],
      tag: "Simulation · first build ~30 s",
      kind: Project("machinekit/examples/cobot-arm/materia.reach1300.project.json")},
    {id: "bench-mill", title: "Bench mill",
      description: ["A servo bench mill", "machining a bearing block"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/bench-mill/materia.project.json")},
    {id: "enclosed-bench-mill", title: "Enclosed bench mill",
      description: ["A mill enclosure, sliding door", "and preset pneumatic vise"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/bench-mill/materia.enclosed.project.json")},
    {id: "gantry-picker", title: "Gantry picker",
      description: ["A Cartesian suction gantry", "moves six cartons onto a pallet"],
      tag: "Simulation · first build ~30 s",
      kind: Project("machinekit/examples/gantry-picker/materia.project.json")},
    {id: "cnc-router", title: "Desktop CNC router",
      description: ["A generated three-axis gantry", "router with stock on its bed"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/cnc-router/materia.project.json")},
    {id: "mobile-base", title: "Mobile base",
      description: ["A generated differential-drive", "robot base with a lidar"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/mobile-base/materia.project.json")},
    {id: "robot-welder", title: "Robot welder",
      description: ["A generated MIG welding cell: arm,", "torch, power source and a weldment"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/robot-welder/materia.project.json")},
    {id: "robot-welder-seam", title: "Robot welder: one seam",
      description: ["A focused MIG welding mission", "along a plate T-joint"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/robot-welder/materia.seam.project.json")},
    {id: "robot-welder-post", title: "Robot welder: tube post",
      description: ["Weld four sides of a tube post", "as one continuous path"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/robot-welder/materia.post.project.json")},
    {id: "robot-welder-weave", title: "Robot welder: woven seam",
      description: ["A 7 mm seam deposited", "with a weaving torch path"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/robot-welder/materia.weave.project.json")},
    {id: "robot-welder-multipass", title: "Robot welder: three passes",
      description: ["A 10 mm seam built up", "over three welding passes"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/robot-welder/materia.multipass.project.json")},
    {id: "mobile-robot-welder", title: "Mobile robot welder",
      description: ["A mobile welding cell", "with an arm and torch"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/robot-welder/materia.mobile.project.json")},
    {id: "mobile-welding-mission", title: "Mobile welding mission",
      description: ["A planned welding mission", "from mobile work stations"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/robot-welder/materia.mobilemission.project.json")},
    {id: "gantry-welder", title: "Gantry welder",
      description: ["A Cartesian gantry", "carrying a welding torch"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/gantry-welder/materia.project.json")},
    {id: "track-welder", title: "Coordinated track welding",
      description: ["An arm on a linear track", "with coordinated welding motion"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/track-arm/materia.welder.project.json")},
    {id: "cad-modeling", title: "CAD modelling",
      description: ["Parametric sketches, extrusions", "and features in CadKit"],
      tag: "Project · first build ~15 s",
      kind: Project("cadkit/examples/modeling/materia.project.json")},
    {id: "two-robot", title: "Two robots with sensors",
      description: ["A script-owned setup: press Play", "to simulate LiDAR and IMUs"],
      tag: "Simulation",
      kind: Script("materia.examples.two-robot")},
    {id: "worker-rack-to-table", title: "Worker moves a rack",
      description: ["A walking human worker in a", "running physics simulation"],
      tag: "Simulation · starts running",
      kind: WorkerRackToTable},
    {id: "worker-gallery", title: "Worker gallery",
      description: ["Six workers side by side: bending,", "crouching, kneeling, turning, two hands"],
      tag: "Simulation · starts running",
      kind: WorkerGallery}
  ];

  /** Id of the entry built for a project named on the command line; it is not part of `entries`. */
  public static final LAUNCH_PROJECT_ID:String = "launch-project";

  /** A project named at launch, opened by the same background build as a Start page example. */
  public static function launchEntry(projectPath:String):ExampleEntry {
    return {id: LAUNCH_PROJECT_ID, title: projectTitle(projectPath), description: [], tag: "",
      kind: Project(projectPath)};
  }

  /** The manifest's own name, or its folder name when the manifest cannot be read yet. */
  static function projectTitle(projectPath:String):String {
    try {
      var name:Dynamic = Reflect.field(Json.parse(File.getContent(projectPath)), "name");
      if (Std.isOfType(name, String) && StringTools.trim(name).length > 0) return name;
    } catch (_:Dynamic) {}
    var folder = Path.withoutDirectory(Path.directory(projectPath));
    return folder.length > 0 ? folder : projectPath;
  }

  /** Entries that can actually open on this machine. */
  public static function available():Array<ExampleEntry>
    return [for (entry in entries) if (isAvailable(entry)) entry];

  public static function find(id:String):Null<ExampleEntry> {
    for (entry in entries) if (entry.id == id) return entry;
    return null;
  }

  static function isAvailable(entry:ExampleEntry):Bool return switch (entry.kind) {
    case Project(path): FileSystem.exists(path);
    case Script(reference): SetupScriptRegistry.references().indexOf(reference) >= 0;
    case WorkerRackToTable: workerAssetExists("app/examples/worker-rack-to-table.materia");
    case WorkerGallery: workerAssetExists("app/examples/worker-gallery.materia");
  };

  static function workerAssetExists(document:String):Bool {
    try {
      return FileSystem.exists(WorkerAssetPath.resolve(document));
    } catch (_:Dynamic) {
      return false;
    }
  }

  /**
   * Starts opening an example. A project builds on a worker thread, so its job is returned and the caller
   * applies the result with finish() once the job is done; every other kind opens immediately and returns null.
   */
  public static function begin(app:ReferenceEditorApp, entry:ExampleEntry):Null<ProjectLoadJob> {
    switch (entry.kind) {
      case Project(path):
        return new ProjectLoadJob(path);
      default:
        open(app, entry);
        return null;
    }
  }

  /** Replaces the current document with a built project example. */
  public static function finish(app:ReferenceEditorApp, entry:ExampleEntry, generated:GeneratedAssemblyScene):Void {
    var path = switch (entry.kind) {
      case Project(projectPath): projectPath;
      default: throw "Only project examples finish from a build";
    };
    app.session.openGeneratedProject(generated, path);
    app.documentChanged();
    showModel(app);
    app.log((entry.id == LAUNCH_PROJECT_ID ? "Opened project: " : "Opened example: ") + entry.title);
  }

  /** Replaces the current document with the example, blocking until it is built (headless use). */
  public static function open(app:ReferenceEditorApp, entry:ExampleEntry):Void {
    switch (entry.kind) {
      case Project(path):
        finish(app, entry, MateriaProjectRunner.loadProject(path));
        return;
      case Script(reference):
        var scripted = app.session.openScript(reference);
        app.simulation.setBackend(scripted.backend);
        app.simulation.setTimestep(scripted.timestep);
        app.documentChanged();
        showModel(app);
      case WorkerRackToTable:
        app.enableWorkerDemo(0, true);
        app.enterSimulationMode();
        showModel(app);
      case WorkerGallery:
        app.enableWorkerDemo(0, true, ReferenceEditorApp.WORKER_GALLERY);
        app.enterSimulationMode();
        showModel(app);
    }
    app.log("Opened example: " + entry.title);
  }

  static function showModel(app:ReferenceEditorApp):Void {
    if (app.perspectiveViewport == null) return;
    app.workspace.activate("perspective");
    app.scene.select("scene");
    app.perspectiveViewport.frameSelected();
  }
}
