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
    {id: "cnc-router", title: "Desktop CNC router",
      description: ["A generated three-axis gantry", "router with stock on its bed"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/cnc-router/materia.project.json")},
    {id: "mobile-base", title: "Mobile base",
      description: ["A generated differential-drive", "robot base with a lidar"],
      tag: "Project · first build ~30 s",
      kind: Project("machinekit/examples/mobile-base/materia.project.json")},
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
      kind: WorkerRackToTable}
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
    case WorkerRackToTable: workerAssetExists();
  };

  static function workerAssetExists():Bool {
    try {
      return FileSystem.exists(WorkerAssetPath.resolve("app/examples/worker-rack-to-table.materia"));
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
