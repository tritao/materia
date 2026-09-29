package app.editor;

import app.MateriaProjectRunner;
import app.Main.ReferenceEditorApp;
import app.SetupScriptRegistry;
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
      tag: "Project · builds in ~30 s",
      kind: Project("machinekit/examples/picking-station/materia.project.json")},
    {id: "motor-shaft-bearings", title: "Motor, shaft and bearings",
      description: ["Standard parts assembled", "from MachineKit generators"],
      tag: "Project · builds in ~30 s",
      kind: Project("machinekit/examples/materia.project.json")},
    {id: "cad-modeling", title: "CAD modelling",
      description: ["Parametric sketches, extrusions", "and features in CadKit"],
      tag: "Project · builds in ~15 s",
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

  /** Replaces the current document with the example. Building a project can take tens of seconds. */
  public static function open(app:ReferenceEditorApp, entry:ExampleEntry):Void {
    switch (entry.kind) {
      case Project(path):
        var generated = MateriaProjectRunner.loadProject(path);
        app.session.openGeneratedScene(generated.objects, path, generated.assembly,
          generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
          generated.localCentersByDefinition, generated.metresPerUnit,
          generated.physical, generated.recipeDocument);
        app.documentChanged();
        showModel(app);
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
