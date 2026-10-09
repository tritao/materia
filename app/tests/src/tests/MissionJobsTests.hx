package tests;

import app.ProjectJobs;
import app.ProjectSourceLoader;
import app.GeneratedAssemblyScene;
import app.Main.ReferenceEditorApp;
import app.SceneCodec;
import app.editor.ExampleCatalog;
import app.editor.MissionPanel;
import haxe.Json;
import sys.io.File;
import sys.FileSystem;
import haxeon.ui.LayoutFrame;
import haxeon.ui.core.RenderNode;
import haxeon.ui.widgets.text.Text;

@:access(app.Main.ReferenceEditorApp)
class MissionJobsTests {
  static function check(value:Bool, message:String):Void { if (!value) throw message; }
  static function refuses(action:Void->Void):Void {
    var failed = false;
    try action() catch (_:Dynamic) failed = true;
    check(failed, "invalid job metadata is rejected");
  }

  public static function main():Int {
    try { run(); Sys.println("Mission jobs: presets, persistence, shared views, failure and simulation lock passed"); return 0; }
    catch (error:Dynamic) { Sys.println("Mission jobs failed: " + error); return 1; }
  }

  static function run():Void {
    var mainFile = FileSystem.fullPath(app.editor.WorkerAssetPath.resolve("app/src/Main.hx"));
    Sys.setCwd(haxe.io.Path.directory(haxe.io.Path.directory(haxe.io.Path.directory(mainFile))));
    var manifest = app.editor.WorkerAssetPath.resolve("machinekit/examples/robot-welder/materia.project.json");
    var root:Dynamic = Json.parse(File.getContent(manifest));
    var jobs = ProjectJobs.decode(root);
    check(jobs.length == 5, "one welding project exposes five jobs");
    check(ProjectJobs.entrypoint(root, null) == "cell", "default job preserves the full weldment");
    for (id in ["robot-welder", "robot-welder-seam", "robot-welder-post", "robot-welder-weave", "robot-welder-multipass"]) {
      var entry:app.editor.ExampleCatalog.ExampleEntry = cast ExampleCatalog.find(id);
      check(entry != null && entry.jobId != null, "Start preset identifies a job");
      var source = switch entry.kind { case Project(path): path; default: throw "Not a project"; };
      check(source == "machinekit/examples/robot-welder/materia.project.json", "presets share one source");
      var requirement = ProjectSourceLoader.executionRequirement(source, entry.jobId);
      check(requirement.entrypoint == ProjectJobs.entrypoint(root, entry.jobId), "loader selects the requested entrypoint");
    }
    refuses(function() ProjectJobs.selected(root, "missing"));
    var duplicate:Dynamic = Json.parse(Json.stringify(root));
    var duplicateJobs:Array<Dynamic> = duplicate.jobs;
    duplicateJobs.push(duplicateJobs[0]);
    refuses(function() ProjectJobs.decode(duplicate));
    check(ProjectJobs.decode({}).length == 0, "existing projects need no jobs metadata");
    check(ProjectJobs.entrypoint({defaultEntrypoint:"legacy"}, null) == "legacy", "legacy entrypoints still work");

    var directory = "build/mission-jobs-" + Std.random(100000000);
    FileSystem.createDirectory(directory);
    // A source that can be inspected but cannot build exercises replacement failure without invoking a compiler.
    Reflect.setField(root, "build", {system:"haxeon", manifest:"missing-haxeon.json"});
    var source = directory + "/materia.project.json";
    File.saveContent(source, Json.stringify(root));
    var fonts = haxeon.ui.FontCollection.create();
    fonts.add("haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf");
    var editor = new ReferenceEditorApp(fonts, directory + "/workspace.json", null, null, null, null, null, null, true);
    var generated:GeneratedAssemblyScene = {objects:editor.scene.records(), geometryBySnapshot:new Map(),
      assemblyDefinition:null, assemblyState:null, localCentersByDefinition:new Map(), metresPerUnit:1.0,
      physical:null, recipeDocument:null, projectJob:"single-seam", mission:null};
    editor.session.openGeneratedProject(generated, source);
    editor.documentChanged();
    var controller = editor.missionController;
    check(controller.snapshot().selectedJob == "single-seam", "controller reads installed selection");
    check(controller.snapshot().jobs.length == 5, "controller exposes project jobs");
    var saved = directory + "/selected.materia";
    editor.session.save(saved);
    var record = SceneCodec.decodeProjectRoot(Json.parse(File.getContent(saved)));
    check(record != null && record.jobId == "single-seam", "saved documents retain job selection");
    var frame = new LayoutFrame(800, 600);
    var panel = editor.ui.submit(new MissionPanel().build(controller, editor.appearance.theme.tokens), frame);
    check(findLabel(panel, "Mission job") != null, "panel contains shared job selector");
    var overlay = editor.ui.submit(editor.viewportWithControls(new Text("Scene"), 800), frame);
    check(findLabel(overlay, "Mission job") != null && find(overlay, "viewport-mission-steps") != null,
      "viewport overlay exposes the same selector and details action");

    var originalDocument = editor.session.document;
    controller.selectJob("multipass");
    check(controller.isLoading() && !controller.snapshot().switchAllowed, "job preparation locks switching");
    check(editor.session.document == originalDocument && editor.session.projectJob == "single-seam", "loading retains the installed scene");
    var deadline = Sys.time() + 10.0;
    while (controller.isLoading() && Sys.time() < deadline) { controller.tick(); Sys.sleep(0.005); }
    check(!controller.isLoading() && controller.snapshot().error != null, "failed preparation is reported");
    check(editor.session.document == originalDocument && editor.session.projectJob == "single-seam", "failed job preserves document and selection");
    check(editor.simulation.rebuild(editor.sensors, editor.scene, editor.session), "simulation fixture builds");
    check(!controller.snapshot().switchAllowed, "paused simulation locks job switching");
    controller.selectJob("complete");
    check(!controller.isLoading(), "locked selection starts no build");
    editor.simulation.clear();
    controller.selectJob("complete"); controller.cancel(); controller.tick();
    check(!controller.isLoading() && editor.session.document == originalDocument, "cancelled job cannot replace the scene");
    editor.dispose(); fonts.dispose();
    FileSystem.deleteFile(saved); FileSystem.deleteFile(source);
    if (Sys.getEnv("MISSION_JOBS_INTEGRATION") == "1") integration(manifest, directory);
  }

  static function integration(manifest:String, directory:String):Void {
    var editor = new ReferenceEditorApp(null, directory + "/integration-workspace.json");
    var first = ProjectSourceLoader.load(manifest, null, null, "single-seam");
    editor.session.openGeneratedProject(first, manifest); editor.documentChanged();
    check(editor.session.projectJob == "single-seam" && editor.session.mission != null, "real seam job installs");
    var original = editor.session.document;
    var originalMission = Json.stringify(editor.session.mission);
    editor.missionController.selectJob("multipass");
    check(editor.session.document == original, "real job retains current document during preparation");
    var deadline = Sys.time() + 240.0;
    while (editor.missionController.isLoading() && Sys.time() < deadline) { editor.tick(); Sys.sleep(0.01); }
    check(!editor.missionController.isLoading() && editor.missionController.snapshot().error == null,
      "real job replacement succeeds: " + editor.missionController.snapshot().error);
    check(editor.session.projectJob == "multipass" && editor.session.document != original &&
      Json.stringify(editor.session.mission) != originalMission, "replacement installs selected process and mission");
    var file = directory + "/real-job.materia";
    editor.session.save(file);
    editor.session.newDocument();
    editor.session.open(file); editor.documentChanged();
    check(editor.session.projectJob == "multipass" && editor.session.mission != null,
      "reopening saved document regenerates the selected job");
    editor.dispose(); FileSystem.deleteFile(file);
    Sys.println("Real welding job switch and saved-document reopen passed");
  }

  static function findLabel(node:RenderNode, label:String):Null<RenderNode> {
    if (node.semantics != null && node.semantics.label == label) return node;
    for (child in node.children) { var result = findLabel(child, label); if (result != null) return result; }
    return null;
  }

  static function find(node:RenderNode, key:String):Null<RenderNode> {
    if (node.styleKey == key) return node;
    for (child in node.children) { var result = find(child, key); if (result != null) return result; }
    return null;
  }
}
