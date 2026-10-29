package tests;

import app.EditorSceneTree;
import materia.project.SceneArtifact;

/** Every recorded pendant step is a row in the real tree, and selecting it lights exactly the faces it created. */
@:access(app.EditorScene) class PendantStepLinkTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }
  public static function main():Int {
    run();
    return 0;
  }
  public static function run():Void {
    var bytes = SceneArtifact.encode(machinekit.pendant.PendantScene.build());
    var session = new app.ProjectDocumentSession(null, false);
    session.openGeneratedProject(new app.ProjectArtifactLoader().load(bytes), "/tmp/pendant-steps.mtrg");
    var scene = session.scene, tree = new EditorSceneTree(scene, session.projectAssemblyDefinition);
    var map = scene.sourceMap;
    if (map == null) throw "The pendant scene has no source map";
    var before = haxe.Json.stringify(scene.records()), history = scene.document.history.undoCount;
    try {
      var perPart = new Map<String, Int>();
      for (feature in map.features) perPart.set(feature.occurrence, (perPart.exists(feature.occurrence) ? perPart.get(feature.occurrence) : 0) + 1);
      for (occurrence in perPart.keys()) {
        var rows = 0, key = "project:" + occurrence;
        for (index in 0...tree.childCount(key)) if (StringTools.startsWith(tree.childKeyAt(key, index), "source:")) rows++;
        check(rows == perPart.get(occurrence), occurrence + ": " + rows + " step rows for " + perPart.get(occurrence) + " steps");
      }
      var lit = 0;
      for (feature in map.features) {
        tree.selectKey("source:" + feature.id);
        check(tree.selectionKey() == "source:" + feature.id, "Selecting " + feature.label + " keeps the step selected");
        scene.renderSnapshot();
        var active = scene.presentation.sourceFaces.active.length;
        check(active == feature.faces.length, feature.label + ": " + active + " faces lit for " + feature.faces.length + " declared");
        var shown = new Map<Int, Bool>();
        for (entry in scene.presentation.sourceFaces.entries) if (entry.face >= 0) shown.set(entry.face, true);
        for (face in feature.faces) check(shown.exists(face), feature.label + ": face " + face + " is not highlighted");
        lit += active;
      }
      check(lit > 0, "Steps light faces");
      check(
        haxe.Json.stringify(scene.records()) == before && scene.document.history.undoCount == history,
        "Selecting steps leaves engineering and history unchanged"
      );
      Sys.println("Pendant steps: " + map.features.length + " steps, " + lit + " faces lit");
    } catch (error:Dynamic) {
      session.dispose();
      throw error;
    }
    session.dispose();
    Sys.println("Pendant step links: " + checks + " checks passed");
  }
}
