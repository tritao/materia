package app;

import haxe.io.Bytes;
import sys.FileSystem;
import sys.io.File;
import app.MateriaProjectRunner.ProjectExecutionRequirement;

/** Selects the artifact producer. Shared materialization never interprets source paths. */
class ProjectSourceLoader {
  public static function isPrebuilt(path:String):Bool return StringTools.endsWith(path.toLowerCase(), ".mtrg");

  public static function executionRequirement(path:String, ?jobId:String):ProjectExecutionRequirement {
    if (!isPrebuilt(path)) return MateriaProjectRunner.executionRequirement(path, jobId);
    return {kind: "prebuilt-artifact", projectPath: FileSystem.fullPath(path), entrypoint: "", module: "",
      reconcilesSavedRecipe: false};
  }

  public static function load(path:String, ?recipeDocument:String,
      ?control:ProjectLoadControl, ?jobId:String):GeneratedAssemblyScene {
    if (!isPrebuilt(path)) return MateriaProjectRunner.loadProject(path, recipeDocument, control, jobId);
    if (recipeDocument != null) throw "A prebuilt project cannot regenerate its geometry; open its source project to edit the recipe";
    if (control != null) { control.throwIfCancelled(); control.phase("Reading the artifact"); }
    var metadata = FileSystem.metadata(path);
    if (metadata == null || FileSystem.isDirectory(path) || metadata.size > 150000000) throw "Project artifact is missing or too large";
    var bytes = File.getBytes(path);
    var generated = new ProjectArtifactLoader(PreparedSceneCaches.current()).load(bytes, control);
    if (jobId != null && generated.projectJob != jobId) throw "A prebuilt artifact cannot select a different source project job";
    return generated;
  }
}
