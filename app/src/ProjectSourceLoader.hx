package app;

import haxe.io.Bytes;
import sys.FileSystem;
import sys.io.File;
import app.MateriaProjectRunner.ProjectExecutionRequirement;

/** Selects the artifact producer. Shared materialization never interprets source paths. */
class ProjectSourceLoader {
  public static function isPrebuilt(path:String):Bool return StringTools.endsWith(path.toLowerCase(), ".mtrg");

  public static function executionRequirement(path:String, ?variantId:String):ProjectExecutionRequirement {
    if (!isPrebuilt(path)) return MateriaProjectRunner.executionRequirement(path, variantId);
    return {kind: "prebuilt-artifact", projectPath: FileSystem.fullPath(path), entrypoint: "", module: "",
      reconcilesSavedRecipe: false};
  }

  /** Paths have already been checked by SceneArtifactProgram; bound reads before allocating. */
  public static function programReader(artifactPath:String):String->Bytes {
    var root = haxe.io.Path.directory(FileSystem.fullPath(artifactPath));
    return function(relative:String):Bytes {
      var path = root + "/" + relative;
      var metadata = FileSystem.metadata(path);
      if (metadata == null || FileSystem.isDirectory(path) || metadata.size > materia.project.SceneArtifactProgram.MAX_BYTES)
        throw "Machining program is missing or too large: " + relative;
      return File.getBytes(path);
    };
  }

  public static function load(path:String, ?recipeDocument:String,
      ?control:ProjectLoadControl, ?variantId:String):GeneratedAssemblyScene {
    if (!isPrebuilt(path)) return MateriaProjectRunner.loadProject(path, recipeDocument, control, variantId);
    if (recipeDocument != null) throw "A prebuilt project cannot regenerate its geometry; open its source project to edit the recipe";
    if (control != null) { control.throwIfCancelled(); control.phase("Reading the artifact"); }
    var metadata = FileSystem.metadata(path);
    if (metadata == null || FileSystem.isDirectory(path) || metadata.size > 150000000) throw "Project artifact is missing or too large";
    var bytes = File.getBytes(path);
    var generated = new ProjectArtifactLoader(PreparedSceneCaches.current(),programReader(path)).load(bytes, control);
    if (variantId != null && (generated.projectVariant ?? generated.projectJob) != variantId) throw "A prebuilt artifact cannot select a different source project variant";
    return generated;
  }
}
