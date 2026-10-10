package app;

#if wasm
import materia.project.SceneArtifact;

/** Isolated preparation entry: bytes in/out, no editor, storage or native handles. */
class PreparationWorkerMain {
  @:expose public static function main():Int {
    try {
      MateriaPreparation.materia_prepare_phase("Decoding the artifact");
      var input = MateriaPreparation.materia_prepare_input();
      if (input.status != 0 || input.data.length > 150000000) throw "Invalid worker input";
      var artifact = SceneArtifact.decodeView(input.data);
      var target = artifact.machining == null ? null : artifact.machining.target;
      var components = [for (part in artifact.parts) if (part.id != target) part];
      MateriaPreparation.materia_prepare_phase("Preparing geometry and physics");
      var parts = ProjectScenePreparation.prepare(components, artifact.metresPerUnit);
      var bytes = PreparedSceneCodec.encode(parts);
      MateriaPreparation.materia_prepare_complete(bytes);
      return 0;
    } catch (error:Dynamic) {
      MateriaPreparation.materia_prepare_error(Std.string(error));
      return 1;
    }
  }
}
#end
