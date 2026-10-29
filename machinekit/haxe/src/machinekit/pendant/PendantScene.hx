package machinekit.pendant;

import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.TypedProperty;
import machinekit.assembly.AssemblyPreview;
import machinekit.document.MachineKitDocuments;
import machinekit.document.MachineKitRecipes;
import materia.project.SceneArtifact.SceneArtifactData;

/**
 * The reference pendant as a scene: geometry from the recorded steps, which the source map lists, and an editable recipe
 * for the case's size. `savedDocument` is the recipe as the editor last saved it; its values win over the defaults, so
 * editing the plate's inputs in the Inspector regenerates the whole pendant.
 */
class PendantScene {
  /** The occurrence whose Inspector carries the case's inputs. */
  public static inline var RECIPE_OCCURRENCE:String = "case/plate";

  public static function build(?savedDocument:String):SceneArtifactData {
    MachineKitRecipes.register();
    var recipe = new Document();
    try {
      var definition = MachineKitDocuments.define(recipe, PendantCasePlate.recipeType());
      var instance = recipe.createInstance(RECIPE_OCCURRENCE, definition);
      instance.restoreProperty("machinekit.occurrence", TypedProperty.text("machinekit.occurrence", RECIPE_OCCURRENCE));
      var diagnostics:Array<String> = [];
      if (savedDocument != null) MachineKitRecipes.reconcileDocument(recipe, savedDocument, diagnostics);
      var size:PendantSize = {
        width: instance.resolved("width"),
        height: instance.resolved("height"),
        thickness: instance.resolved("thickness"),
        wall: instance.resolved("wall"),
        depth: instance.resolved("depth")
      };
      var problems = ReferencePendant.layout(size).problems();
      if (problems.length > 0) throw "The pendant does not fit its controls: " + problems.join("; ");
      var documents = new PendantDocuments(ReferencePendant.pendantCase(size));
      try {
        var scene = AssemblyPreview.scene(ReferencePendant.assembly(documents), "pendant");
        scene.presentation = PendantPresentation.build(scene.assemblyDefinition);
        scene.project = {dynamicParts: [], motions: [], sources: documents.sources(), histories: documents.histories()};
        scene.recipeDocument = DocumentCodec.encode(recipe);
        scene.recipeDiagnostics = diagnostics;
        documents.close();
        MachineKitRecipes.forget(recipe);
        recipe.close();
        return scene;
      } catch (error:Dynamic) {
        documents.close();
        throw error;
      }
    } catch (error:Dynamic) {
      MachineKitRecipes.forget(recipe);
      recipe.close();
      throw error;
    }
  }
}
