package machinekit.pendant;

import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.TypedProperty;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import machinekit.document.MachineKitDocuments;
import machinekit.document.MachineKitRecipes;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactData;

/**
 * The reference pendant as a scene: geometry from the recorded steps, which the source map lists, and an editable recipe
 * for the case's size. `savedDocument` is the recipe as the editor last saved it; its values win over the defaults, so
 * editing the plate's inputs in the Inspector regenerates the whole pendant.
 */
class PendantScene {
  /** The occurrence whose Inspector carries the case's inputs. */
  public static inline var RECIPE_OCCURRENCE:String = "case/plate";

  /**
   * The pendant of a machine: `machine` is the scene's root and the pendant is included in it under `PendantMachine.ACCESSORY` at
   * `pose`, serving `operator` (or, without one, the profile derived from the machine's joints) and jogging the machine's joints
   * (see `PendantMachine.attachedTo`).
   */
  public static function attach(
    root:MachineAssembly,
    pose:AssemblyFrame,
    ?savedDocument:String,
    ?operator:machinekit.operator.OperatorProfile,
    ?axisJoints:Map<String, String>
  ):SceneArtifactData {
    return build(savedDocument, PendantMachine.attachedTo(root, operator, axisJoints), root, pose);
  }

  /** The pendant alone, or (with `root` and `pose`) included in the machine `root`; `machine` says whose pendant it is. */
  public static function build(
    ?savedDocument:String,
    ?machine:PendantMachine,
    ?root:MachineAssembly,
    ?pose:AssemblyFrame
  ):SceneArtifactData {
    var prefix = machine == null ? "" : machine.prefix;
    var recipeOccurrence = prefix + RECIPE_OCCURRENCE;
    MachineKitRecipes.register();
    var recipe = new Document();
    try {
      var definition = MachineKitDocuments.define(recipe, PendantCasePlate.recipeType());
      var instance = recipe.createInstance(recipeOccurrence, definition);
      instance.restoreProperty("machinekit.occurrence", TypedProperty.text("machinekit.occurrence", recipeOccurrence));
      var diagnostics:Array<String> = [];
      if (savedDocument != null) MachineKitRecipes.reconcileDocument(recipe, savedDocument, diagnostics);
      var size:PendantSize = {
        width: instance.resolved("width"),
        height: instance.resolved("height"),
        thickness: instance.resolved("thickness"),
        wall: instance.resolved("wall"),
        depth: instance.resolved("depth")
      };
      var problems = ReferencePendant.layout(size, false, machine).problems();
      if (problems.length > 0) throw "The pendant does not fit its controls: " + problems.join("; ");
      var assembly = ReferencePendant.assembly(ReferencePendant.pendantCase(size, false, machine));
      if (root != null) {
        root.include(PendantMachine.ACCESSORY, assembly, pose);
        assembly = root;
      }
      var scene = AssemblyPreview.scene(assembly, root == null ? "pendant" : "machine-with-pendant");
      scene.presentation = PendantPresentation.build(scene.assemblyDefinition, prefix);
      scene.recipeDocument = DocumentCodec.encode(recipe);
      scene.recipeDiagnostics = diagnostics;
      MachineKitRecipes.forget(recipe);
      recipe.close();
      return scene;
    } catch (error:Dynamic) {
      MachineKitRecipes.forget(recipe);
      recipe.close();
      throw error;
    }
  }
}
