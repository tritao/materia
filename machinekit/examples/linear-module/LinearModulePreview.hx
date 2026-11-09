import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.hobbing.HobberYLayout;
import machinekit.hobbing.HobberYPart;
import machinekit.motion.ScrewLinearModule;
import machinekit.motion.ScrewLinearModuleSpec;
import materia.project.SceneArtifact;

/**
 * The screw linear module on its own, in its own frame: X along the travel, Y across the guides, Z up off the ground base,
 * so the carriage plate faces up. `standard` is the library default; `hobberY` is the hobber's Y module as fitted (its spec,
 * host base, carriage and guards), laid flat instead of hung on the swivel.
 */
class LinearModulePreview {
  public static function standard():Bytes return SceneArtifact.encode(AssemblyPreview.scene(new ScrewLinearModule(new ScrewLinearModuleSpec()), "Screw linear module"));

  public static function hobberY():Bytes {
    var guards = new Map<String, MachineComponent>();
    for (role in ["coverUpper", "coverLower", "retainerUpper", "retainerLower", "lipUpper", "lipLower"])
      guards.set(role, new HobberYPart(new ComponentValues().setToken("role", role)));
    var module = new ScrewLinearModule(HobberYLayout.moduleSpec(), new HobberYPart(new ComponentValues().setToken("role", "base")),
      new HobberYPart(new ComponentValues().setToken("role", "carriage")), guards);
    return SceneArtifact.encode(AssemblyPreview.scene(module, "Hobber Y module"));
  }

  public static function main():Void {
    standard();
    hobberY();
    Sys.println("Linear module scenes built");
  }
}
