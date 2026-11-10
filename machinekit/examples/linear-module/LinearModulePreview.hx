import hobber.*;
import hobber.HobberLayout;
import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.component.ComponentValues;
import hobber.HobberYPart;
import machinekit.motion.ScrewLinearModule;
import machinekit.motion.ScrewLinearModuleSpec;
import materia.project.SceneArtifact;

/**
 * The v2 open screw linear module on its own, in its own frame: X along the travel, Y across the guides, Z up off the ground base,
 * so the carriage plate faces up. `standard` is the library default; `hobberY` is the hobber's Y module as fitted (its spec,
 * host base and carriage; optional covers omitted), laid flat instead of hung on the swivel.
 */
class LinearModulePreview {
  static function preview(module:ScrewLinearModule, title:String):Bytes {
    // Derive the internal module joints without the host's exposed spindle/
    // actuator interfaces, whose mating parts are absent in this standalone view.
    var internal=new ScrewLinearModule(module.spec);
    var hardware=internal.deriveFasteners();
    for(member in module.components()) {
      var source=internal.component(member.id);
      var cuts=source.machinedCuts();
      if(cuts.length>0)member.component.machine(cuts);
    }
    for(line in hardware.lines())module.addBomItem(line.item,line.quantity);
    return SceneArtifact.encode(AssemblyPreview.scene(module, title));
  }

  public static function standard():Bytes return preview(new ScrewLinearModule(new ScrewLinearModuleSpec()), "Screw linear module v2 (20 mm)");

  public static function hobberY():Bytes {
    HobberComponents.install();
    var module = new ScrewLinearModule(HobberLayout.get().moduleSpec(), new HobberYPart(new ComponentValues().setToken("role", "base")),
      new HobberYPart(new ComponentValues().setToken("role", "carriage")));
    return preview(module, "Hobber Y module v2 (30 mm)");
  }

  public static function main():Void {
    standard();
    hobberY();
    Sys.println("Linear module scenes built");
  }
}
