package machinekit.hmi;

import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import materia.assembly.AssemblyFrames;

/**
 * A panel as a MachineAssembly: the plate (and an optional shell) at the origin and every part at its
 * position on the panel's front face. Check it with `AssemblyIntegrity.check`, which reports any two members
 * that overlap.
 */
class PanelAssemblyBuilder {
  public static function build(layout:PanelLayout, plate:() -> Part, ?shell:() -> Part):MachineAssembly {
    var assembly = new MachineAssembly();
    assembly.addComponent("plate", new PanelSolid("PANEL-PLATE", "panel plate", plate));
    if (shell != null) assembly.addComponent("shell", new PanelSolid("PANEL-SHELL", "case shell", shell));
    for (c in layout.controls) assembly.addComponent(c.id, c.part, AssemblyFrames.translation(c.x, c.y, layout.thickness));
    return assembly;
  }
}
