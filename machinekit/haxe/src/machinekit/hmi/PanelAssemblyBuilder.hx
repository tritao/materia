package machinekit.hmi;

import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * A panel as a MachineAssembly: the plate (and an optional shell) at the origin and every part at its
 * position on the panel's front face. Check it with `AssemblyIntegrity.check`, which reports any two members
 * that overlap.
 */
class PanelAssemblyBuilder {
  /**
	 * `extras` are further fixed solids in panel coordinates (a board, say), each with an id and a builder, made of
	 * `extrasMaterial`. The plate and shell are printed nylon, light against the dark controls. When there is a shell or
	 * extras the plate, shell and extras sit in a `case` group; each of `groups` gathers controls by id, and a control in
	 * no group stays at the top level. `hardware` (inserts, screws) goes in a `fasteners` group.
	 */
  public static function build(
    layout:PanelLayout,
    plate:() -> Part,
    ?shell:() -> Part,
    ?extras:Array<{id:String, solid:() -> Part}>,
    extrasMaterial:String = "plastic",
    ?groups:Array<PanelGroup>,
    ?hardware:Array<PanelHardware>
  ):MachineAssembly {
    var assembly = new MachineAssembly();
    var housing = shell != null || (extras != null && extras.length > 0) ? new MachineAssembly() : assembly;
    housing.addComponent("plate", new PanelSolid("PANEL-PLATE", "panel plate", plate, "PA12 nylon"));
    if (shell != null) housing.addComponent("shell", new PanelSolid("PANEL-SHELL", "case shell", shell, "PA12 nylon"));
    if (extras != null) for (e in extras) housing.addComponent(
      e.id,
      new PanelSolid("PANEL-" + e.id.toUpperCase(), e.id, e.solid, extrasMaterial)
    );
    if (housing != assembly) assembly.include("case", housing);
    // An included assembly is copied when it is included, so fill each group before including it.
    var groupOf:Map<String, MachineAssembly> = [];
    var grouped:Array<{id:String, assembly:MachineAssembly}> = [];
    if (groups != null) for (g in groups) {
      var members = new MachineAssembly();
      for (id in g.members) groupOf.set(id, members);
      grouped.push({id: g.id, assembly: members});
    }
    var partHardware:Array<PanelHardware> = [];
    for (c in layout.controls) {
      var target = groupOf.exists(c.id) ? groupOf.get(c.id) : assembly;
      var pose = AssemblyFrames.translation(c.x, c.y, layout.thickness);
      // A device made of several finished bodies goes in as a subassembly of them; a one-solid part goes in as itself.
      var device:Null<HmiPart> = Std.isOfType(c.part, HmiPart) ? cast c.part : null;
      var expanded = device == null ? null : device.expand();
      if (device != null) for (h in device.hardware()) partHardware.push(h.placed(c.id + "-" + h.id, c.x, c.y, layout.thickness));
      if (expanded == null) target.addComponent(c.id, c.part, pose);
      else target.include(c.id, expanded, pose);
    }
    for (g in grouped) assembly.include(g.id, g.assembly);
    var allHardware = (hardware == null ? [] : hardware).concat(partHardware);
    if (allHardware.length > 0) {
      var fasteners = new MachineAssembly();
      for (h in allHardware) fasteners.addComponent(h.id, h.component, pose(h));
      assembly.include("fasteners", fasteners);
    }
    return assembly;
  }

  /** A hardware item's pose: its position, turned over about x when it is flipped. */
  static function pose(h:PanelHardware):AssemblyFrame {
    var at = AssemblyFrames.translation(h.x, h.y, h.z);
    return h.flipped ? AssemblyFrames.compose(at, AssemblyFrames.turnX(Math.PI)) : at;
  }
}
