package machinekit.hmi;

import machinekit.component.MachineComponent;

/** A further fixed component of a panel, in panel coordinates: a board behind the plate, say. */
class PanelExtra {
  public final id:String;
  public final component:MachineComponent;

  public function new(id:String, component:MachineComponent) {
    this.id = id;
    this.component = component;
  }
}
