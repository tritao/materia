package machinekit.pendant;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.hmi.PanelSolid;

/**
 * The pendant's circuit board as a component: its envelope, and the terminals the hand wheel is wired to. The board
 * supplies the wheel's 0 V and VCC and reads its two encoder phases.
 */
class PendantBoard extends PanelSolid {
  public function new(build:() -> Part) {
    super("PANEL-BOARD", "pendant board", build, "FR4 PCB");
    for (name in ["vcc", "gnd"]) addPort({
      name: name,
      kind: ElectricalPower,
      role: Supply,
      iface: Plug("screw-terminal", 1),
      required: false
    });
    for (name in ["enc-a", "enc-b"]) addPort({
      name: name,
      kind: Signal,
      role: Consumer,
      iface: Plug("screw-terminal", 1),
      required: true
    });
  }
}
