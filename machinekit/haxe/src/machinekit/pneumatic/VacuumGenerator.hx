package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;

/** Generic ejector converting compressed air to vacuum. */
class VacuumGenerator extends MachineComponent {
	public function new() {
		super("VACUUM-GENERATOR", "Generic pneumatic vacuum generator", "aluminium 6061", true);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "air", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
		addPort({name: "vacuum", kind: Vacuum, role: Supply, iface: PushIn(6), required: false});
		addConversion("air", "vacuum");
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(18, 14, 30);
}
