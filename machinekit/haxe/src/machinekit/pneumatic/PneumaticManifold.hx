package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;

/** Generic air manifold with one required input and N bridged outlets. */
class PneumaticManifold extends MachineComponent {
	public final outlets:Int;

	public function new(outlets:Int) {
		if (outlets < 1) throw "Pneumatic manifold needs at least one outlet";
		super('MANIFOLD-$outlets', 'Generic $outlets-outlet pneumatic manifold', "aluminium 6061", true);
		this.outlets = outlets;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "input", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
		for (i in 1...outlets + 1) {
			addPort({name: 'out$i', kind: Pneumatic, role: Supply, iface: PushIn(6), required: false});
			addBridge("input", 'out$i');
		}
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(20, 10, 12 + 8 * outlets);
}
