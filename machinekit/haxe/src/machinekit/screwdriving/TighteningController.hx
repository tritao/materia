package machinekit.screwdriving;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed controller cabinet; program slots carry no invented tightening recipes. */
class TighteningController extends MachineComponent {
	public function new(?programs:Array<Int>, controlInterface:TighteningControlInterface = DiscreteIO) {
		super("CUSTOM-TIGHTENING-CONTROLLER", "Tightening controller cabinet", "steel", true);
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addPort({name: "mains", kind: ElectricalPower, role: Consumer, iface: ScrewdrivingInterfaces.mains(), required: true});
		addPort({name: "toolPower", kind: ElectricalPower, role: Supply, iface: ScrewdrivingInterfaces.tool(), required: false});
		addPort({name: "tool", kind: Signal, role: Supply, iface: ScrewdrivingInterfaces.tool(), required: false});
		addPort({name: "control", kind: Signal, role: Consumer, iface: ScrewdrivingInterfaces.control(controlInterface), required: true});
		addFacet(new TighteningSupplyFacet(programs == null ? [1, 2, 3, 4] : programs, controlInterface));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(250, 200, 400);
}
