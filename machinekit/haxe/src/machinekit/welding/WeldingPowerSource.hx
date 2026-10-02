package machinekit.welding;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Single-phase MIG/MAG power source for robot welding. It runs from 230 V mains, passes the
 * shielding gas through to the feeder, and takes its trigger and setpoints from the robot's
 * controller.
 *
 * CAD frame: a box centred over its origin in X and Y, standing on its base at z=0, front toward
 * +Y. Connector: `base` (+Y along +Z). Ports: the `mains` inlet, the `gas` inlet and `control`
 * inlet, the weld output sockets `weldPositive` (to the feeder) and `weldNegative` (the work
 * lead), and the pass-throughs `gasOut` and `feederControl`.
 */
class WeldingPowerSource extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public final maxCurrentA:Float;

	public function new(maxCurrentA:Float = 350, width:Float = 600, depth:Float = 300, height:Float = 450) {
		if (!(maxCurrentA > 0) || !(width > 0) || !(depth > 0) || !(height > 0))
			throw "Welding power source needs a positive rating and dimensions";
		super('WELD-SOURCE-${Dimension.format(maxCurrentA)}A-230V',
			'MIG/MAG power source, ${Dimension.format(maxCurrentA)} A, 230 V single-phase', "painted steel", true);
		this.maxCurrentA = maxCurrentA;
		this.width = width;
		this.depth = depth;
		this.height = height;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addPort({name: "mains", kind: ElectricalPower, role: Consumer, iface: WeldingInterfaces.mains(), required: true});
		addPort({name: "gas", kind: Gas, role: Consumer, iface: WeldingInterfaces.gas(), required: true});
		addPort({name: "control", kind: Signal, role: Consumer, iface: WeldingInterfaces.control(), required: true});
		addPort({name: "weldPositive", kind: ElectricalPower, role: Supply, iface: WeldingInterfaces.weldCable(), required: false});
		addPort({name: "weldNegative", kind: ElectricalPower, role: Supply, iface: WeldingInterfaces.weldCable(), required: false});
		addPort({name: "gasOut", kind: Gas, role: Supply, iface: WeldingInterfaces.gas(), required: false});
		addPort({name: "feederControl", kind: Signal, role: Supply, iface: WeldingInterfaces.control(), required: false});
		addBridge("gas", "gasOut");
		addBridge("control", "feederControl");
		addCapability(WeldingSupply([Mig, Mag], maxCurrentA, AnalogIo));
		declareMass(55, new Vector(0, 0, 0.4 * height));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(width, depth, height);
}
