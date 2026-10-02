package machinekit.welding;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Wire feeder for a robot torch: a closed box with the wire spool and drive rolls inside. It
 * takes weld current, shielding gas and control from the power source and passes them, with the
 * wire, on to the torch.
 *
 * CAD frame: a box centred over its origin in X and Y, standing on its base at z=0. Connector:
 * `mount` (the base, +Y along +Z). Ports: `power`, `gas` and `control` inlets, and the supplies
 * `torchPower`, `torchGas`, `torchControl` and `torchWire`.
 */
class WireFeeder extends MachineComponent {
	public final width:Float;
	public final length:Float;
	public final height:Float;
	/** The wire it feeds, in millimetres, and its top speed in metres per minute. */
	public final wireDiameterMm:Float;
	public final maxSpeedMPerMin:Float;

	public function new(width:Float = 160, length:Float = 400, height:Float = 250, wireDiameterMm:Float = 1.2,
			maxSpeedMPerMin:Float = 20) {
		if (!(width > 0) || !(length > 0) || !(height > 0)) throw "Wire feeder needs positive dimensions";
		if (!(wireDiameterMm > 0) || !(maxSpeedMPerMin > 0)) throw "Wire feeder needs a positive wire diameter and speed";
		super('WIRE-FEEDER-${Dimension.format(width)}x${Dimension.format(length)}x${Dimension.format(height)}',
			"Wire feeder for robot MIG torch, 4-roll drive", "painted steel", true);
		this.width = width;
		this.length = length;
		this.height = height;
		this.wireDiameterMm = wireDiameterMm;
		this.maxSpeedMPerMin = maxSpeedMPerMin;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "power", kind: ElectricalPower, role: Consumer, iface: WeldingInterfaces.weldCable(), required: true});
		addPort({name: "gas", kind: Gas, role: Consumer, iface: WeldingInterfaces.gas(), required: true});
		addPort({name: "control", kind: Signal, role: Consumer, iface: WeldingInterfaces.control(), required: true});
		addPort({name: "torchPower", kind: ElectricalPower, role: Supply, iface: WeldingInterfaces.weldCable(), required: false});
		addPort({name: "torchGas", kind: Gas, role: Supply, iface: WeldingInterfaces.gas(), required: false});
		addPort({name: "torchControl", kind: Signal, role: Supply, iface: WeldingInterfaces.control(), required: false});
		addPort({name: "torchWire", kind: Wire, role: Supply, iface: WeldingInterfaces.wireLiner(), required: false});
		addBridge("power", "torchPower");
		addBridge("gas", "torchGas");
		addBridge("control", "torchControl");
		addCapability(WireFeed(wireDiameterMm, maxSpeedMPerMin));
		declareMass(14, new Vector(0, 0, height / 2));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(width, length, height);
}
