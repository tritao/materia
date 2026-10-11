package machinekit.screwdriving;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed motor/gearbox or slide collar, kept as separate bodies for the real tool joints. */
class ScrewdriverHousing extends MachineComponent {
	public final length:Float;
	public final radius:Float;

	public function new(name:String, length:Float, radius:Float, powered:Bool = false) {
		if (!Math.isFinite(length) || length <= 0 || !Math.isFinite(radius) || radius <= 0)
			throw "Screwdriver body needs finite positive dimensions";
		super("CUSTOM-" + StringTools.replace(name.toUpperCase(), " ", "-"), name, "aluminium 6061", true);
		this.length = length; this.radius = radius;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("end", Mount, Solids.axial(0, 0, length));
		if (powered) addPort({name: "power", kind: ElectricalPower, role: Consumer,
			iface: ScrewdrivingInterfaces.tool(), required: true});
		addPort({name: "control", kind: Signal, role: Consumer, iface: ScrewdrivingInterfaces.tool(), required: true});
		addPort({name: "bitControl", kind: Signal, role: Supply, iface: ScrewdrivingInterfaces.tool(), required: false});
		addPort({name: "vacuum", kind: Vacuum, role: Consumer, iface: ScrewdrivingInterfaces.vacuum(), required: true});
		addPort({name: "bitVacuum", kind: Vacuum, role: Supply, iface: ScrewdrivingInterfaces.vacuum(), required: false});
		addBridge("control", "bitControl"); addBridge("vacuum", "bitVacuum");
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Part.cylinderSpan(radius, 0, length);
}
