package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;

/** Generic inline vacuum sensor with a pressure feedback signal. */
class VacuumPressureSensor extends MachineComponent {

	public function new(tubeOdMm:Float = 4) {
		if (!Math.isFinite(tubeOdMm) || tubeOdMm <= 0)
			throw "Vacuum pressure sensor needs a positive tube diameter";
		super('VACUUM-PRESSURE-SENSOR-${tubeOdMm}',
			"Generic inline vacuum pressure sensor", "plastic", true);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "vacuumIn", kind: Vacuum, role: Consumer,
			iface: PushIn(tubeOdMm), required: true});
		addPort({name: "vacuumOut", kind: Vacuum, role: Supply,
			iface: PushIn(tubeOdMm), required: false});
		addBridge("vacuumIn", "vacuumOut");
		addPort({name: "pressureSignal", kind: Signal, role: Supply,
			iface: Plug("analog-vacuum-kpa", 3), required: false});
		addCapability(VacuumPressureSensor("vacuumIn", "pressureSignal"));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(20, 15, 25);
}
