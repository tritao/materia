package machinekit.control;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** A physical cabinet with individually wired drive and digital I/O terminals. */
class ControllerCabinet extends MachineComponent {
	public final kind:String;
	public final axes:Int;
	public final inputs:Int;
	public final outputs:Int;
	public final valves:Int;

	function new(kind:String, axes:Int, inputs:Int, outputs:Int, valves:Int) {
		if (axes < 0 || axes > 16 || inputs < 0 || inputs > 64 || outputs < 0 || outputs > 64 ||
			valves < 0 || valves > 32) throw "Controller I/O counts are out of range";
		super("CONTROLLER-" + kind.toUpperCase() + '-$axes-$inputs-$outputs-$valves',
			'$kind controller cabinet', "aluminium 6061");
		this.kind = kind;
		this.axes = axes;
		this.inputs = inputs;
		this.outputs = outputs;
		this.valves = valves;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		for (i in 0...axes) addPort({name: 'axis${i + 1}', kind: Signal, role: Supply,
			iface: Unspecified, required: false});
		for (i in 0...inputs) addPort({name: 'input${i + 1}', kind: Signal, role: Consumer,
			iface: Unspecified, required: false});
		for (i in 0...outputs) addPort({name: 'output${i + 1}', kind: Signal, role: Supply,
			iface: Unspecified, required: false});
		for (i in 0...valves) addPort({name: 'valve${i + 1}', kind: Signal, role: Supply,
			iface: Unspecified, required: false});
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(180, 180, 280), "controller-module");
}
