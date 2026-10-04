package machinekit.welding;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** A welding power source: the processes it runs, its rated current in amperes, how a controller drives it, and its efficiency (the fraction of the mains power it delivers to the arc). */
class WeldingSupplyFacet implements ComponentFacet {
	public final processes:Array<WeldingProcess>;
	public final maxCurrentA:Float;
	public final controlInterface:WeldingControlInterface;
	public final efficiency:Float;

	public function new(processes:Array<WeldingProcess>, maxCurrentA:Float, controlInterface:WeldingControlInterface, efficiency:Float) {
		this.processes = processes;
		this.maxCurrentA = maxCurrentA;
		this.controlInterface = controlInterface;
		this.efficiency = efficiency;
	}

	public function check(component:MachineComponent):Void {
		if (processes == null || processes.length == 0 || !(maxCurrentA > 0))
			throw 'Welding supply on "${component.designation}" needs a process and a positive rated current';
		if (!(efficiency > 0 && efficiency <= 1))
			throw 'Welding supply on "${component.designation}" needs an efficiency in (0, 1]';
	}

	public function describe():String return 'welding supply ${processes.join("/")} $maxCurrentA A $controlInterface $efficiency';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<WeldingSupplyFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, WeldingSupplyFacet)) return cast facet;
		return null;
	}
}
