package machinekit.screwdriving;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** Available program slots; process parameters are supplied by the joint later. */
class TighteningSupplyFacet implements ComponentFacet {
	public final programs:Array<Int>;
	public final controlInterface:TighteningControlInterface;

	public function new(programs:Array<Int>, controlInterface:TighteningControlInterface) {
		if (programs == null) throw "Tightening supply needs programs";
		this.programs = programs.copy(); this.controlInterface = controlInterface;
	}
	public function check(component:MachineComponent):Void {
		if (programs.length == 0 || controlInterface == null) throw 'Tightening supply "${component.designation}" needs programs and an interface';
		var seen:Map<Int, Bool> = [];
		for (program in programs) {
			if (program <= 0 || seen.exists(program)) throw 'Tightening supply "${component.designation}" has an invalid or duplicate program $program';
			seen.set(program, true);
		}
	}
	public function describe():String return 'tightening supply ${programs.join("/")} $controlInterface';

	public static function of(component:MachineComponent):Null<TighteningSupplyFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, TighteningSupplyFacet)) return cast facet;
		return null;
	}
}
