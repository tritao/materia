package machinekit.welding;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** A wire feeder: the diameter of the wire it feeds in millimetres, its top wire speed in metres per minute, and the fraction of the wire that ends up in the weld. */
class WireFeedFacet implements ComponentFacet {
	public final wireDiameterMm:Float;
	public final maxSpeedMPerMin:Float;
	public final depositionEfficiency:Float;

	public function new(wireDiameterMm:Float, maxSpeedMPerMin:Float, depositionEfficiency:Float) {
		this.wireDiameterMm = wireDiameterMm;
		this.maxSpeedMPerMin = maxSpeedMPerMin;
		this.depositionEfficiency = depositionEfficiency;
	}

	public function check(component:MachineComponent):Void {
		if (!(wireDiameterMm > 0) || !(maxSpeedMPerMin > 0))
			throw 'Wire feed on "${component.designation}" needs a positive wire diameter and speed';
		if (!(depositionEfficiency > 0 && depositionEfficiency <= 1))
			throw 'Wire feed on "${component.designation}" needs a deposition efficiency in (0, 1]';
	}

	public function describe():String return 'wire feed $wireDiameterMm mm $maxSpeedMPerMin m/min $depositionEfficiency';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<WireFeedFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, WireFeedFacet)) return cast facet;
		return null;
	}
}
