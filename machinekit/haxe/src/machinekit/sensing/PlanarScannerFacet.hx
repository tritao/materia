package machinekit.sensing;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** A planar scanner: it sweeps `rayCount` rays round the horizontal plane of its connector `scanConnector` (zero bearing along the connector's +X), seeing out to `maxRangeMeters`, and scans `rateHz` times a second. */
class PlanarScannerFacet implements ComponentFacet {
	public final scanConnector:String;
	public final rayCount:Int;
	public final maxRangeMeters:Float;
	public final rateHz:Float;

	public function new(scanConnector:String, rayCount:Int, maxRangeMeters:Float, rateHz:Float) {
		this.scanConnector = scanConnector;
		this.rayCount = rayCount;
		this.maxRangeMeters = maxRangeMeters;
		this.rateHz = rateHz;
	}

	public function check(component:MachineComponent):Void {
		component.connector(scanConnector);
		if (rayCount < 2 || !(maxRangeMeters > 0) || !(rateHz > 0))
			throw 'Planar scanner on "${component.designation}" needs rays, a range and a rate';
	}

	public function describe():String return 'planar scanner $scanConnector $rayCount rays $maxRangeMeters m $rateHz Hz';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<PlanarScannerFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, PlanarScannerFacet)) return cast facet;
		return null;
	}
}
