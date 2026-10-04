package machinekit.pneumatic;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** A suction contact: vacuum at `vacuumPort` holds work against `contactConnector`. */
class SuctionFacet implements ComponentFacet {
	/** Sealed area in mm², when known. */
	public final effectiveAreaMm2:Null<Float>;
	/** Rated tilting moment in N m, when known. */
	public final ratedMomentNm:Null<Float>;
	public final vacuumPort:String;
	public final contactConnector:String;

	public function new(effectiveAreaMm2:Null<Float>, ratedMomentNm:Null<Float>, vacuumPort:String, contactConnector:String) {
		this.effectiveAreaMm2 = effectiveAreaMm2;
		this.ratedMomentNm = ratedMomentNm;
		this.vacuumPort = vacuumPort;
		this.contactConnector = contactConnector;
	}

	public function check(component:MachineComponent):Void {
		component.port(vacuumPort);
		component.connector(contactConnector);
	}

	public function describe():String return 'suction $effectiveAreaMm2 mm² $ratedMomentNm N m, $vacuumPort to $contactConnector';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<SuctionFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, SuctionFacet)) return cast facet;
		return null;
	}
}
