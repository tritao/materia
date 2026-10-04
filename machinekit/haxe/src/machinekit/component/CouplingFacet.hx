package machinekit.component;

/** A coupling half: it mates at `connector` with any half whose `key` is the same. */
class CouplingFacet implements ComponentFacet {
	public final key:String;
	public final connector:String;

	public function new(key:String, connector:String) {
		this.key = key;
		this.connector = connector;
	}

	public function check(component:MachineComponent):Void {
		if (key == null || key.length == 0) throw 'Coupling on "${component.designation}" needs a key';
		component.connector(connector);
	}

	public function describe():String return 'coupling $key at $connector';

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<CouplingFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, CouplingFacet)) return cast facet;
		return null;
	}
}
