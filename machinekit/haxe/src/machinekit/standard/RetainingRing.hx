package machinekit.standard;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** DIN 471 external retaining ring, sized by the shaft diameter it fits. The installed ring
 * spans `grooveDiameter`..`outerDiameter`; use `spec.grooveDiameter` and `thickness` to build the
 * matching `SteppedShaft` groove.
 * CAD frame: axis along +Z, spanning z=0..thickness. Connector: `seat` (axis, mid-thickness),
 * with +Y along +Z, matching a `SteppedShaft` groove connector for a fixed mate.
 * Preview leaves a 40 degree gap, matching the ring's open form; Envelope is a full annulus.
 */
class RetainingRing extends MachineComponent {
	static var table:Null<Catalog<RetainingRingSpec>>;
	static inline var GAP:Float = Math.PI * 2 / 9;

	public final spec:RetainingRingSpec;
	public var thickness(get, never):Float;

	static function rows():Array<RetainingRingSpec>
		return [
			{shaftDiameter: 5, grooveDiameter: 4.7, outerDiameter: 8.7, thickness: 0.6},
			{shaftDiameter: 6, grooveDiameter: 5.6, outerDiameter: 10.0, thickness: 0.7},
			{shaftDiameter: 8, grooveDiameter: 7.4, outerDiameter: 12.2, thickness: 0.8},
			{shaftDiameter: 10, grooveDiameter: 9.3, outerDiameter: 15.0, thickness: 1.0},
			{shaftDiameter: 12, grooveDiameter: 11.0, outerDiameter: 18.0, thickness: 1.0},
			{shaftDiameter: 15, grooveDiameter: 14.0, outerDiameter: 21.0, thickness: 1.0},
			{shaftDiameter: 17, grooveDiameter: 16.0, outerDiameter: 24.0, thickness: 1.0},
			{shaftDiameter: 20, grooveDiameter: 19.0, outerDiameter: 27.0, thickness: 1.2},
			{shaftDiameter: 25, grooveDiameter: 23.9, outerDiameter: 34.0, thickness: 1.2},
		];

	public static function catalog():Catalog<RetainingRingSpec> {
		if (table == null)
			table = new Catalog("retaining ring shaft diameter", spec -> Std.string(Std.int(spec.shaftDiameter)), rows());
		return table;
	}

	public static function forShaft(shaftDiameter:Float):RetainingRing
		return new RetainingRing(catalog().get(Std.string(Std.int(shaftDiameter))));

	public function new(spec:RetainingRingSpec) {
		super('DIN471-${spec.shaftDiameter}', 'External retaining ring for ${spec.shaftDiameter} mm shaft',
			"spring steel");
		this.spec = spec;
		addConnector("seat", Axis, Solids.axial(0, 0, spec.thickness / 2));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var id = spec.grooveDiameter / 2, od = spec.outerDiameter / 2;
		var angle = detail == Envelope ? Math.PI * 2 : Math.PI * 2 - GAP;
		return Solids.revolve([{r: id, z: 0}, {r: od, z: 0}, {r: od, z: spec.thickness}, {r: id, z: spec.thickness}], angle);
	}

	function get_thickness():Float return spec.thickness;
}
