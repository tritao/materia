package machinekit.standard;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** ISO 7089 plain washer.
 * CAD frame: axis along +Z, bottom face at z=0, top face at z=thickness, matching `HexNut`.
 * Connectors: `front`, `back` (faces) and `axis` (mid-thickness), all with +Y along +Z.
 */
class FlatWasher extends MachineComponent {
	static var table:Null<Catalog<FlatWasherSpec>>;

	public final spec:FlatWasherSpec;
	public var innerDiameter(get, never):Float;
	public var outerDiameter(get, never):Float;
	public var thickness(get, never):Float;

	static function rows():Array<FlatWasherSpec>
		return [
			{size: "M3", innerDiameter: 3.2, outerDiameter: 7, thickness: 0.5},
			{size: "M4", innerDiameter: 4.3, outerDiameter: 9, thickness: 0.8},
			{size: "M5", innerDiameter: 5.3, outerDiameter: 10, thickness: 1.0},
			{size: "M6", innerDiameter: 6.4, outerDiameter: 12, thickness: 1.6},
			{size: "M8", innerDiameter: 8.4, outerDiameter: 16, thickness: 1.6},
			{size: "M10", innerDiameter: 10.5, outerDiameter: 20, thickness: 2.0},
			{size: "M12", innerDiameter: 13, outerDiameter: 24, thickness: 2.5},
		];

	public static function catalog():Catalog<FlatWasherSpec> {
		if (table == null)
			table = new Catalog("flat washer size", spec -> spec.size, rows());
		return table;
	}

	public static function metric(size:String):FlatWasher
		return new FlatWasher(catalog().get(size));

	public function new(spec:FlatWasherSpec) {
		if (!(spec.innerDiameter > 0) || !(spec.outerDiameter > spec.innerDiameter) || !(spec.thickness > 0))
			throw 'Flat washer ${spec.size} has inconsistent dimensions';
		super('ISO7089-${spec.size}', 'Flat washer ${spec.size}', "steel");
		this.spec = spec;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, spec.thickness / 2));
		addConnector("back", Face, Solids.axial(0, 0, spec.thickness));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cut(Solids.cylinder(outerDiameter / 2, 0, thickness),
			[Solids.cylinder(innerDiameter / 2, -0.1, thickness + 0.1)]);

	function get_innerDiameter():Float return spec.innerDiameter;
	function get_outerDiameter():Float return spec.outerDiameter;
	function get_thickness():Float return spec.thickness;
}
