package machinekit.standard;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** ISO 15 boundary dimensions in millimetres; `chamfer` is the minimum corner radius r_s. */
typedef DeepGrooveBearingSpec = {
	var designation:String;
	var bore:Float;
	var outside:Float;
	var width:Float;
	var chamfer:Float;
}

/** Radial ball bearing with exact boundary dimensions and simplified rings (no balls or cage).
 * CAD frame: axis along +Z, front face at z=0, back face at z=width.
 * Connectors: `front`, `back` (faces) and `axis` (mid-width), all with +Y along +Z.
 */
class DeepGrooveBearing extends MachineComponent {
	static var table:Null<Catalog<DeepGrooveBearingSpec>>;

	public final spec:DeepGrooveBearingSpec;
	public final shielded:Bool;
	public var bore(get, never):Float;
	public var outside(get, never):Float;
	public var width(get, never):Float;

	static function rows():Array<DeepGrooveBearingSpec>
		return [
			{designation: "625", bore: 5, outside: 16, width: 5, chamfer: 0.3},
			{designation: "626", bore: 6, outside: 19, width: 6, chamfer: 0.3},
			{designation: "608", bore: 8, outside: 22, width: 7, chamfer: 0.3},
			{designation: "6000", bore: 10, outside: 26, width: 8, chamfer: 0.3},
			{designation: "6001", bore: 12, outside: 28, width: 8, chamfer: 0.3},
			{designation: "6002", bore: 15, outside: 32, width: 9, chamfer: 0.3},
			{designation: "6003", bore: 17, outside: 35, width: 10, chamfer: 0.3},
			{designation: "6004", bore: 20, outside: 42, width: 12, chamfer: 0.6},
			{designation: "6200", bore: 10, outside: 30, width: 9, chamfer: 0.6},
			{designation: "6201", bore: 12, outside: 32, width: 10, chamfer: 0.6},
			{designation: "6202", bore: 15, outside: 35, width: 11, chamfer: 0.6},
			{designation: "6203", bore: 17, outside: 40, width: 12, chamfer: 0.6},
			{designation: "6204", bore: 20, outside: 47, width: 14, chamfer: 1.0},
			{designation: "6205", bore: 25, outside: 52, width: 15, chamfer: 1.0},
		];

	public static function catalog():Catalog<DeepGrooveBearingSpec> {
		if (table == null)
			table = new Catalog("deep groove bearing", spec -> spec.designation, rows(), spec -> switch (spec.designation) {
				case "608" | "6000": {source: 'https://eshop.ntn-snr.com/en/product/${spec.designation}-NTN/${spec.designation}',
					standard: "ISO 15", standardEdition: null, dimensionKind: Mixed, conformance: NominalEnvelope};
				default: {source: "MachineKit embedded nominal table; source verification pending", standard: "ISO 15",
					standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope};
			});
		return table;
	}

	/** `designation` is the basic series number; shielded bearings add the "-2Z" suffix. */
	public static function metric(designation:String, shielded:Bool = true):DeepGrooveBearing
		return new DeepGrooveBearing(catalog().get(designation), shielded);

	public function new(spec:DeepGrooveBearingSpec, shielded:Bool = true) {
		if (!(spec.bore > 0) || !(spec.outside > spec.bore) || !(spec.width > 0) ||
			!(spec.chamfer >= 0) || 2 * spec.chamfer >= Math.min(spec.width, (spec.outside - spec.bore) / 2))
			throw 'Invalid deep groove bearing "${spec.designation}"';
		super(spec.designation + (shielded ? "-2Z" : ""),
			'Deep groove ball bearing ${spec.designation}${shielded ? " shielded" : ""} ' +
			'${Dimension.format(spec.bore)}x${Dimension.format(spec.outside)}x${Dimension.format(spec.width)}',
			"bearing steel");
		this.spec = spec;
		this.shielded = shielded;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, spec.width / 2));
		addConnector("back", Face, Solids.axial(0, 0, spec.width));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var ri = bore / 2, ro = outside / 2, b = width;
		if (detail == Envelope)
			return Solids.revolve([{r: ri, z: 0}, {r: ro, z: 0}, {r: ro, z: b}, {r: ri, z: b}]);
		// Ring shoulders at 30% of the section height; the recess shows the shield or ball gap.
		var c = spec.chamfer, section = ro - ri;
		var a = ri + 0.3 * section, o = ro - 0.3 * section;
		var e = shielded ? Math.min(0.5, 0.06 * b) : 0.2 * b;
		return Solids.revolve([
			{r: ri + c, z: 0}, {r: a, z: 0}, {r: a, z: e}, {r: o, z: e}, {r: o, z: 0},
			{r: ro - c, z: 0}, {r: ro, z: c}, {r: ro, z: b - c}, {r: ro - c, z: b},
			{r: o, z: b}, {r: o, z: b - e}, {r: a, z: b - e}, {r: a, z: b},
			{r: ri + c, z: b}, {r: ri, z: b - c}, {r: ri, z: c},
		]);
	}

	/** Cutting tool for a housing bore in the bearing's frame, from the front face to `depth`.
	 * `allowance` is diametral: negative for interference, positive for a slip fit.
	 */
	public function housingSeat(?depth:Float, allowance:Float = 0):Part {
		var length = depth == null ? width : depth;
		return Solids.cylinder((outside + allowance) / 2, 0, length);
	}

	/** Shaft journal diameter for a diametral allowance (positive for interference). */
	public function journalDiameter(allowance:Float = 0):Float return bore + allowance;

	function get_bore():Float return spec.bore;
	function get_outside():Float return spec.outside;
	function get_width():Float return spec.width;
}
