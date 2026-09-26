package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** LM-series linear ball bearing, for a round rail. Unlike `Bushing`, sizes come from a fixed
 * catalog rather than a proportional formula.
 * CAD frame: axis along +Z, front face at z=0, back face at z=length, matching
 * `DeepGrooveBearing`. Connectors: `front`, `back` (faces) and `axis` (mid-length), all with +Y
 * along +Z.
 */
class LinearBearing extends MachineComponent {
	static var table:Null<Catalog<LinearBearingSpec>>;

	public final spec:LinearBearingSpec;
	public var boreDiameter(get, never):Float;
	public var outerDiameter(get, never):Float;
	public var length(get, never):Float;

	static function rows():Array<LinearBearingSpec>
		return [
			{designation: "LM8UU", boreDiameter: 8, outerDiameter: 15, length: 24},
			{designation: "LM10UU", boreDiameter: 10, outerDiameter: 19, length: 29},
			{designation: "LM12UU", boreDiameter: 12, outerDiameter: 21, length: 30},
			{designation: "LM16UU", boreDiameter: 16, outerDiameter: 28, length: 37},
			{designation: "LM20UU", boreDiameter: 20, outerDiameter: 32, length: 42},
		];

	public static function catalog():Catalog<LinearBearingSpec> {
		if (table == null)
			table = new Catalog("linear bearing", spec -> spec.designation, rows());
		return table;
	}

	public static function metric(designation:String):LinearBearing
		return new LinearBearing(catalog().get(designation));

	public function new(spec:LinearBearingSpec) {
		super(spec.designation, 'Linear ball bearing ${spec.designation}', "steel");
		this.spec = spec;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, spec.length / 2));
		addConnector("back", Face, Solids.axial(0, 0, spec.length));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cut(Solids.cylinder(outerDiameter / 2, 0, length),
			[Solids.cylinder(boreDiameter / 2, -0.1, length + 0.1)]);

	function get_boreDiameter():Float return spec.boreDiameter;
	function get_outerDiameter():Float return spec.outerDiameter;
	function get_length():Float return spec.length;
}
