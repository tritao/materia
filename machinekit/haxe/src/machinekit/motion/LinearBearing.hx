package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.BearingFit;
import machinekit.standard.BearingFit.BearingHousingFit;
import machinekit.standard.BearingFit.BearingShaftFit;

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
			table = new Catalog("linear bearing", spec -> spec.designation, rows(), _ -> ({
				source: "https://www.tuli.si/media/custom/upload/Linear_bushings_LM_LME.pdf",
				standard: null, standardEdition: null, dimensionKind: Nominal, conformance: NominalEnvelope,
				verifiedFields: ["boreDiameter", "outerDiameter", "length"]}));
		return table;
	}

	public static function metric(designation:String):LinearBearing
		return new LinearBearing(catalog().get(designation));

	public function new(spec:LinearBearingSpec) {
		if (!(spec.boreDiameter > 0) || !(spec.outerDiameter > spec.boreDiameter) || !(spec.length > 0))
			throw 'Linear bearing ${spec.designation} has inconsistent dimensions';
		super(spec.designation, 'Linear ball bearing ${spec.designation}', "steel");
		this.spec = spec;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, spec.length / 2));
		addConnector("back", Face, Solids.axial(0, 0, spec.length));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cut(Solids.cylinder(outerDiameter / 2, 0, length),
			[Solids.cylinder(boreDiameter / 2, -0.1, length + 0.1)]);

	/** Diameter of the round guide rod for a named shaft fit. The allowance is diametral. */
	public function guideRodDiameter(fit:BearingShaftFit = Slip):Float
		return boreDiameter + BearingFit.shaftAllowance(fit, boreDiameter);

	/** Diameter of the carriage seat for a named housing fit. The allowance is diametral. */
	public function housingSeatDiameter(fit:BearingHousingFit = Slip):Float
		return outerDiameter + BearingFit.housingAllowance(fit, outerDiameter);

	/** Cylindrical carriage seat tool for this bearing and housing fit. */
	public function housingSeat(?depth:Float, fit:BearingHousingFit = Slip):Part {
		var seatDepth = depth == null ? length : depth;
		if (!(seatDepth > 0) || !Math.isFinite(seatDepth)) throw "Linear bearing housing seat depth must be positive";
		return Solids.cylinder(housingSeatDiameter(fit) / 2, 0, seatDepth);
	}

	function get_boreDiameter():Float return spec.boreDiameter;
	function get_outerDiameter():Float return spec.outerDiameter;
	function get_length():Float return spec.length;
}
