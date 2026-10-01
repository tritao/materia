package machinekit.motion;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;
import materia.project.MaterialLibrary;

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

	public static function custom(spec:LinearBearingSpec):LinearBearing
		return new LinearBearing(spec, true);

	private function new(spec:LinearBearingSpec, codeOnly:Bool = false) {
		if (!(spec.boreDiameter > 0) || !(spec.outerDiameter > spec.boreDiameter) || !(spec.length > 0))
			throw 'Linear bearing ${spec.designation} has inconsistent dimensions';
		var customName = '${spec.designation}-D${Dimension.format(spec.boreDiameter)}x${Dimension.format(spec.outerDiameter)}x${Dimension.format(spec.length)}';
		super(codeOnly ? customDesignation(customName) : spec.designation,
			'Linear ball bearing ${spec.designation}', "steel", codeOnly);
		this.spec = spec;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, spec.length / 2));
		addConnector("back", Face, Solids.axial(0, 0, spec.length));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var ownedParts:Array<Part> = [];
		return Solids.building(ownedParts, tracked -> {
			var outer = Solids.named(Part.cylinderSpan(outerDiameter / 2, 0, length), "body");
			tracked.push(outer);
			var bore = Solids.named(Part.cylinderSpan(boreDiameter / 2, -0.1, length + 0.1), "bore");
			tracked.push(bore);
			var envelope = Solids.cut(outer, [bore]);
			tracked.push(envelope);
			if (detail == Envelope) return envelope;

			// Preview cuts retaining-ring style end grooves and recessed seal tracks without
			// extending beyond the catalog outside diameter.
			var rimWidth = Math.min(1.2, length / 8);
			var rimDepth = Math.min(0.2, outerDiameter * 0.02);
			var rimOuter = outerDiameter / 2 + 0.05;
			var rimInner = outerDiameter / 2 - rimDepth;
			var frontRim = Solids.named(annulus(rimOuter, rimInner, 0, rimWidth), "rim.front");
			tracked.push(frontRim);
			var backRim = Solids.named(annulus(rimOuter, rimInner, length - rimWidth, length), "rim.back");
			tracked.push(backRim);
			var rims = [frontRim, backRim];
			var detailed = Solids.cut(envelope, rims);
			tracked.push(detailed);

			var sealWidth = Math.min(0.8, length / 12);
			var sealInner = boreDiameter / 2 + Math.min(0.6, (outerDiameter - boreDiameter) * 0.2);
			var sealOuter = Math.min(outerDiameter / 2 - 0.5, sealInner + 0.8);
			if (sealOuter > sealInner) {
				var frontSeal = Solids.named(annulus(sealOuter, sealInner, -0.05, sealWidth), "seal.front");
				tracked.push(frontSeal);
				var backSeal = Solids.named(annulus(sealOuter, sealInner, length - sealWidth, length + 0.05), "seal.back");
				tracked.push(backSeal);
				var seals = [frontSeal, backSeal];
				detailed = Solids.cut(detailed, seals);
				tracked.push(detailed);
			}
			return detailed;
		});
	}

	static function annulus(outerRadius:Float, innerRadius:Float, z0:Float, z1:Float):Part {
		var ownedParts:Array<Part> = [];
		return Solids.building(ownedParts, tracked -> {
			var outer = Solids.named(Part.cylinderSpan(outerRadius, z0, z1), "outer");
			tracked.push(outer);
			var inner = Solids.named(Part.cylinderSpan(innerRadius, z0 - 0.05, z1 + 0.05), "inner");
			tracked.push(inner);
			var result = Solids.cut(outer, [inner]);
			tracked.push(result);
			return result;
		});
	}

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
		return Part.cylinderSpan(housingSeatDiameter(fit) / 2, 0, seatDepth);
	}

	function get_boreDiameter():Float return spec.boreDiameter;
	function get_outerDiameter():Float return spec.outerDiameter;
	function get_length():Float return spec.length;

	private static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null)
			recipeTypeCache = new ComponentType("machinekit.motion.linear-bearing",
			[ComponentRecipeSupport.catalog("designation", LinearBearing.catalog(), "LM8UU")],
			v -> LinearBearing.metric(v.token("designation")),
			true);
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return codeOnly ? null : recipeType();

	override public function values():ComponentValues {
		return new ComponentValues().setToken("designation", this.spec.designation)
			.setToken("material", materialSpec());
	}

}
