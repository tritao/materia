package machinekit.pneumatic.schmalz;

import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.PortInterface;
import machinekit.pneumatic.VacuumGenerator;

typedef SchmalzEjectorSpec = {
	var designation:String;
	var label:String;
	var widthMm:Float;
	var heightMm:Float;
	var envelopeLengthMm:Float;
	var massKg:Float;
	var degreeOfEvacuationPercent:Float;
	var optimalAirPressureBar:Float;
	var suctionRateLMin:Float;
	var airConsumptionLMin:Float;
}

/** Schmalz basic ejector. The box includes its silencer; mass centre and
 * inertia use the box approximation. The 85% evacuation rating becomes
 * 85 kPa against a conservative 100 kPa ambient reference. */
class SchmalzVacuumGenerator extends VacuumGenerator {
	static var table:Null<Catalog<SchmalzEjectorSpec>>;
	public final spec:SchmalzEjectorSpec;

	public static function catalog():Catalog<SchmalzEjectorSpec> {
		if (table == null) table = new Catalog("Schmalz vacuum generator", row -> row.designation, [{
			designation: "10.02.01.00563", label: "SBP 05 S01 SDA",
			widthMm: 10, heightMm: 28, envelopeLengthMm: 71, massKg: 0.0075,
			degreeOfEvacuationPercent: 85, optimalAirPressureBar: 4.5,
			suctionRateLMin: 8, airConsumptionLMin: 13.5
		}], _ -> ({
			source: "https://www.schmalz.com/en/vacuum-technology-for-automation/vacuum-components/vacuum-generators/basic-ejectors/basic-ejectors-sbp-307660/10.02.01.00563/",
			standard: null, standardEdition: null, dimensionKind: Mixed,
			conformance: NominalEnvelope,
			verifiedFields: ["widthMm", "heightMm", "envelopeLengthMm", "massKg",
				"degreeOfEvacuationPercent", "optimalAirPressureBar",
				"suctionRateLMin", "airConsumptionLMin"]
		}));
		return table;
	}

	public function new(designation:String) {
		var row = catalog().get(designation);
		super(row.degreeOfEvacuationPercent, row.designation, PushIn(4), PushIn(4),
			'Schmalz ${row.label} basic ejector');
		this.spec = row;
		setMaterial("plastic");
		var x = row.widthMm, y = row.heightMm, z = row.envelopeLengthMm, m = row.massKg;
		declareMass(m, new Vector(0, 0, z / 2),
			new InertiaTensor(m * (y * y + z * z) / 12, 0, 0,
				m * (x * x + z * z) / 12, 0, m * (x * x + y * y) / 12));
	}

	public static function recipeType():machinekit.component.ComponentType
		return machinekit.component.MachineKitAdditionalRecipes.byId("machinekit.pneumatic.schmalz-vacuum-generator");

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<machinekit.component.ComponentType>
		return Std.isExactType(this, SchmalzVacuumGenerator) ? recipeType() : null;

	override public function values():machinekit.component.ComponentValues return new machinekit.component.ComponentValues().setToken("designation", spec.designation).setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(spec.widthMm, spec.heightMm, spec.envelopeLengthMm);
}
