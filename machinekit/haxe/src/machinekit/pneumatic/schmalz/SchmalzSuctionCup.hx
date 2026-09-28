package machinekit.pneumatic.schmalz;

import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.PortInterface;
import machinekit.pneumatic.SuctionCup;

typedef SchmalzCupSpec = {
	var designation:String;
	var label:String;
	var nominalDiameterMm:Float;
	var envelopeDiameterMm:Float;
	var heightMm:Float;
	var massKg:Float;
	var theoreticalForceAt60KpaN:Float;
	var vacuumThread:String;
}

/** Schmalz cup with source mass and dimensions. Centre and inertia are
 * approximated by a solid cylinder; sealed area is inferred from the
 * manufacturer's theoretical force at 60 kPa, not a measured guarantee. */
class SchmalzSuctionCup extends SuctionCup {
	static var table:Null<Catalog<SchmalzCupSpec>>;
	public final spec:SchmalzCupSpec;

	public static function catalog():Catalog<SchmalzCupSpec> {
		if (table == null) table = new Catalog("Schmalz suction cup", row -> row.designation, [{
			designation: "10.01.01.11401", label: "SAF 40 NBR-45 G1/4-IG",
			nominalDiameterMm: 40, envelopeDiameterMm: 46, heightMm: 22,
			massKg: 0.0136, theoreticalForceAt60KpaN: 69, vacuumThread: 'G1/4-F'
		}], _ -> ({
			source: "https://www.schmalz.co.jp/en-jp/products/vacuum-technology-for-automation-301607/vacuum-components-301608/vacuum-suction-cups-301609/flat-suction-cups-round-301610/flat-suction-cups-saf-302308/10.01.01.11401",
			standard: null, standardEdition: null, dimensionKind: Mixed,
			conformance: NominalEnvelope,
			verifiedFields: ["nominalDiameterMm", "envelopeDiameterMm", "heightMm",
				"massKg", "theoreticalForceAt60KpaN", "vacuumThread"]
		}));
		return table;
	}

	public function new(designation:String) {
		var row = catalog().get(designation);
		var areaMm2 = row.theoreticalForceAt60KpaN / 60000 * 1e6;
		super(row.nominalDiameterMm, row.heightMm, areaMm2, null,
			row.designation, Thread(row.vacuumThread), 'Schmalz ${row.label} suction cup');
		this.spec = row;
		setMaterial("nitrile rubber NBR");
		var radius = row.envelopeDiameterMm / 2;
		var transverse = row.massKg * (3 * radius * radius + row.heightMm * row.heightMm) / 12;
		var axial = row.massKg * radius * radius / 2;
		declareMass(row.massKg, new Vector(0, 0, row.heightMm / 2),
			new InertiaTensor(transverse, 0, 0, transverse, 0, axial));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.cylinderSpan(spec.envelopeDiameterMm / 2, 0, spec.heightMm);
}
