package machinekit.pneumatic.schmalz;

import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;

typedef SchmalzFittingSpec = {
	var designation:String;
	var label:String;
	var envelopeDiameterMm:Float;
	var envelopeLengthMm:Float;
	var tubeOdMm:Float;
	var thread:String;
	var massKg:Float;
}

/** Straight G1/4 male to 4 mm OD tube fitting. Cylindrical envelope and
 * centre/inertia are approximations; the published weight is retained. */
class SchmalzPushInFitting extends MachineComponent {
	static var table:Null<Catalog<SchmalzFittingSpec>>;
	public final spec:SchmalzFittingSpec;

	public static function catalog():Catalog<SchmalzFittingSpec> {
		if (table == null) table = new Catalog("Schmalz push-in fitting", row -> row.designation, [{
			designation: "10.08.02.00203", label: "STV-GE G1/4-AG 4",
			envelopeDiameterMm: 8.6, envelopeLengthMm: 19.6,
			tubeOdMm: 4, thread: "G1/4-M", massKg: 0.015
		}], _ -> ({
			source: "https://www.schmalz.com/en-gb/products/vacuum-technology-for-automation-301607/vacuum-components-301608/filters-and-connections-308965/hoses-and-connections-309034/screw-in-push-fittings-309092/10.08.02.00203",
			standard: null, standardEdition: null, dimensionKind: Mixed,
			conformance: NominalEnvelope,
			verifiedFields: ["envelopeDiameterMm", "envelopeLengthMm", "tubeOdMm",
				"thread", "massKg"]
		}));
		return table;
	}

	public function new(designation:String) {
		var row = catalog().get(designation);
		super(row.designation, 'Schmalz ${row.label} vacuum fitting', "brass");
		this.spec = row;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "hose", kind: Vacuum, role: Consumer, iface: PushIn(row.tubeOdMm), required: true});
		addPort({name: "thread", kind: Vacuum, role: Supply, iface: Thread(row.thread), required: false});
		addBridge("hose", "thread");
		var r = row.envelopeDiameterMm / 2, h = row.envelopeLengthMm, m = row.massKg;
		var transverse = m * (3 * r * r + h * h) / 12;
		declareMass(m, new Vector(0, 0, -h / 2),
			new InertiaTensor(transverse, 0, 0, transverse, 0, m * r * r / 2));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.cylinderSpan(spec.envelopeDiameterMm / 2, -spec.envelopeLengthMm, 0);
}
