package machinekit.robotics.schmalz;

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

typedef SchmalzSxtMasterSpec = {
	var designation:String;
	var label:String;
	var nominalPipeDiameterMm:Float;
	var channels:Int;
	var widthMm:Float;
	var depthMm:Float;
	var lengthMm:Float;
	var massKg:Float;
	var outsideThread:String;
}

/** Female, robot-side Schmalz SXT 40 bayonet with three pneumatic passages.
 * Its box envelope and mass centre are estimates from published bounds. */
class SchmalzSxtMaster extends MachineComponent {
	static var table:Null<Catalog<SchmalzSxtMasterSpec>>;
	public final spec:SchmalzSxtMasterSpec;

	public static function catalog():Catalog<SchmalzSxtMasterSpec> {
		if (table == null) table = new Catalog("Schmalz SXT robot-side bayonet", row -> row.designation, [{
			designation: "10.07.13.00013", label: "SXT-QC-CON-40-F-3",
			nominalPipeDiameterMm: 40, channels: 3, widthMm: 109, depthMm: 85,
			lengthMm: 133, massKg: 1.7, outsideThread: "G1/8-F"
		}], _ -> ({
			source: "https://www.schmalz.com/en-us/products/vacuum-technology-for-automation-301607/vacuum-components-301608/mounting-elements-307004/tooling-system-sxt-307259/quick-change-robot-bayonets-sxt-qc-con-307260/10.07.13.00013",
			standard: null, standardEdition: null, dimensionKind: Mixed,
			conformance: NominalEnvelope,
			verifiedFields: ["nominalPipeDiameterMm", "channels", "widthMm", "depthMm",
				"lengthMm", "massKg", "outsideThread"]
		}));
		return table;
	}


	public function new(designation:String) {
		var row = catalog().get(designation);
		super(row.designation, 'Schmalz ${row.label} robot-side bayonet', "aluminium 6061");
		this.spec = row;
		addConnector("robot", Mount, Solids.axial(0, 0, 0));
		addConnector("tool", Mount, Solids.axial(0, 0, row.lengthMm));
		for (i in 1...row.channels + 1) {
			addPort({name: 'airIn$i', kind: Pneumatic, role: Consumer,
				iface: Thread(row.outsideThread), required: false});
			addPort({name: 'airOut$i', kind: Pneumatic, role: Supply,
				iface: Coupling('schmalz:sxt:${spec.nominalPipeDiameterMm}:${spec.channels}', i), required: false});
			addBridge('airIn$i', 'airOut$i');
		}
		addCapability(Coupling('schmalz:sxt:${spec.nominalPipeDiameterMm}:${spec.channels}', "tool"));
		var x = row.widthMm, y = row.depthMm, z = row.lengthMm, m = row.massKg;
		declareMass(m, new Vector(0, 0, z / 2),
			new InertiaTensor(m * (y * y + z * z) / 12, 0, 0,
				m * (x * x + z * z) / 12, 0, m * (x * x + y * y) / 12));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(spec.widthMm, spec.depthMm, spec.lengthMm);
}
