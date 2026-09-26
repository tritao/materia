package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Align;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.PillowBlockSpec.PillowBlockSpec;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.SocketHeadCapScrew;

/** Base-mounted UCP-style pillow block unit. The housing is a cast base with two mounting
 * holes along the shaft direction and an insert-bearing envelope above the base. CAD frame: the
 * shaft runs along +Z, its centre is at `shaftHeight`, and the base bottom is y=0. Connectors:
 * `axis` at the shaft centre, `input`/`output` at the housing ends, `base` on the mounting face,
 * and `bolt1`/`bolt2` at the two mounting-hole centres. */
class PillowBlock extends MachineComponent {
	static var table:Null<Catalog<PillowBlockSpec>>;

	public final spec:PillowBlockSpec;
	public final bearing:DeepGrooveBearing;
	public final boreDiameter:Float;
	public final baseWidth:Float;
	public final length:Float;
	public final shaftHeight:Float;
	public final baseHeight:Float;
	public final overallHeight:Float;
	public final boltSpacing:Float;
	public final mountScrew:String;

	static function rows():Array<PillowBlockSpec>
		return [{
			designation: "UCP204", family: "KOYO UCP", bearingDesignation: "6204", boreDiameter: 20,
			baseWidth: 38, length: 127, shaftHeight: 33.3, baseHeight: 16, overallHeight: 64.5,
			boltSpacing: 95, mountScrew: "M10"
		}];

	/** Koyo/JTEKT UCP204 product dimensions, with the generated solid treated as a nominal envelope. */
	public static function catalog():Catalog<PillowBlockSpec> {
		if (table == null)
			table = new Catalog("pillow block unit", spec -> spec.designation, rows(), _ -> ({
				source: "https://koyo.jtekt.co.jp/en/products/detail/print.php?pno=UCP204",
				standard: "JIS", standardEdition: null, dimensionKind: Nominal, conformance: NominalEnvelope,
				verifiedFields: ["boreDiameter", "baseWidth", "length", "shaftHeight", "baseHeight", "overallHeight",
					"boltSpacing", "mountScrew"]
			}));
		return table;
	}

	public static function metric(designation:String):PillowBlock
		return new PillowBlock(catalog().get(designation));

	public function new(spec:PillowBlockSpec) {
		if (!(spec.boreDiameter > 0) || !(spec.baseWidth > 0) || !(spec.length > 0) ||
			!(spec.shaftHeight > spec.baseHeight) || !(spec.overallHeight > spec.shaftHeight) ||
			!(spec.boltSpacing > 0) || spec.boltSpacing >= spec.length || spec.mountScrew == null)
			throw 'Pillow block profile "${spec.designation}" has invalid dimensions';
		bearing = DeepGrooveBearing.metric(spec.bearingDesignation);
		if (Math.abs(bearing.bore - spec.boreDiameter) > 1e-9)
			throw 'Pillow block bearing "${spec.bearingDesignation}" bore does not match ${spec.boreDiameter} mm';
		super(spec.designation, '${spec.family} base-mounted pillow block unit', "cast iron");
		this.spec = spec;
		boreDiameter = spec.boreDiameter;
		baseWidth = spec.baseWidth;
		length = spec.length;
		shaftHeight = spec.shaftHeight;
		baseHeight = spec.baseHeight;
		overallHeight = spec.overallHeight;
		boltSpacing = spec.boltSpacing;
		mountScrew = spec.mountScrew;
		addConnector("axis", Axis, Solids.axial(0, shaftHeight, 0));
		addConnector("input", Shaft, Solids.axial(0, shaftHeight, -length / 2));
		addConnector("output", Shaft, Solids.axial(0, shaftHeight, length / 2));
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addConnector("bolt1", Mount, Solids.axial(0, 0, -boltSpacing / 2));
		addConnector("bolt2", Mount, Solids.axial(0, 0, boltSpacing / 2));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var base = Part.box(baseWidth, baseHeight, length, Align.Center, Align.Min, Align.Center);
		var barrelRadius = overallHeight - shaftHeight;
		var barrel = Solids.cylinder(barrelRadius, -length / 2, length / 2, 0, shaftHeight);
		var body = Solids.union([base, barrel]);
		var boreTool = Solids.cylinder(bearing.outside / 2, -length / 2 - 0.1, length / 2 + 0.1, 0, shaftHeight);
		if (detail == Envelope) return Solids.cut(body, [boreTool]);
		var tools = [boreTool];
		var screw = mountScrewPart(10);
		for (z in [-boltSpacing / 2, boltSpacing / 2])
			tools.push(Solids.cylinderAlongY(screw.clearanceDiameter(Medium) / 2, -0.1, baseHeight + 0.1, 0, z));
		return Solids.cut(body, tools);
	}

	/** Socket-head mounting screw sized for this unit's base holes. */
	public function mountScrewPart(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(mountScrew, length);
}
