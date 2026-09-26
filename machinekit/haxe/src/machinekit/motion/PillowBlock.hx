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
	public final mountHoleDiameter:Float;
	public final mountScrew:String;

	static function rows():Array<PillowBlockSpec>
		return [
			{designation: "UCP204", family: "KOYO UCP", bearingDesignation: "6204", boreDiameter: 20,
				baseWidth: 38, length: 127, shaftHeight: 33.3, baseHeight: 16, overallHeight: 64.5,
				boltSpacing: 95, mountHoleDiameter: 13, mountScrew: "M10"},
			{designation: "UCP205", family: "KOYO UCP", bearingDesignation: "6205", boreDiameter: 25,
				baseWidth: 38, length: 140, shaftHeight: 36.5, baseHeight: 16, overallHeight: 70,
				boltSpacing: 105, mountHoleDiameter: 13, mountScrew: "M10"},
			{designation: "UCP206", family: "KOYO UCP", bearingDesignation: "6206", boreDiameter: 30,
				baseWidth: 48, length: 165, shaftHeight: 42.9, baseHeight: 17, overallHeight: 84,
				boltSpacing: 121, mountHoleDiameter: 17, mountScrew: "M14"},
			{designation: "UCP207", family: "KOYO UCP", bearingDesignation: "6207", boreDiameter: 35,
				baseWidth: 48, length: 167, shaftHeight: 47.6, baseHeight: 18, overallHeight: 95,
				boltSpacing: 127, mountHoleDiameter: 17, mountScrew: "M14"},
			{designation: "UCP208", family: "KOYO UCP", bearingDesignation: "6208", boreDiameter: 40,
				baseWidth: 54, length: 184, shaftHeight: 49.2, baseHeight: 18, overallHeight: 98,
				boltSpacing: 137, mountHoleDiameter: 17, mountScrew: "M14"},
			{designation: "UCP209", family: "KOYO UCP", bearingDesignation: "6209", boreDiameter: 45,
				baseWidth: 54, length: 190, shaftHeight: 54, baseHeight: 20, overallHeight: 106,
				boltSpacing: 146, mountHoleDiameter: 17, mountScrew: "M14"},
			{designation: "UCP210", family: "KOYO UCP", bearingDesignation: "6210", boreDiameter: 50,
				baseWidth: 60, length: 206, shaftHeight: 57.2, baseHeight: 21, overallHeight: 113,
				boltSpacing: 159, mountHoleDiameter: 20, mountScrew: "M16"},
			{designation: "UCP211", family: "KOYO UCP", bearingDesignation: "6211", boreDiameter: 55,
				baseWidth: 60, length: 219, shaftHeight: 63.5, baseHeight: 23, overallHeight: 125,
				boltSpacing: 171, mountHoleDiameter: 20, mountScrew: "M16"},
			{designation: "UCP212", family: "KOYO UCP", bearingDesignation: "6212", boreDiameter: 60,
				baseWidth: 70, length: 241, shaftHeight: 69.8, baseHeight: 25, overallHeight: 138,
				boltSpacing: 184, mountHoleDiameter: 20, mountScrew: "M16"},
			{designation: "UCP213", family: "KOYO UCP", bearingDesignation: "6213", boreDiameter: 65,
				baseWidth: 70, length: 265, shaftHeight: 76.2, baseHeight: 27, overallHeight: 150,
				boltSpacing: 203, mountHoleDiameter: 25, mountScrew: "M20"}
		];

	/** Koyo/JTEKT UCP204–UCP213 product dimensions, with the generated solids treated as nominal envelopes. */
	public static function catalog():Catalog<PillowBlockSpec> {
		if (table == null)
			table = new Catalog("pillow block unit", spec -> spec.designation, rows(), spec -> ({
				source: 'https://koyo.jtekt.co.jp/en/products/detail/print.php?pno=${spec.designation}',
				standard: "JIS", standardEdition: null, dimensionKind: Nominal, conformance: NominalEnvelope,
				verifiedFields: ["boreDiameter", "baseWidth", "length", "shaftHeight", "baseHeight", "overallHeight",
					"boltSpacing", "mountHoleDiameter", "mountScrew"]
			}));
		return table;
	}

	public static function metric(designation:String):PillowBlock
		return new PillowBlock(catalog().get(designation));

	public function new(spec:PillowBlockSpec) {
		if (!(spec.boreDiameter > 0) || !(spec.baseWidth > 0) || !(spec.length > 0) ||
			!(spec.shaftHeight > spec.baseHeight) || !(spec.overallHeight > spec.shaftHeight) ||
			!(spec.boltSpacing > 0) || spec.boltSpacing >= spec.length || !(spec.mountHoleDiameter > 0) || spec.mountScrew == null)
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
		mountHoleDiameter = spec.mountHoleDiameter;
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
		for (z in [-boltSpacing / 2, boltSpacing / 2])
			tools.push(Solids.cylinderAlongY(mountHoleDiameter / 2, -0.1, baseHeight + 0.1, 0, z));
		return Solids.cut(body, tools);
	}

	/** Socket-head mounting screw sized for this unit's base holes. */
	public function mountScrewPart(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(mountScrew, length);
}
