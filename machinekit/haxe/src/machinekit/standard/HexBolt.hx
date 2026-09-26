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

/** ISO 4017 hex bolt, threaded to the head. Threads are semantic (diameter, pitch, length), not modelled.
 * CAD frame: head bearing face at z=0, head toward +Z, shank toward -Z, matching
 * `SocketHeadCapScrew`. Connectors: `head` at the bearing face and `tip` at z=-length, both
 * with +Y along +Z. Envelope approximates the head as its across-corners cylinder.
 */
class HexBolt extends MachineComponent {
	static var table:Null<Catalog<HexBoltSpec>>;

	public final spec:HexBoltSpec;
	public final length:Float;
	public var diameter(get, never):Float;
	public var pitch(get, never):Float;
	/** Threaded length measured from the tip; ISO 4017 bolts are threaded over their full length. */
	public var threadLength(get, never):Float;
	/** Distance across the head's corners, for clearance around the hex flats. */
	public var acrossCorners(get, never):Float;

	static function rows():Array<HexBoltSpec>
		return [
			{size: "M3", diameter: 3, pitch: 0.5, acrossFlats: 5.5, headHeight: 2,
				tapDrill: 2.5, clearanceFine: 3.2, clearanceMedium: 3.4, clearanceCoarse: 3.6},
			{size: "M4", diameter: 4, pitch: 0.7, acrossFlats: 7, headHeight: 2.8,
				tapDrill: 3.3, clearanceFine: 4.3, clearanceMedium: 4.5, clearanceCoarse: 4.8},
			{size: "M5", diameter: 5, pitch: 0.8, acrossFlats: 8, headHeight: 3.5,
				tapDrill: 4.2, clearanceFine: 5.3, clearanceMedium: 5.5, clearanceCoarse: 5.8},
			{size: "M6", diameter: 6, pitch: 1.0, acrossFlats: 10, headHeight: 4,
				tapDrill: 5.0, clearanceFine: 6.4, clearanceMedium: 6.6, clearanceCoarse: 7},
			{size: "M8", diameter: 8, pitch: 1.25, acrossFlats: 13, headHeight: 5.3,
				tapDrill: 6.8, clearanceFine: 8.4, clearanceMedium: 9, clearanceCoarse: 10},
			{size: "M10", diameter: 10, pitch: 1.5, acrossFlats: 16, headHeight: 6.4,
				tapDrill: 8.5, clearanceFine: 10.5, clearanceMedium: 11, clearanceCoarse: 12},
			{size: "M12", diameter: 12, pitch: 1.75, acrossFlats: 18, headHeight: 7.5,
				tapDrill: 10.2, clearanceFine: 13, clearanceMedium: 13.5, clearanceCoarse: 14.5},
		];

	public static function catalog():Catalog<HexBoltSpec> {
		if (table == null)
			table = new Catalog("hex bolt size", spec -> spec.size, rows(), _ -> ({source: "MachineKit embedded nominal table; source verification pending", standard: "ISO 4017",
				standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope}));
		return table;
	}

	public static function metric(size:String, length:Float):HexBolt
		return new HexBolt(catalog().get(size), length);

	public function new(spec:HexBoltSpec, length:Float) {
		if (!(length > 0) || !Math.isFinite(length)) throw 'Bolt ${spec.size} needs a positive length';
		if (!(spec.diameter > 0) || !(spec.acrossFlats > spec.diameter) || !(spec.headHeight > 0)
			|| !(spec.tapDrill < spec.diameter) || !(spec.clearanceFine > spec.diameter))
			throw 'Bolt ${spec.size} has inconsistent dimensions';
		var name = '${spec.size}x${Dimension.format(length)}';
		super('ISO4017-$name', 'Hex bolt $name', "steel 8.8");
		this.spec = spec;
		this.length = length;
		addConnector("head", Face, Solids.axial(0, 0, 0));
		addConnector("tip", Face, Solids.axial(0, 0, -length));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var shank = Solids.cylinder(diameter / 2, -length, 0);
		var head = detail == Envelope
			? Solids.cylinder(acrossCorners / 2, 0, spec.headHeight)
			: Solids.prism(Solids.regularPolygon(6, spec.acrossFlats), 0, spec.headHeight);
		return Solids.union([head, shank]);
	}

	public function clearanceDiameter(fit:ClearanceFit = Medium):Float {
		return switch (fit) {
			case Fine: spec.clearanceFine;
			case Medium: spec.clearanceMedium;
			case Coarse: spec.clearanceCoarse;
		}
	}

	/** Through-hole tool from z=0 down to z=-depth. */
	public function clearanceHole(depth:Float, fit:ClearanceFit = Medium):Part
		return Solids.cylinder(clearanceDiameter(fit) / 2, -depth, 0);

	/** Tap-drill tool from z=0 down to z=-depth; the thread itself is not modelled. */
	public function tapHole(depth:Float):Part
		return Solids.cylinder(spec.tapDrill / 2, -depth, 0);

	/** Counterbored through-hole tool, sized with a 0.5 mm diametral clearance around the hex
	 * head's corners and 0.5 mm axial clearance above it. The bolt's `head` sits at z=-headHeight-0.5.
	 */
	public function counterboreHole(depth:Float, fit:ClearanceFit = Medium):Part {
		var seatDepth = spec.headHeight + 0.5;
		if (!(depth > seatDepth)) throw 'Counterbore for ${spec.size} needs depth over $seatDepth';
		return Solids.union([
			Solids.cylinder(clearanceDiameter(fit) / 2, -depth, 0),
			Solids.cylinder((acrossCorners + 0.5) / 2, -seatDepth, 0),
		]);
	}

	function get_diameter():Float return spec.diameter;
	function get_pitch():Float return spec.pitch;
	function get_threadLength():Float return length;
	function get_acrossCorners():Float return spec.acrossFlats / Math.cos(Math.PI / 6);
}
