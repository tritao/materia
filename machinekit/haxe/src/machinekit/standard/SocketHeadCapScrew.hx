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

/** ISO 4762 socket head cap screw. Threads are semantic (diameter, pitch, length), not modelled.
 * CAD frame: head bearing face at z=0, head toward +Z, shank toward -Z.
 * Connectors: `head` at the bearing face and `tip` at z=-length, both with +Y along +Z.
 * Hole tools share the convention: the seat surface is z=0 and material lies at z<0.
 */
class SocketHeadCapScrew extends MachineComponent {
	static var table:Null<Catalog<MetricScrewSpec>>;

	public final spec:MetricScrewSpec;
	public final length:Float;
	public var diameter(get, never):Float;
	public var pitch(get, never):Float;
	/** Threaded length measured from the tip. */
	public var threadLength(get, never):Float;

	static function rows():Array<MetricScrewSpec>
		return [
			row("M3", 3, 0.5, 5.5, 3, 2.5, 1.3, 18, 2.5, 3.2, 3.4, 3.6, 6.5, 3.4),
			row("M4", 4, 0.7, 7, 4, 3, 2, 20, 3.3, 4.3, 4.5, 4.8, 8, 4.4),
			row("M5", 5, 0.8, 8.5, 5, 4, 2.5, 22, 4.2, 5.3, 5.5, 5.8, 10, 5.4),
			row("M6", 6, 1.0, 10, 6, 5, 3, 24, 5.0, 6.4, 6.6, 7, 11, 6.4),
			row("M8", 8, 1.25, 13, 8, 6, 4, 28, 6.8, 8.4, 9, 10, 15, 8.6),
			row("M10", 10, 1.5, 16, 10, 8, 5, 32, 8.5, 10.5, 11, 12, 18, 10.6),
			row("M12", 12, 1.75, 18, 12, 10, 6, 36, 10.2, 13, 13.5, 14.5, 20, 12.6),
			row("M14", 14, 2.0, 21, 14, 12, 7, 40, 12.0, 15, 15.5, 16, 24, 14.8),
			row("M16", 16, 2.0, 24, 16, 14, 8, 44, 14.0, 17, 17.5, 18, 27, 16.8),
			row("M20", 20, 2.5, 30, 20, 17, 10, 52, 17.5, 21, 22, 24, 33, 21),
		];

	public static function catalog():Catalog<MetricScrewSpec> {
		if (table == null)
			table = new Catalog("metric screw size", spec -> spec.size, rows(), spec -> switch (spec.size) {
				case "M5": ({source: "https://www.accu.co.uk/api/product-datasheet?id=652689", standard: "ISO 4762",
					standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope,
					sources: ["https://norelem.co.uk/medias/Technische-Hinweise-Schrauben-Muttern-EN.pdf?context=bWFzdGVyfHJvb3R8MjAxMTYxfGFwcGxpY2F0aW9uL3BkZnxhR1UyTDJoaU1pODVNamc1TURNd05UTXpNVFV3TDFSbFkyaHVhWE5qYUdVdFNHbHVkMlZwYzJVdFUyTm9jbUYxWW1WdUxVMTFkSFJsY201ZlJVNHVjR1JtfDNkOGZmNzZiMzAyMDRjZGQzMzIzZWIzNGEzY2I5NjA0MzkxNjY2ZTdkMDNmZjU2NDg2YWY3N2YyODUxZmFlMTA"],
					verifiedFields: ["diameter", "pitch", "headDiameter", "headHeight", "socketSize", "socketDepth", "threadLength",
						"tapDrill", "clearanceFine", "clearanceMedium", "counterboreDiameter"]});
				case "M14": ({source: "https://www.accu.co.uk/metric-cap-head-screws/3248-SSC-M14-65-A4", standard: "ISO 4762",
					standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope,
					verifiedFields: ["diameter", "pitch", "headDiameter", "headHeight", "socketSize", "socketDepth", "threadLength"]});
				case "M16": ({source: "https://www.accu.co.uk/metric-cap-head-screws/3260-SSC-M16-65-A4", standard: "ISO 4762",
					standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope,
					verifiedFields: ["diameter", "pitch", "headDiameter", "headHeight", "socketSize", "socketDepth", "threadLength"]});
				case "M20": ({source: "https://www.accu.co.uk/metric-cap-head-screws/15854-SSC-M20-75-A2", standard: "ISO 4762",
					standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope,
					verifiedFields: ["diameter", "pitch", "headDiameter", "headHeight", "socketSize", "socketDepth", "threadLength"]});
				default: ({source: "MachineKit embedded nominal table; source verification pending", standard: "ISO 4762",
					standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope})
			});
		return table;
	}

	public static function metric(size:String, length:Float, material:String = "steel 12.9"):SocketHeadCapScrew
		return new SocketHeadCapScrew(catalog().get(size), length, material);

	public function new(spec:MetricScrewSpec, length:Float, material:String = "steel 12.9") {
		if (!(length > 0) || !Math.isFinite(length)) throw 'Screw ${spec.size} needs a positive length';
		if (!(spec.diameter > 0) || !(spec.headDiameter > spec.diameter) || !(spec.headHeight > spec.socketDepth)
			|| !(spec.tapDrill < spec.diameter) || !(spec.clearanceFine > spec.diameter)
			|| !(spec.counterboreDiameter > spec.headDiameter) || !(spec.counterboreDepth > spec.headHeight))
			throw 'Screw ${spec.size} has inconsistent dimensions';
		var name = '${spec.size}x${Dimension.format(length)}';
		super('ISO4762-$name', 'Socket head cap screw $name', material);
		this.spec = spec;
		this.length = length;
		addConnector("head", Face, Solids.axial(0, 0, 0));
		addConnector("tip", Face, Solids.axial(0, 0, -length));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var head = Solids.cylinder(spec.headDiameter / 2, 0, spec.headHeight);
		var shank = Solids.cylinder(diameter / 2, -length, 0);
		var body = Solids.union([head, shank]);
		if (detail == Envelope) return body;
		var top = spec.headHeight;
		var socket = Solids.prism(Solids.regularPolygon(6, spec.socketSize), top - spec.socketDepth, top + 0.1);
		return Solids.cut(body, [socket]);
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

	/** Counterbored through-hole tool. The screw's `head` sits at z=-counterboreDepth. */
	public function counterboreHole(depth:Float, fit:ClearanceFit = Medium):Part {
		if (!(depth > spec.counterboreDepth)) throw 'Counterbore for ${spec.size} needs depth over ${spec.counterboreDepth}';
		return Solids.union([
			Solids.cylinder(clearanceDiameter(fit) / 2, -depth, 0),
			Solids.cylinder(spec.counterboreDiameter / 2, -spec.counterboreDepth, 0),
		]);
	}

	function get_diameter():Float return spec.diameter;
	function get_pitch():Float return spec.pitch;
	function get_threadLength():Float return Math.min(length, spec.threadLength);

	static function row(size:String, diameter:Float, pitch:Float, headDiameter:Float, headHeight:Float,
			socketSize:Float, socketDepth:Float, threadLength:Float, tapDrill:Float, fine:Float,
			medium:Float, coarse:Float, counterboreDiameter:Float, counterboreDepth:Float):MetricScrewSpec {
		return {size: size, diameter: diameter, pitch: pitch, headDiameter: headDiameter,
			headHeight: headHeight, socketSize: socketSize, socketDepth: socketDepth,
			threadLength: threadLength, tapDrill: tapDrill, clearanceFine: fine,
			clearanceMedium: medium, clearanceCoarse: coarse,
			counterboreDiameter: counterboreDiameter, counterboreDepth: counterboreDepth};
	}
}
