package machinekit.standard;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** ISO 4032 hex nut. Threads are semantic (a plain through bore at the nominal diameter).
 * CAD frame: axis along +Z, bottom bearing face at z=0, top face at z=height, matching
 * `DeepGrooveBearing`. Connectors: `front`, `back` (faces) and `axis` (mid-height), all with
 * +Y along +Z.
 */
class HexNut extends MachineComponent {
	static var table:Null<Catalog<HexNutSpec>>;

	public final spec:HexNutSpec;
	public var acrossFlats(get, never):Float;
	public var height(get, never):Float;

	static function rows():Array<HexNutSpec>
		return [
			{size: "M3", diameter: 3, acrossFlats: 5.5, height: 2.4},
			{size: "M4", diameter: 4, acrossFlats: 7, height: 3.2},
			{size: "M5", diameter: 5, acrossFlats: 8, height: 4.7},
			{size: "M6", diameter: 6, acrossFlats: 10, height: 5.2},
			{size: "M8", diameter: 8, acrossFlats: 13, height: 6.8},
			{size: "M10", diameter: 10, acrossFlats: 16, height: 8.4},
			{size: "M12", diameter: 12, acrossFlats: 18, height: 10.8},
		];

	public static function catalog():Catalog<HexNutSpec> {
		if (table == null)
			table = new Catalog("hex nut size", spec -> spec.size, rows());
		return table;
	}

	public static function metric(size:String):HexNut
		return new HexNut(catalog().get(size));

	public function new(spec:HexNutSpec) {
		super('ISO4032-${spec.size}', 'Hex nut ${spec.size}', "steel 8");
		this.spec = spec;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, spec.height / 2));
		addConnector("back", Face, Solids.axial(0, 0, spec.height));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.prism(Solids.regularPolygon(6, spec.acrossFlats), 0, spec.height);
		if (detail == Envelope) return body;
		return Solids.cut(body, [Solids.cylinder(spec.diameter / 2, -0.1, spec.height + 0.1)]);
	}

	/** Cutting tool for a trapped-nut pocket, sized with a 0.5 mm diametral clearance around
	 * the flats, from z=0 to z=depth.
	 */
	public function pocket(depth:Float):Part {
		if (!(depth >= spec.height)) throw 'Nut pocket for ${spec.size} needs depth at least ${spec.height}';
		return Solids.prism(Solids.regularPolygon(6, spec.acrossFlats + 0.5), 0, depth);
	}

	function get_acrossFlats():Float return spec.acrossFlats;
	function get_height():Float return spec.height;
}
