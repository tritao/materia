package machinekit.standard;

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
			table = new Catalog("hex nut size", spec -> spec.size, rows(), _ -> ({source: "https://www.bossard.com/in-en/eshop/hex-nuts/hex-nuts-type-1/p/1984/", standard: "ISO 4032",
				standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope,
				sources: ["https://www.scribd.com/document/816804171/M-FHN-4032-8-Z-3U-03"],
				verifiedFields: ["acrossFlats", "height"]}));
		return table;
	}

	public static function metric(size:String):HexNut
		return new HexNut(catalog().get(size));

	public static function custom(spec:HexNutSpec):HexNut
		return new HexNut(spec, true);

	private function new(spec:HexNutSpec, codeOnly:Bool = false) {
		if (!(spec.diameter > 0) || !(spec.acrossFlats > spec.diameter) || !(spec.height > 0))
			throw 'Hex nut ${spec.size} has inconsistent dimensions';
		var designation = 'ISO4032-${spec.size}';
		var customName = '${spec.size}-D${Dimension.format(spec.diameter)}-AF${Dimension.format(spec.acrossFlats)}-H${Dimension.format(spec.height)}';
		super(codeOnly ? customDesignation(customName) : designation, 'Hex nut ${spec.size}', "steel 8", codeOnly);
		this.spec = spec;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, spec.height / 2));
		addConnector("back", Face, Solids.axial(0, 0, spec.height));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.prism(cadkit.modeling.Polygon.regular(6, spec.acrossFlats), 0, spec.height);
		if (detail == Envelope) return body;
		return Solids.cut(body, [Part.cylinderSpan(spec.diameter / 2, -0.1, spec.height + 0.1)]);
	}

	/** Cutting tool for a trapped-nut pocket, sized with a 0.5 mm diametral clearance around
	 * the flats, from z=0 to z=depth.
	 */
	public function pocket(depth:Float):Part {
		if (!(depth >= spec.height)) throw 'Nut pocket for ${spec.size} needs depth at least ${spec.height}';
		return Part.prism(cadkit.modeling.Polygon.regular(6, spec.acrossFlats + 0.5), 0, depth);
	}

	function get_acrossFlats():Float return spec.acrossFlats;
	function get_height():Float return spec.height;

	private static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null)
			recipeTypeCache = new ComponentType("machinekit.standard.hex-nut",
			[ComponentRecipeSupport.catalog("size", HexNut.catalog(), "M5")],
			v -> HexNut.metric(v.token("size")),
			true);
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return codeOnly ? null : recipeType();

	override public function values():ComponentValues {
		return new ComponentValues().set("size", Token(this.spec.size)).setToken("material", materialSpec());
	}

}
