package machinekit.pneumatic.schmalz;

import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.Dimension;
import machinekit.pneumatic.RoutedHose;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

typedef SchmalzHoseSpec = {
	var designation:String;
	var label:String;
	var outerDiameterMm:Float;
	var innerDiameterMm:Float;
	var massPerMetreKg:Float;
}

/** A cut and routed length of Schmalz VSL 4-2 PU stock hose. */
class SchmalzVacuumHose extends RoutedHose {
	static var table:Null<Catalog<SchmalzHoseSpec>>;
	public final stock:SchmalzHoseSpec;

	public static function catalog():Catalog<SchmalzHoseSpec> {
		if (table == null) table = new Catalog("Schmalz vacuum hose", row -> row.designation, [{
			designation: "10.07.09.00001", label: "VSL 4-2 PU MI-TR",
			outerDiameterMm: 4, innerDiameterMm: 2, massPerMetreKg: 0.011
		}], _ -> ({
			source: "https://www.schmalz.co.jp/en-jp/products/vacuum-technology-for-automation-301607/vacuum-components-301608/filters-and-connections-308965/hoses-and-connections-309034/vacuum-compressed-air-hoses-vsl-309035/10.07.09.00001",
			standard: null, standardEdition: null, dimensionKind: Nominal,
			conformance: NominalEnvelope,
			verifiedFields: ["outerDiameterMm", "innerDiameterMm", "massPerMetreKg"]
		}));
		return table;
	}

	static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null) recipeTypeCache = new ComponentType("machinekit.pneumatic.schmalz-vacuum-hose",
			[ComponentRecipeSupport.catalog("stock", SchmalzVacuumHose.catalog(), "10.07.09.00001"), ComponentRecipeSupport.text("route", ComponentRecipeSupport.defaultRoute())],
			v -> new SchmalzVacuumHose(v.token("stock"), ComponentRecipeSupport.route(v.token("route"))), true, false, ["route"]);
		return recipeTypeCache;
	}

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, SchmalzVacuumHose) ? recipeType() : null;

	override public function values():machinekit.component.ComponentValues return new machinekit.component.ComponentValues().setToken("stock", stock.designation).setToken("route", ComponentRecipeSupport.routeText(route)).setToken("material", materialSpec());

	public function new(stockDesignation:String, route:Array<Vector>) {
		var row = catalog().get(stockDesignation);
		var length = 0.0;
		if (route != null) for (i in 0...route.length - 1)
			if (route[i] != null && route[i + 1] != null) length += route[i + 1].subtract(route[i]).length();
		super('${row.designation}-L${Dimension.format(length)}', route,
			row.outerDiameterMm, row.innerDiameterMm, row.massPerMetreKg, Vacuum,
			"polyurethane PU", false);
		this.stock = row;
	}
}
