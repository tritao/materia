package machinekit.printing;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

typedef HeaterCartridgeSpec = {
	var designation:String;
	var diameter:Float;
	var length:Float;
	/** Rated supply, volts, and the power it draws at that supply, watts. */
	var volts:Float;
	var watts:Float;
	var maxTemperature:Float;
}

/** A resistive cartridge heater. Its resistance follows from its rating: R = V^2 / P. */
class HeaterCartridge extends MachineComponent {
	static var table:Null<Catalog<HeaterCartridgeSpec>>;
	static var recipe:Null<ComponentType>;
	public final spec:HeaterCartridgeSpec;

	public static function catalog():Catalog<HeaterCartridgeSpec> {
		if (table == null) table = new Catalog("heater cartridge", s -> s.designation, [
			{designation: "CARTRIDGE-6x20-24V-40W", diameter: 6.0, length: 20.0, volts: 24.0, watts: 40.0, maxTemperature: 300.0}
		], _ -> ({source: "Assumed common 6 x 20 mm cartridge; no vendor verification", standard: null, standardEdition: null,
			dimensionKind: Unverified, conformance: GenericApproximation, verifiedFields: []}));
		return table;
	}

	public function new(designation:String = "CARTRIDGE-6x20-24V-40W") {
		var row = catalog().get(designation);
		super(designation, 'Heater cartridge, ${Std.string(Math.round(row.volts))} V, ${Std.string(Math.round(row.watts))} W', "steel");
		spec = row;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "power", kind: ElectricalPower, role: Consumer, iface: Unspecified, required: false});
	}

	public function resistance():Float return spec.volts * spec.volts / spec.watts;

	/** The power it draws on `supply` volts, in watts. */
	public function power(supply:Float):Float {
		if (!(supply >= 0) || !Math.isFinite(supply)) throw "Heater supply must be non-negative";
		return supply * supply / resistance();
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.printing.heater-cartridge", [
			ComponentRecipeSupport.catalog("designation", catalog(), "CARTRIDGE-6x20-24V-40W")
		], v -> new HeaterCartridge(v.token("designation")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return Std.isExactType(this, HeaterCartridge) ? recipeType() : null;
	override public function values():ComponentValues
		return new ComponentValues().setToken("designation", designation).setToken("material", materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.cylinderSpan(spec.diameter / 2, 0, spec.length), "body");
}
