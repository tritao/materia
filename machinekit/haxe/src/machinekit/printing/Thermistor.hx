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

/** Beta-model NTC thermistor: R = R25 * exp(beta * (1/T - 1/T25)), T in kelvin. */
typedef ThermistorSpec = {
	var designation:String;
	/** Resistance at 25 degrees C, in ohms. */
	var ohmsAt25:Float;
	var beta:Float;
	var maxTemperature:Float;
	/** Diameter of the glass bead and the length of the lead-in, in mm. */
	var beadDiameter:Float;
	var length:Float;
}

/** An NTC temperature sensor. Rows are assumed values for the common 100k, beta 3950 part. */
class Thermistor extends MachineComponent {
	static var table:Null<Catalog<ThermistorSpec>>;
	static var recipe:Null<ComponentType>;
	public static inline var KELVIN:Float = 273.15;
	public final spec:ThermistorSpec;

	public static function catalog():Catalog<ThermistorSpec> {
		if (table == null) table = new Catalog("thermistor", s -> s.designation, [
			{designation: "NTC-100K-B3950", ohmsAt25: 100000.0, beta: 3950.0, maxTemperature: 300.0, beadDiameter: 2.5, length: 20.0}
		], _ -> ({source: "Assumed common 100k NTC, beta 3950; no vendor verification", standard: null, standardEdition: null,
			dimensionKind: Unverified, conformance: GenericApproximation, verifiedFields: []}));
		return table;
	}

	public function new(designation:String = "NTC-100K-B3950") {
		var row = catalog().get(designation);
		super(designation, 'NTC thermistor, ${Std.string(Math.round(row.ohmsAt25 / 1000))}k beta ${Std.string(Math.round(row.beta))}', "plastic");
		spec = row;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("sense", Face, Solids.axial(0, 0, row.length));
		addPort({name: "signal", kind: Signal, role: Supply, iface: Unspecified, required: false});
	}

	/** The sensor's resistance at a temperature, in ohms. */
	public function resistance(celsius:Float):Float {
		if (!(celsius > -KELVIN) || !Math.isFinite(celsius)) throw "Thermistor temperature must be above absolute zero";
		return spec.ohmsAt25 * Math.exp(spec.beta * (1.0 / (celsius + KELVIN) - 1.0 / (25.0 + KELVIN)));
	}

	/** The temperature a resistance reads, in degrees C. */
	public function temperature(ohms:Float):Float {
		if (!(ohms > 0) || !Math.isFinite(ohms)) throw "Thermistor resistance must be positive";
		return 1.0 / (1.0 / (25.0 + KELVIN) + Math.log(ohms / spec.ohmsAt25) / spec.beta) - KELVIN;
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.printing.thermistor", [
			ComponentRecipeSupport.catalog("designation", catalog(), "NTC-100K-B3950")
		], v -> new Thermistor(v.token("designation")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return Std.isExactType(this, Thermistor) ? recipeType() : null;
	override public function values():ComponentValues
		return new ComponentValues().setToken("designation", designation).setToken("material", materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.cylinderSpan(spec.beadDiameter / 2, 0, spec.length), "bead");
}
